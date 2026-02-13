"""
Minimap System for Real-time Drone Mapping
Shows discovered walls, doors, and objects as the drone learns about them
"""

import numpy as np
import math
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass
from enum import Enum

class FeatureType(Enum):
    """Types of features that can be discovered and mapped."""
    WALL = "wall"
    DOOR = "door" 
    OBJECT = "object"
    IED = "ied"
    UNKNOWN_OBSTACLE = "unknown"

@dataclass
class DiscoveredFeature:
    """A feature discovered by the drone's sensors."""
    feature_type: FeatureType
    position: Tuple[float, float]  # (x, y) world coordinates
    confidence: float  # 0.0 to 1.0
    timestamp: float
    additional_data: Dict = None  # Extra info like size, orientation, etc.

class MinimapSystem:
    """
    Real-time minimap that builds as the drone explores.
    Only shows what the drone has actually discovered through its sensors.
    """
    
    def __init__(self, world_bounds: Tuple[float, float, float, float] = (-5, -5, 55, 45)):
        """
        Initialize minimap system.
        
        Args:
            world_bounds: (min_x, min_y, max_x, max_y) of the world to map
        """
        self.world_bounds = world_bounds
        self.min_x, self.min_y, self.max_x, self.max_y = world_bounds
        
        # Resolution for discretized mapping (meters per cell)
        self.resolution = 0.5  # 0.5m per cell
        
        # Calculate grid dimensions
        self.width_cells = int((self.max_x - self.min_x) / self.resolution)
        self.height_cells = int((self.max_y - self.min_y) / self.resolution)
        
        # Discovered features storage
        self.discovered_features: List[DiscoveredFeature] = []
        self.feature_grid: Dict[Tuple[int, int], List[DiscoveredFeature]] = {}
        
        # Wall segments discovered (as line segments)
        self.discovered_walls: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
        
        # Door locations discovered
        self.discovered_doors: List[Tuple[float, float]] = []
        
        # Objects discovered
        self.discovered_objects: List[DiscoveredFeature] = []
        
        # IEDs discovered
        self.discovered_ieds: List[DiscoveredFeature] = []
        
        # Areas that have been explored (for coverage visualization)
        self.explored_cells: Set[Tuple[int, int]] = set()
        
        # Confidence thresholds for feature acceptance
        self.wall_confidence_threshold = 0.7
        self.door_confidence_threshold = 0.6
        self.object_confidence_threshold = 0.5
        self.ied_confidence_threshold = 0.3
        
    def world_to_grid(self, world_pos: Tuple[float, float]) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        x, y = world_pos
        grid_x = int((x - self.min_x) / self.resolution)
        grid_y = int((y - self.min_y) / self.resolution)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_pos: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        grid_x, grid_y = grid_pos
        world_x = self.min_x + grid_x * self.resolution
        world_y = self.min_y + grid_y * self.resolution
        return (world_x, world_y)
    
    def add_lidar_scan(self, drone_pos: Tuple[float, float], lidar_distances, drone_orientation: float = 0.0):
        """
        Process LIDAR scan to discover walls and open areas.

        Args:
            drone_pos: Current drone position (x, y)
            lidar_distances: dict (new STM format) or list (legacy)
            drone_orientation: Drone's current orientation in radians
        """
        drone_x, drone_y = drone_pos

        # Mark drone's current cell as explored
        drone_grid = self.world_to_grid(drone_pos)
        self.explored_cells.add(drone_grid)

        # Unpack new dict format or fall back to legacy list
        if isinstance(lidar_distances, dict):
            ranges = lidar_distances["ranges"]
            start_angle = lidar_distances["start_angle"]
            angle_step = lidar_distances["angle_step"]
            max_range = 9.0
        else:
            ranges = lidar_distances
            lidar_resolution = len(ranges)
            start_angle = drone_orientation
            angle_step = 2 * math.pi / lidar_resolution if lidar_resolution > 0 else 0
            max_range = 15.0

        # Process each LIDAR hit
        for i, distance in enumerate(ranges):
            if distance > 0 and distance < max_range:  # Valid hit within range
                # Calculate ray angle
                ray_angle = start_angle + (i * angle_step)

                # Calculate hit position
                hit_x = drone_x + distance * math.cos(ray_angle)
                hit_y = drone_y + distance * math.sin(ray_angle)

                # This represents a wall/obstacle
                wall_feature = DiscoveredFeature(
                    feature_type=FeatureType.WALL,
                    position=(hit_x, hit_y),
                    confidence=0.8,  # High confidence for LIDAR hits
                    timestamp=np.random.random(),  # Would be real timestamp
                    additional_data={"method": "lidar", "distance": distance}
                )

                self._add_feature(wall_feature)

                # Mark cells along the ray as explored (free space)
                self._mark_ray_as_explored(drone_pos, (hit_x, hit_y))
    
    def add_vision_detection(self, position: Tuple[float, float], detection_type: str, confidence: float):
        """
        Add a vision-based detection (door, object, etc).
        
        Args:
            position: World position of detection
            detection_type: Type of object detected
            confidence: Detection confidence 0.0-1.0
        """
        # Map detection types to our feature types
        feature_type_map = {
            "door": FeatureType.DOOR,
            "object": FeatureType.OBJECT,
            "person": FeatureType.OBJECT,
            "furniture": FeatureType.OBJECT,
            "ied": FeatureType.IED,
            "explosive": FeatureType.IED
        }
        
        feature_type = feature_type_map.get(detection_type.lower(), FeatureType.UNKNOWN_OBSTACLE)
        
        feature = DiscoveredFeature(
            feature_type=feature_type,
            position=position,
            confidence=confidence,
            timestamp=np.random.random(),  # Would be real timestamp
            additional_data={"method": "vision", "detection_type": detection_type}
        )
        
        self._add_feature(feature)
    
    def add_ied_detection(self, position: Tuple[float, float], confidence: float):
        """Add IED detection from IED sensor."""
        if confidence >= self.ied_confidence_threshold:
            ied_feature = DiscoveredFeature(
                feature_type=FeatureType.IED,
                position=position,
                confidence=confidence,
                timestamp=np.random.random(),
                additional_data={"method": "ied_sensor"}
            )
            
            self._add_feature(ied_feature)
    
    def _add_feature(self, feature: DiscoveredFeature):
        """Add a discovered feature to the minimap."""
        # Add to main list
        self.discovered_features.append(feature)
        
        # Add to spatial grid for quick lookup
        grid_pos = self.world_to_grid(feature.position)
        if grid_pos not in self.feature_grid:
            self.feature_grid[grid_pos] = []
        self.feature_grid[grid_pos].append(feature)
        
        # Add to type-specific lists if confidence is high enough
        if feature.feature_type == FeatureType.WALL and feature.confidence >= self.wall_confidence_threshold:
            # For walls, we'll build wall segments later
            pass
        elif feature.feature_type == FeatureType.DOOR and feature.confidence >= self.door_confidence_threshold:
            if feature.position not in self.discovered_doors:
                self.discovered_doors.append(feature.position)
        elif feature.feature_type == FeatureType.OBJECT and feature.confidence >= self.object_confidence_threshold:
            self.discovered_objects.append(feature)
        elif feature.feature_type == FeatureType.IED and feature.confidence >= self.ied_confidence_threshold:
            self.discovered_ieds.append(feature)
    
    def _mark_ray_as_explored(self, start: Tuple[float, float], end: Tuple[float, float]):
        """Mark cells along a ray as explored (free space)."""
        start_grid = self.world_to_grid(start)
        end_grid = self.world_to_grid(end)
        
        # Simple line drawing algorithm to mark cells
        x0, y0 = start_grid
        x1, y1 = end_grid
        
        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            # Mark cell as explored
            if 0 <= x < self.width_cells and 0 <= y < self.height_cells:
                self.explored_cells.add((x, y))
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def build_wall_segments(self) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """
        Build wall segments from individual wall points.
        Creates continuous lines by connecting nearby points.
        """
        if not self.discovered_features:
            return []
        
        # Get all wall features with high confidence
        wall_points = []
        for feature in self.discovered_features:
            if (feature.feature_type == FeatureType.WALL and 
                feature.confidence >= self.wall_confidence_threshold):
                wall_points.append(feature.position)
        
        if len(wall_points) < 2:
            return []
        
        # Group points by proximity to form continuous wall segments
        segments = []
        connect_distance = 1.0  # Connect points within 1m to form continuous walls
        
        # Sort points to help with line formation
        wall_points.sort(key=lambda p: (p[0], p[1]))
        
        # Connect consecutive nearby points
        for i in range(len(wall_points) - 1):
            point1 = wall_points[i]
            point2 = wall_points[i + 1]
            
            dist = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
            if dist <= connect_distance:
                segments.append((point1, point2))
        
        self.discovered_walls = segments
        return segments
    
    def get_minimap_data(self) -> Dict:
        """
        Get all minimap data for rendering.
        
        Returns:
            Dictionary containing all discovered features and exploration data
        """
        # Build wall segments from discovered points
        wall_segments = self.build_wall_segments()
        
        # Include detection_type labels for objects
        objects_with_labels = []
        for f in self.discovered_objects:
            label = f.additional_data.get('detection_type', '') if f.additional_data else ''
            objects_with_labels.append((f.position, f.confidence, label))
        
        ieds_with_labels = []
        for f in self.discovered_ieds:
            ieds_with_labels.append((f.position, f.confidence, 'IED'))
        
        return {
            "world_bounds": self.world_bounds,
            "resolution": self.resolution,
            "explored_cells": list(self.explored_cells),
            "wall_segments": wall_segments,
            "doors": self.discovered_doors,
            "objects": objects_with_labels,  # Now includes labels
            "ieds": ieds_with_labels,  # Now includes labels
            "grid_size": (self.width_cells, self.height_cells),
            "total_features": len(self.discovered_features)
        }
    
    def get_exploration_coverage(self) -> float:
        """Calculate percentage of world that has been explored."""
        total_cells = self.width_cells * self.height_cells
        explored_count = len(self.explored_cells)
        return (explored_count / total_cells) * 100.0 if total_cells > 0 else 0.0
    
    def reset(self):
        """Reset the minimap to empty state."""
        self.discovered_features.clear()
        self.feature_grid.clear()
        self.discovered_walls.clear()
        self.discovered_doors.clear()
        self.discovered_objects.clear()
        self.discovered_ieds.clear()
        self.explored_cells.clear()
