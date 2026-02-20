"""
SLAM (Simultaneous Localization and Mapping) System
Implements grid-based mapping with particle filter localization
Optimized for embedded systems with limited memory
"""

import numpy as np
import math
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from core.navigation import PathPlanner

@dataclass
class Particle:
    """Particle for particle filter localization."""
    x: float
    y: float
    theta: float  # orientation
    weight: float = 1.0

class GridMap:
    """
    Occupancy grid map for environment representation.
    Memory efficient implementation for embedded systems.
    """
    
    def __init__(self, width: float, height: float, resolution: float = 0.2):
        """Initialize grid map with given dimensions and resolution."""
        self.width = width
        self.height = height
        self.resolution = resolution
        
        # Calculate grid dimensions
        self.grid_width = int(math.ceil(width / resolution))
        self.grid_height = int(math.ceil(height / resolution))
        
        # Grid values: 0=free, 50=unknown, 100=occupied
        self.grid = np.full((self.grid_height, self.grid_width), 50, dtype=np.uint8)
        
        # Track exploration coverage
        self.explored_cells = set()
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        
        # Clamp to grid bounds
        grid_x = max(0, min(self.grid_width - 1, grid_x))
        grid_y = max(0, min(self.grid_height - 1, grid_y))
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        x = (grid_x + 0.5) * self.resolution
        y = (grid_y + 0.5) * self.resolution
        return x, y
    
    def update_with_lidar(self, robot_pos: Tuple[float, float], 
                         robot_theta: float, 
                         lidar_ranges: List[float],
                         max_range: float = 25.0):
        """Update map using LIDAR scan data."""
        robot_grid_x, robot_grid_y = self.world_to_grid(robot_pos[0], robot_pos[1])
        
        num_rays = len(lidar_ranges)
        angle_step = 2 * math.pi / num_rays
        
        for i, range_reading in enumerate(lidar_ranges):
            if range_reading >= max_range:
                continue  # Ignore max range readings
                
            # Calculate ray angle
            ray_angle = robot_theta + (i * angle_step)
            
            # Calculate end point of ray in world
            end_x = robot_pos[0] + range_reading * math.cos(ray_angle)
            end_y = robot_pos[1] + range_reading * math.sin(ray_angle)
            end_grid_x, end_grid_y = self.world_to_grid(end_x, end_y)
            
            # Trace ray using Bresenham's algorithm
            cells = self._bresenham_line(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y)
            
            # Mark cells along ray as free (except the last one)
            for j, (cell_x, cell_y) in enumerate(cells[:-1]):
                if 0 <= cell_x < self.grid_width and 0 <= cell_y < self.grid_height:
                    # Decrease occupancy probability (mark as more likely free)
                    current_val = int(self.grid[cell_y, cell_x])
                    self.grid[cell_y, cell_x] = max(0, current_val - 5)
                    self.explored_cells.add((cell_x, cell_y))
            
            # Mark end cell as occupied (obstacle)
            if 0 <= end_grid_x < self.grid_width and 0 <= end_grid_y < self.grid_height:
                current_val = int(self.grid[end_grid_y, end_grid_x])
                self.grid[end_grid_y, end_grid_x] = min(100, current_val + 20)
                self.explored_cells.add((end_grid_x, end_grid_y))
    
    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham's line algorithm for ray tracing."""
        points = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def get_occupancy(self, x: float, y: float) -> int:
        """Get occupancy value at world coordinates."""
        grid_x, grid_y = self.world_to_grid(x, y)
        return self.grid[grid_y, grid_x]
    
    def is_free(self, x: float, y: float, threshold: int = 30) -> bool:
        """Check if a world position is free space."""
        return self.get_occupancy(x, y) < threshold
    
    def get_exploration_progress(self) -> float:
        """Get percentage of map explored."""
        # COMMENTED OUT - This calculation is totally arbitrary and wrong
        # We have NO idea what the actual coverage is without knowing building layout
        # total_cells = self.grid_width * self.grid_height
        # explored = len(self.explored_cells)
        # 
        # # Don't return 100% unless we've explored a reasonable amount
        # if explored < 100:  # Haven't even explored 100 cells yet
        #     return 0.0
        #     
        # coverage = (explored / total_cells) * 100.0
        # return min(95.0, coverage)  # Cap at 95% to avoid premature completion
        
        # Return 0 - we don't actually know the coverage
        return 0.0

class ParticleFilter:
    """
    Particle filter for robot localization.
    Tracks robot position uncertainty using weighted particles.
    """
    
    def __init__(self, num_particles: int = 100):
        """Initialize particle filter."""
        self.num_particles = num_particles
        self.particles = []
        self.initialized = False
    
    def initialize(self, initial_pos: Tuple[float, float], initial_theta: float):
        """Initialize particles around initial position."""
        self.particles = []
        
        for _ in range(self.num_particles):
            # Add noise to initial position
            x = initial_pos[0] + np.random.normal(0, 0.5)
            y = initial_pos[1] + np.random.normal(0, 0.5)
            theta = initial_theta + np.random.normal(0, 0.1)
            
            particle = Particle(x, y, theta, 1.0 / self.num_particles)
            self.particles.append(particle)
        
        self.initialized = True
    
    def predict(self, motion: Tuple[float, float, float], motion_noise: Tuple[float, float, float]):
        """Predict particle positions based on motion model."""
        if not self.initialized:
            return
        
        dx, dy, dtheta = motion
        noise_x, noise_y, noise_theta = motion_noise
        
        for particle in self.particles:
            # Apply motion with noise
            particle.x += dx + np.random.normal(0, noise_x)
            particle.y += dy + np.random.normal(0, noise_y)
            particle.theta += dtheta + np.random.normal(0, noise_theta)
            
            # Normalize angle
            particle.theta = self._normalize_angle(particle.theta)
    
    def update(self, lidar_ranges: List[float], grid_map: GridMap):
        """Update particle weights based on sensor observations."""
        if not self.initialized:
            return
        
        for particle in self.particles:
            # Calculate likelihood of observation given particle pose
            likelihood = self._calculate_likelihood(particle, lidar_ranges, grid_map)
            particle.weight *= likelihood
        
        # Normalize weights
        total_weight = sum(p.weight for p in self.particles)
        if total_weight > 1e-10:  # Avoid division by zero
            for particle in self.particles:
                particle.weight /= total_weight
        else:
            # Reset weights if all are zero/very small
            for particle in self.particles:
                particle.weight = 1.0 / self.num_particles
        
        # Resample if effective particle count is low
        if self._effective_particle_count() < self.num_particles / 2:
            self._resample()
    
    def get_pose_estimate(self) -> Tuple[float, float, float]:
        """Get current pose estimate (weighted average of particles)."""
        if not self.initialized or not self.particles:
            return (0.0, 0.0, 0.0)
        
        total_weight = sum(p.weight for p in self.particles)
        if total_weight == 0:
            return (self.particles[0].x, self.particles[0].y, self.particles[0].theta)
        
        x = sum(p.x * p.weight for p in self.particles) / total_weight
        y = sum(p.y * p.weight for p in self.particles) / total_weight
        
        # Handle circular average for angle
        sin_sum = sum(math.sin(p.theta) * p.weight for p in self.particles) / total_weight
        cos_sum = sum(math.cos(p.theta) * p.weight for p in self.particles) / total_weight
        theta = math.atan2(sin_sum, cos_sum)
        
        return (x, y, theta)
    
    def _calculate_likelihood(self, particle: Particle, lidar_ranges: List[float], 
                            grid_map: GridMap) -> float:
        """Calculate observation likelihood for a particle."""
        likelihood = 1.0
        
        num_rays = len(lidar_ranges)
        angle_step = 2 * math.pi / num_rays
        
        # Sample every 10th ray for efficiency
        for i in range(0, num_rays, 10):
            expected_range = self._expected_range(particle, i * angle_step, grid_map)
            observed_range = lidar_ranges[i]
            
            # Gaussian likelihood model
            diff = abs(expected_range - observed_range)
            likelihood *= math.exp(-diff * diff / (2 * 0.5 * 0.5))
        
        return likelihood
    
    def _expected_range(self, particle: Particle, ray_angle: float, 
                       grid_map: GridMap, max_range: float = 10.0) -> float:
        """Calculate expected range for a ray from particle position."""
        ray_dir = particle.theta + ray_angle
        
        # Ray casting in grid
        step_size = grid_map.resolution / 2
        current_range = 0.0
        
        while current_range < max_range:
            x = particle.x + current_range * math.cos(ray_dir)
            y = particle.y + current_range * math.sin(ray_dir)
            
            # Check if we hit an obstacle
            if grid_map.get_occupancy(x, y) > 70:  # Occupied threshold
                return current_range
            
            current_range += step_size
        
        return max_range
    
    def _effective_particle_count(self) -> float:
        """Calculate effective number of particles."""
        weight_sum_sq = sum(p.weight * p.weight for p in self.particles)
        if weight_sum_sq == 0:
            return 0
        return 1.0 / weight_sum_sq
    
    def _resample(self):
        """Resample particles based on weights."""
        if not self.particles:
            return
        
        new_particles = []
        weights = [p.weight for p in self.particles]
        
        # Ensure weights sum to 1.0
        weight_sum = sum(weights)
        if weight_sum > 1e-10:
            weights = [w / weight_sum for w in weights]
        else:
            weights = [1.0 / len(self.particles)] * len(self.particles)
        
        # Systematic resampling
        try:
            indices = np.random.choice(len(self.particles), self.num_particles, p=weights)
            
            for idx in indices:
                old_particle = self.particles[idx]
                new_particle = Particle(old_particle.x, old_particle.y, old_particle.theta, 
                                       1.0 / self.num_particles)
                new_particles.append(new_particle)
            
            self.particles = new_particles
        except ValueError:
            # If resampling fails, just reset all particles with equal weights
            for particle in self.particles:
                particle.weight = 1.0 / self.num_particles
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

class SLAMSystem:
    """
    Complete SLAM system combining mapping and localization.
    Coordinates between grid mapping and particle filter localization.
    """
    
    def __init__(self, map_width: float = 50, map_height: float = 50):
        """Initialize SLAM system."""
        self.grid_map = GridMap(map_width, map_height)
        self.particle_filter = ParticleFilter()
        self.path_planner = PathPlanner()
        
        # State tracking
        self.last_position = None
        self.last_orientation = None
        self.initialized = False
        
        # Exploration parameters
        self.exploration_points = []
        self.current_exploration_index = 0
        self.exploration_radius = 3.0  # Minimum distance to consider a point explored (favor progress)
        self.entry_point: Tuple[float, float] = (0.0, 0.0)
        self.room_exit_events: int = 0
    
    def initialize(self, initial_pos: Tuple[float, float], initial_theta: float = 0.0):
        """Initialize SLAM system with starting position."""
        self.particle_filter.initialize(initial_pos, initial_theta)
        self.last_position = initial_pos
        self.last_orientation = initial_theta
        self.initialized = True
        self.entry_point = (initial_pos[0], initial_pos[1])
        
        # Plan initial exploration (defer until we have environment reference)
        self.exploration_points = []
    
    def update(self, current_pos: Tuple[float, float], lidar_ranges: List[float], 
              current_theta: Optional[float] = None):
        """Update SLAM system with new sensor data."""
        if not self.initialized:
            self.initialize(current_pos)
            return
        
        # Calculate motion since last update
        if self.last_position is not None:
            dx = current_pos[0] - self.last_position[0]
            dy = current_pos[1] - self.last_position[1]
            dtheta = 0.0
            
            if current_theta is not None and self.last_orientation is not None:
                dtheta = current_theta - self.last_orientation
                # Normalize angle difference
                while dtheta > math.pi:
                    dtheta -= 2 * math.pi
                while dtheta < -math.pi:
                    dtheta += 2 * math.pi
            
            # Motion noise model
            motion_noise = (0.1, 0.1, 0.05)
            
            # Predict particle positions
            self.particle_filter.predict((dx, dy, dtheta), motion_noise)
        
        # Get current pose estimate
        estimated_pos = self.particle_filter.get_pose_estimate()
        
        # Update map with LIDAR data
        self.grid_map.update_with_lidar(
            (estimated_pos[0], estimated_pos[1]), 
            estimated_pos[2], 
            lidar_ranges
        )
        
        # Update particle weights with observation
        self.particle_filter.update(lidar_ranges, self.grid_map)
        
        # Store current state
        self.last_position = current_pos
        if current_theta is not None:
            self.last_orientation = current_theta
    
    def get_map(self) -> GridMap:
        """Get current map."""
        return self.grid_map
    
    def get_pose_estimate(self) -> Tuple[float, float, float]:
        """Get current pose estimate."""
        return self.particle_filter.get_pose_estimate()
    
    def plan_exploration(self, environment, breadcrumbs: List[Tuple[float, float]] = None):
        """Plan exploration points using frontier-based strategy."""
        if self.initialized:
            current_pos = self.particle_filter.get_pose_estimate()[:2]
            
            # Use simple lawn mower pattern for exploration
            self.exploration_points = self.path_planner.plan_exploration_mission(
                current_pos, environment
            )
    
    def get_next_exploration_point(self) -> Optional[Tuple[float, float]]:
        """Get next point for exploration."""
        if not self.exploration_points:
            return None
        
        current_pos = self.particle_filter.get_pose_estimate()[:2]
        
        # Check if current exploration point is reached
        if self.current_exploration_index < len(self.exploration_points):
            target = self.exploration_points[self.current_exploration_index]
            distance = math.sqrt((current_pos[0] - target[0])**2 + 
                               (current_pos[1] - target[1])**2)
            
            if distance < self.exploration_radius:
                self.current_exploration_index += 1
        
        # Return next exploration point
        if self.current_exploration_index < len(self.exploration_points):
            return self.exploration_points[self.current_exploration_index]
        
        return None  # Exploration complete
    
    def get_exploration_progress(self) -> Dict:
        """Get exploration progress information."""
        return {
            'map_coverage': self.grid_map.get_exploration_progress(),
            'points_visited': self.current_exploration_index,
            'total_points': len(self.exploration_points),
            'current_target': self.get_next_exploration_point(),
            'room_exits': self.room_exit_events
        }
    
    def reset(self):
        """Reset SLAM system."""
        self.grid_map = GridMap(self.grid_map.width, self.grid_map.height)
        self.particle_filter = ParticleFilter()
        self.last_position = None
        self.last_orientation = None
        self.initialized = False
        self.exploration_points = []
        self.current_exploration_index = 0
        self.room_exit_events = 0
