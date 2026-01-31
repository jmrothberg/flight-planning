"""
Environment representation and sensor simulation
Models building interiors with obstacles, walls, and objects of interest
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
import math
import random
import time

class Environment:
    """
    Represents the building environment that the drone navigates.
    Includes walls, obstacles, and objects of interest.
    """
    
    def __init__(self, width: float = 50, height: float = 50):
        """Initialize environment with given dimensions (in meters)."""
        self.width = width
        self.height = height
        
        # Environment elements
        self.walls = []  # List of wall segments [(x1,y1), (x2,y2)]
        self.obstacles = []  # List of circular obstacles [(x,y,radius)]
        self.objects_of_interest = []  # People, important items [(x,y,type,confidence)]
        
        # Sensor parameters
        self.lidar_range = 25.0  # meters  # increased to detect openings further away
        self.lidar_resolution = 360  # number of rays
        self.camera_fov = 60.0  # degrees
        self.camera_range = 8.0  # meters
        
        # Building layout management - ALL REALISTIC ROOM LAYOUTS WITH DOORS
        # Each layout has proper walls, rooms, and doorways connecting all spaces
        self.current_layout = 0
        self.layout_generators = [
            self._generate_layout_complex_rooms,     # 1. Original: Complex rooms with corridors
            self._generate_layout_connected_rooms_1, # 2. Mixed room sizes with central corridor
            self._generate_layout_connected_rooms_2, # 3. L-shaped building layout
            self._generate_layout_connected_rooms_3, # 4. Central hallway with side rooms
            self._generate_layout_connected_rooms_4  # 5. 3x3 grid of interconnected rooms
        ]
        
        # Generate initial building layout
        self.generate_new_building()
    
    def generate_new_building(self):
        """Generate a new building layout cycling through different designs."""
        # Clear existing environment
        self.walls = []
        self.obstacles = []
        self.objects_of_interest = []
        
        # Generate new layout
        self.layout_generators[self.current_layout]()
        
        # Always randomize objects after generating layout
        self.randomize_objects()
        
        # Move to next layout for next time
        self.current_layout = (self.current_layout + 1) % len(self.layout_generators)
        
        print(f"Generated building layout #{self.current_layout}/{len(self.layout_generators)}")
    
    def _generate_layout_complex_rooms(self):
        """Generate original complex room layout with corridors and multiple rooms."""
        # Outer walls
        self.walls = [
            # Bottom wall with FRONT DOOR gap from x=20 to x=30
            [(0, 0), (20, 0)],
            [(30, 0), (self.width, 0)],
            [(self.width, 0), (self.width, self.height)],  # Right wall
            [(self.width, self.height), (0, self.height)],  # Top wall
            [(0, self.height), (0, 0)]  # Left wall
        ]
        
        # Inner walls to create rooms
        # Horizontal walls
        self.walls.extend([
            [(0, 15), (20, 15)],  # Room divider
            [(25, 15), (50, 15)],  # Room divider with gap
            [(0, 35), (25, 35)],  # Upper room divider
            [(30, 35), (50, 35)]   # Upper room divider with gap
        ])
        
        # Vertical walls
        self.walls.extend([
            [(20, 0), (20, 10)],   # Room divider with gap
            [(20, 20), (20, 35)],  # Vertical corridor wall
            [(35, 15), (35, 30)]   # Small room divider
        ])
        
        # Add some obstacles (furniture, etc.)
        self.obstacles = [
            (8, 8, 1.5),   # Table in room 1
            (12, 25, 1.0), # Chair in corridor
            (30, 8, 2.0),  # Large furniture
            (45, 25, 1.2), # Equipment
            (15, 42, 1.8), # Furniture in upper room
            (40, 45, 1.0)  # Small obstacle
        ]
    
    def _generate_layout_connected_rooms_1(self):
        """Generate a building with different sized rooms, all connected by doors."""
        # Outer walls with front door
        self.walls = [
            [(0, 0), (20, 0)],      # Bottom wall left of door
            [(30, 0), (self.width, 0)],  # Bottom wall right of door
            [(self.width, 0), (self.width, self.height)],  # Right wall
            [(self.width, self.height), (0, self.height)],  # Top wall
            [(0, self.height), (0, 0)]  # Left wall
        ]
        
        # Create connected rooms with doorways
        # Main horizontal corridor at y=25
        self.walls.extend([
            [(0, 25), (15, 25)],    # Left side of corridor with door at 15-20
            [(22, 25), (35, 25)],   # Middle section with doors at 20-22 and 35-38
            [(40, 25), (50, 25)]    # Right side with door at 38-40
        ])
        
        # Vertical walls creating rooms
        self.walls.extend([
            # Left section rooms
            [(15, 0), (15, 22)],    # Vertical wall with door to corridor
            [(15, 28), (15, 50)],   # Continues above corridor
            
            # Middle section
            [(30, 0), (30, 10)],    # Lower room divider with door gap
            [(30, 13), (30, 22)],   # Continues with door at 10-13
            [(30, 28), (30, 50)],   # Upper room divider
            
            # Create some interior rooms
            [(0, 12), (12, 12)],    # Horizontal divider in bottom-left
            [(35, 35), (50, 35)]    # Horizontal divider in top-right
        ])
        
        # Add realistic furniture
        self.obstacles = [
            (7, 6, 1.5),    # Table in bottom-left room
            (7, 18, 1.2),   # Chair
            (22, 6, 1.8),   # Desk in bottom-middle room
            (40, 10, 1.5),  # Table in bottom-right
            (10, 32, 1.5),  # Furniture in top-left
            (25, 40, 1.2),  # Top-middle room
            (42, 42, 1.5)   # Top-right room
        ]
    
    def _generate_layout_connected_rooms_2(self):
        """Generate an L-shaped building with connected rooms."""
        # Outer walls with front door
        self.walls = [
            [(0, 0), (20, 0)],      # Bottom wall left of door
            [(30, 0), (self.width, 0)],  # Bottom wall right of door
            [(self.width, 0), (self.width, 30)],  # Right wall (shorter for L-shape)
            [(self.width, 30), (25, 30)],  # Inner corner of L
            [(25, 30), (25, self.height)],  # Vertical part of L
            [(25, self.height), (0, self.height)],  # Top wall
            [(0, self.height), (0, 0)]  # Left wall
        ]
        
        # Interior walls with doorways
        self.walls.extend([
            # Vertical corridor/hallway
            [(12, 0), (12, 8)],     # Bottom room divider with door at 8-12
            [(12, 15), (12, 35)],   # Middle section with doors at 12-15 and 35-40
            [(12, 42), (12, 50)],   # Top section with door at 40-42
            
            # Horizontal walls
            [(0, 20), (8, 20)],     # Left room divider with door
            [(15, 20), (25, 20)],   # Right section
            
            # Right wing rooms
            [(35, 0), (35, 12)],    # Vertical divider with door
            [(35, 18), (35, 30)],   # Continues with door gap
            [(25, 15), (32, 15)],   # Horizontal connector with door
            [(38, 15), (50, 15)]    # Right side
        ])
        
        # Add realistic furniture
        self.obstacles = [
            (6, 10, 1.5),   # Left rooms
            (6, 30, 1.2),
            (18, 10, 1.5),  # Middle section
            (18, 35, 1.8),
            (30, 7, 1.2),   # Right wing
            (42, 7, 1.5),
            (42, 22, 1.2)
        ]
    
    def _generate_layout_connected_rooms_3(self):
        """Generate a building with a central hallway and rooms on both sides."""
        # Outer walls with front door
        self.walls = [
            [(0, 0), (20, 0)],      # Bottom wall left of door
            [(30, 0), (self.width, 0)],  # Bottom wall right of door
            [(self.width, 0), (self.width, self.height)],  # Right wall
            [(self.width, self.height), (0, self.height)],  # Top wall
            [(0, self.height), (0, 0)]  # Left wall
        ]
        
        # Central hallway running north-south from entrance
        self.walls.extend([
            [(18, 0), (18, 8)],     # Left hallway wall with entrance gap
            [(18, 12), (18, 20)],   # Continue with door gaps
            [(18, 24), (18, 38)],
            [(18, 42), (18, 50)],
            
            [(32, 0), (32, 10)],    # Right hallway wall with entrance gap
            [(32, 14), (32, 22)],   # Continue with door gaps
            [(32, 26), (32, 36)],
            [(32, 40), (32, 50)]
        ])
        
        # Left side rooms
        self.walls.extend([
            [(0, 15), (15, 15)],    # Room divider with door to hallway
            [(0, 30), (14, 30)],    # Another room divider
        ])
        
        # Right side rooms
        self.walls.extend([
            [(35, 12), (50, 12)],   # Room divider with door to hallway
            [(36, 25), (50, 25)],   # Another divider
            [(38, 38), (50, 38)]    # Top room divider
        ])
        
        # Add realistic furniture
        self.obstacles = [
            (8, 7, 1.5),    # Left side rooms
            (8, 22, 1.2),
            (8, 40, 1.8),
            (25, 25, 1.0),  # Hallway furniture
            (42, 6, 1.5),   # Right side rooms
            (42, 18, 1.2),
            (42, 31, 1.5),
            (42, 44, 1.2)
        ]
    
    def _generate_layout_connected_rooms_4(self):
        """Generate a grid of connected rooms with multiple doorways."""
        # Outer walls with front door
        self.walls = [
            [(0, 0), (20, 0)],      # Bottom wall left of door
            [(30, 0), (self.width, 0)],  # Bottom wall right of door
            [(self.width, 0), (self.width, self.height)],  # Right wall
            [(self.width, self.height), (0, self.height)],  # Top wall
            [(0, self.height), (0, 0)]  # Left wall
        ]
        
        # Create a 3x3 grid of rooms with connecting doors
        # Horizontal walls (with door gaps)
        self.walls.extend([
            # First horizontal divider (y=17)
            [(0, 17), (10, 17)],     # Door at 10-14
            [(14, 17), (22, 17)],    # Door at 22-28
            [(28, 17), (36, 17)],    # Door at 36-40
            [(40, 17), (50, 17)],
            
            # Second horizontal divider (y=33)
            [(0, 33), (8, 33)],      # Door at 8-12
            [(12, 33), (25, 33)],    # Door at 25-30
            [(30, 33), (38, 33)],    # Door at 38-42
            [(42, 33), (50, 33)]
        ])
        
        # Vertical walls (with door gaps)
        self.walls.extend([
            # First vertical divider (x=17)
            [(17, 0), (17, 7)],      # Door at 7-10
            [(17, 10), (17, 14)],    # Door at 14-20
            [(17, 20), (17, 30)],    # Door at 30-36
            [(17, 36), (17, 44)],    # Door at 44-50
            [(17, 47), (17, 50)],
            
            # Second vertical divider (x=33)
            [(33, 0), (33, 6)],      # Door at 6-11
            [(33, 11), (33, 15)],    # Door at 15-19
            [(33, 19), (33, 28)],    # Door at 28-35
            [(33, 35), (33, 43)],    # Door at 43-47
            [(33, 47), (33, 50)]
        ])
        
        # Add furniture in each room section
        self.obstacles = [
            (8, 8, 1.2),     # Bottom-left section
            (25, 8, 1.5),    # Bottom-middle
            (42, 8, 1.2),    # Bottom-right
            (8, 25, 1.5),    # Middle-left
            (25, 25, 1.0),   # Center
            (42, 25, 1.5),   # Middle-right
            (8, 42, 1.2),    # Top-left
            (25, 42, 1.5),   # Top-middle
            (42, 42, 1.2)    # Top-right
        ]
    
    def get_lidar_scan(self, position: np.ndarray, orientation: float) -> List[float]:
        """
        Simulate LIDAR scan from given position and orientation.
        Returns list of distances for each ray.
        """
        distances = []
        angle_step = 2 * math.pi / self.lidar_resolution
        
        for i in range(self.lidar_resolution):
            ray_angle = orientation + (i * angle_step)
            distance = self._cast_ray(position[:2], ray_angle)
            distances.append(min(distance, self.lidar_range))
        
        return distances
    
    def get_camera_view(self, position: np.ndarray, orientation: float) -> Dict:
        """
        Simulate camera view from given position and orientation.
        Returns dictionary with visible objects and image data.
        """
        visible_objects = []
        
        # Calculate camera viewing cone
        half_fov = math.radians(self.camera_fov / 2)
        start_angle = orientation - half_fov
        end_angle = orientation + half_fov
        
        # Check which objects are in view
        for obj_x, obj_y, obj_type, confidence in self.objects_of_interest:
            # Calculate angle and distance to object
            dx = obj_x - position[0]
            dy = obj_y - position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > self.camera_range:
                continue
                
            angle_to_obj = math.atan2(dy, dx)
            
            # Check if object is within camera FOV
            if self._angle_in_range(angle_to_obj, start_angle, end_angle):
                # Check if object is not occluded
                if not self._is_occluded(position[:2], (obj_x, obj_y)):
                    visible_objects.append({
                        'type': obj_type,
                        'position': (obj_x, obj_y),
                        'distance': distance,
                        'confidence': confidence * (1.0 - distance / self.camera_range)  # Confidence decreases with distance
                    })
        
        return {
            'objects': visible_objects,
            'timestamp': self._get_timestamp()
        }
    
    def _cast_ray(self, start_pos: Tuple[float, float], angle: float) -> float:
        """Cast a ray and return distance to nearest obstacle."""
        ray_dir = (math.cos(angle), math.sin(angle))
        min_distance = float('inf')
        
        # Check walls
        for wall in self.walls:
            distance = self._ray_line_intersection(start_pos, ray_dir, wall)
            if distance is not None and distance >= 0 and distance < min_distance:
                min_distance = distance
        
        # Check circular obstacles
        for obs_x, obs_y, radius in self.obstacles:
            distance = self._ray_circle_intersection(start_pos, ray_dir, (obs_x, obs_y), radius)
            if distance is not None and distance >= 0 and distance < min_distance:
                min_distance = distance
        
        return min_distance
    
    def check_collision(self, position: Tuple[float, float], radius: float = 0.5) -> bool:
        """
        Check if position with given radius would collide with any obstacle.
        Returns True if collision detected.
        """
        x, y = position
        
        # Check walls (simplified - check distance to line segments)
        for wall in self.walls:
            dist = self._point_to_line_distance((x, y), wall)
            if dist < radius:
                return True
        
        # Check circular obstacles
        for obs_x, obs_y, obs_radius in self.obstacles:
            dist = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if dist < radius + obs_radius:
                return True
        
        return False
    
    def is_position_valid(self, position: Tuple[float, float], radius: float = 0.5) -> bool:
        """
        Check if position is valid (no collision).
        Returns True if position is valid (no obstacles).
        This is the inverse of check_collision for compatibility.
        """
        return not self.check_collision(position, radius)
    
    def is_path_clear(self, start: Tuple[float, float], end: Tuple[float, float], radius: float = 0.3) -> bool:
        """
        Check if path from start to end crosses any walls.
        Returns True if path is clear (no wall intersections).
        
        Args:
            start: Starting position (x, y)
            end: Ending position (x, y)
            radius: Clearance radius for wall proximity
        """
        # Direction vector
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.01:
            return True  # Start and end are same point
        
        # Check if path crosses any wall
        for wall in self.walls:
            # Check intersection with wall segment
            intersection = self._ray_line_intersection(start, (dx/dist, dy/dist), wall)
            if intersection is not None and intersection < dist:
                return False  # Wall blocks path
            
            # Also check if path passes too close to wall (within radius)
            # Sample points along path
            for t in [0.25, 0.5, 0.75]:
                check_x = start[0] + dx * t
                check_y = start[1] + dy * t
                wall_dist = self._point_to_line_distance((check_x, check_y), wall)
                if wall_dist < radius:
                    return False  # Path passes too close to wall
        
        # Check obstacles too
        for obs_x, obs_y, obs_radius in self.obstacles:
            for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
                check_x = start[0] + dx * t
                check_y = start[1] + dy * t
                obs_dist = math.sqrt((check_x - obs_x)**2 + (check_y - obs_y)**2)
                if obs_dist < radius + obs_radius:
                    return False  # Path crosses obstacle
        
        return True
    
    def _point_to_line_distance(self, point: Tuple[float, float],
                               line: List[Tuple[float, float]]) -> float:
        """Calculate minimum distance from point to line segment."""
        x0, y0 = point
        x1, y1 = line[0]
        x2, y2 = line[1]
        
        # Vector from line start to point
        dx = x2 - x1
        dy = y2 - y1
        
        # If line is a point
        if dx == 0 and dy == 0:
            return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)
        
        # Parameter t for closest point on line
        t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx*dx + dy*dy)
        
        # Clamp t to [0, 1] for line segment
        t = max(0, min(1, t))
        
        # Closest point on segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        # Distance to closest point
        return math.sqrt((x0 - closest_x)**2 + (y0 - closest_y)**2)
    
    def _ray_line_intersection(self, ray_start: Tuple[float, float],
                              ray_dir: Tuple[float, float],
                              line: List[Tuple[float, float]]) -> Optional[float]:
        """
        Calculate intersection between ray and line segment.
        - Ray: P = ray_start + t * ray_dir (t >= 0 for forward)
        - Line: Q = line[0] + u * (line[1] - line[0]) (0 <= u <= 1 for segment)
        
        Returns distance t where ray hits wall, or None if no intersection.
        """
        # Ray: P = ray_start + t * ray_dir
        # Line segment: Q = line[0] + u * (line[1] - line[0])
        x0, y0 = ray_start
        dx, dy = ray_dir
        x1, y1 = line[0]
        x2, y2 = line[1]
        
        # Line segment vector
        sx = x2 - x1
        sy = y2 - y1
        
        # Cross product to check parallel
        cross = dx * sy - dy * sx
        if abs(cross) < 1e-10:
            return None
        
        # Solve for t and u
        t = ((x1 - x0) * sy - (y1 - y0) * sx) / cross
        u = ((x1 - x0) * dy - (y1 - y0) * dx) / cross
        
        # Check bounds: t >= 0 (ray forward), 0 <= u <= 1 (on segment)
        if t >= 0 and 0 <= u <= 1:
            return t
        return None
    
    def _ray_circle_intersection(self, ray_start: Tuple[float, float],
                                ray_dir: Tuple[float, float],
                                circle_center: Tuple[float, float],
                                radius: float) -> Optional[float]:
        """Calculate intersection distance between ray and circle."""
        ox, oy = ray_start
        dx, dy = ray_dir
        cx, cy = circle_center
        
        # Translate to origin
        ox -= cx
        oy -= cy
        
        # Quadratic equation coefficients
        a = dx*dx + dy*dy
        b = 2 * (ox*dx + oy*dy)
        c = ox*ox + oy*oy - radius*radius
        
        discriminant = b*b - 4*a*c
        if discriminant < 0:
            return None
        
        # Find closest positive intersection
        sqrt_discriminant = math.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2*a)
        t2 = (-b + sqrt_discriminant) / (2*a)
        
        if t1 >= 0:
            return t1
        elif t2 >= 0:
            return t2
        
        return None
    
    def _angle_in_range(self, angle: float, start: float, end: float) -> bool:
        """Check if angle is within range, handling wraparound."""
        # Normalize angles
        angle = self._normalize_angle(angle)
        start = self._normalize_angle(start)
        end = self._normalize_angle(end)
        
        if start <= end:
            return start <= angle <= end
        else:  # Range wraps around
            return angle >= start or angle <= end
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _is_occluded(self, viewer_pos: Tuple[float, float], 
                    target_pos: Tuple[float, float]) -> bool:
        """Check if line of sight between two points is blocked."""
        # Simple occlusion check - cast ray from viewer to target
        dx = target_pos[0] - viewer_pos[0]
        dy = target_pos[1] - viewer_pos[1]
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        
        if distance_to_target == 0:
            return False
            
        ray_dir = (dx / distance_to_target, dy / distance_to_target)
        distance_to_obstacle = self._cast_ray(viewer_pos, math.atan2(dy, dx))
        
        # If obstacle is closer than target, view is occluded
        return distance_to_obstacle < distance_to_target - 0.1  # Small tolerance
    
    def _get_timestamp(self) -> float:
        """Get current timestamp for sensor data."""
        return time.time()

    def randomize_objects(self, seed: Optional[int] = None):
        """Randomize positions of objects_of_interest for each mission.
        Keeps types/confidence but repositions to valid free-space cells.
        """
        rng = random.Random(seed)
        new_objects = []
        
        # Define object types to place
        object_types = [
            ("person", 0.9),
            ("equipment", 0.8),
            ("person", 0.85),
            ("hazard", 0.95),
            ("person", 0.7),
            ("weapon", 0.8),
            ("ied", 0.9)  # The target!
        ]
        
        # Generate positions within valid open areas
        for obj_type, confidence in object_types:
            max_attempts = 50
            for _ in range(max_attempts):
                x = rng.uniform(2, self.width - 2)
                y = rng.uniform(2, self.height - 2)
                
                # Check if position is valid (not in wall or obstacle)
                if not self.check_collision((x, y), radius=1.0):
                    new_objects.append((x, y, obj_type, confidence))
                    break
        
        self.objects_of_interest = new_objects
        
        # Ensure we have at least one IED
        if not any(obj[2] == "ied" for obj in self.objects_of_interest):
            # Place an IED in a valid location
            for _ in range(50):
                x = rng.uniform(5, self.width - 5)
                y = rng.uniform(5, self.height - 5)
                if not self.check_collision((x, y), radius=1.0):
                    self.objects_of_interest.append((x, y, "ied", 0.9))
                    break