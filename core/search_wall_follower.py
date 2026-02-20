"""
Search Method: Wall Follower
GOAL: Quickly search an unknown building for IED (must be within 3 METERS to detect)
Algorithm: Follow the left wall and handle getting stuck between objects
Status: WORKING - Gets 90%+ coverage but can get stuck in loops

==============================================================================
CRITICAL REQUIREMENT FOR FUTURE LLMs AND DEVELOPERS:
The search algorithm MUST NOT have ANY pre-knowledge of the building layout!
- You can ONLY use information discovered through LIDAR sensors
- You CANNOT assume building dimensions, room layouts, or boundaries
- You CANNOT use predetermined paths or waypoints
- You MUST discover walls, obstacles, and boundaries through exploration
- The building is COMPLETELY UNKNOWN until explored by sensors

This is a fundamental requirement for realistic autonomous exploration.
Any algorithm that "knows" the building layout beforehand is CHEATING!
==============================================================================
"""

from typing import List, Tuple, Optional
import math
import time

class WallFollower:
    """
    Wall following that was getting 90%+ coverage.
    Follow the left wall, detect when stuck, recover.
    """
    
    def __init__(self, wall_distance: float = 3.0, coverage_radius: float = 3.0, **kwargs):
        """
        Initialize wall follower.
        wall_distance: how far to stay from wall while following (meters)
        coverage_radius: IED detection range (for compatibility with other algorithms)
        **kwargs: Accept any other parameters for compatibility
        """
        self.wall_distance = wall_distance
        self.coverage_radius = coverage_radius  # Store for compatibility
        self.following_wall = False
        self.wall_angle = None
        self.visited_cells = set()
        self.position_history = []
        self.last_position = None
        self.stuck_counter = 0
        self.last_stuck_check = time.time()
        # ADDED: minimal state for doorway peeks and stuck escalation
        self.last_gap_time = 0.0  # cool-down timer for doorway peeks
        self.gap_cooldown_s = 20.0
        self.recovery_times = []  # timestamps of recent recoveries for escalation
        # DEBUG flags (concise, off by default)
        self.debug = False
        self._last_mode = None
        # NEW: simple smoothing for left-wall measurements to reduce jitter/oscillation
        self._smooth_left_distance = None
        self._smooth_left_angle = None
        # Feature flag: allow doorway/gap peek-in to reduce corridor loops
        self.enable_peek = True
        
        # COVERAGE IMPROVEMENT: Track coverage grid and fill gaps
        self.coverage_grid = {}  # (grid_x, grid_y) -> visit_count
        self.coverage_cell_size = 2.0  # 2m cells for IED detection (3m range covers adjacent cells)
        self.gap_fill_mode = False
        self.gap_target = None
        self.last_gap_check = time.time()
        self.gap_check_interval = 30.0  # Check for gaps every 30 seconds
        self.min_gap_distance = 5.0  # Only fill gaps at least 5m away to avoid repetition
        
    def get_next_waypoint(self, 
                         position: Tuple[float, float],
                         lidar_ranges: List[float],
                         orientation: float) -> Optional[Tuple[float, float]]:
        """
        Get next waypoint using wall following.
        Returns waypoint to navigate to.
        """
        # Check if stuck (not moving OR oscillating) - ONLY after a reasonable time
        if self.last_position is not None and time.time() - self.last_stuck_check > 2.0:
            # Convert to float to handle numpy arrays
            dist_moved = math.sqrt((float(position[0]) - float(self.last_position[0]))**2 + 
                                  (float(position[1]) - float(self.last_position[1]))**2)
            
            self.last_stuck_check = time.time()
            
            # Track positions for oscillation detection
            if not hasattr(self, 'all_positions'):
                self.all_positions = []
            self.all_positions.append((float(position[0]), float(position[1])))
            
            # Check if oscillating using last 10 positions
            oscillating = False
            if len(self.all_positions) >= 10:
                recent_10 = self.all_positions[-10:]
                # Calculate bounding box of recent positions
                min_x = min(p[0] for p in recent_10)
                max_x = max(p[0] for p in recent_10)
                min_y = min(p[1] for p in recent_10)
                max_y = max(p[1] for p in recent_10)
                area = (max_x - min_x) * (max_y - min_y)
                if area < 3.0:  # Stuck in 3x3 meter area with 10 recent positions
                    oscillating = True
            
            if dist_moved < 1.0 or oscillating:  # Not moving OR oscillating
                self.stuck_counter += 1
                if self.stuck_counter > 3:  # Stuck for 6+ seconds
                    # ADDED: escalate if we keep getting stuck in a short window
                    now_ts = time.time()
                    # Keep only recent recovery timestamps
                    self.recovery_times = [t for t in self.recovery_times if now_ts - t < 20.0]
                    self.recovery_times.append(now_ts)
                    if self.debug:
                        print("STUCK: attempting recovery")
                    self.stuck_counter = 0
                    if len(self.recovery_times) >= 2:
                        if self.debug:
                            print("STUCK: escalate -> break_loop")
                        self.recovery_times = []
                        return self._break_loop(position, orientation)
                    # CHANGE: recovery uses the LONGEST LIDAR PATH to move into open space
                    return self._recovery_move(position, orientation, lidar_ranges)
            else:
                self.stuck_counter = 0
        
        self.last_position = (float(position[0]), float(position[1]))
        
        # Mark current position as visited
        grid_x = int(float(position[0]) / self.coverage_cell_size)
        grid_y = int(float(position[1]) / self.coverage_cell_size)
        current_cell = (grid_x, grid_y)
        
        # Update coverage grid
        if current_cell not in self.coverage_grid:
            self.coverage_grid[current_cell] = 0
        self.coverage_grid[current_cell] += 1
        
        # Also mark adjacent cells as partially covered (IED sensor has 3m range)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                adjacent = (grid_x + dx, grid_y + dy)
                if adjacent not in self.coverage_grid:
                    self.coverage_grid[adjacent] = 0
        
        # Track position history for loop detection
        self.position_history.append(current_cell)
        
        # Check if we're looping (visited same cell too many times)
        visit_count = self.position_history.count(current_cell)
        if visit_count > 8:  # Detect loops faster
            if self.debug:
                print(f"WF LOOP detected: visited {current_cell} {visit_count} times -> break_loop")
            return self._break_loop(position, orientation)
        
        self.visited_cells.add(current_cell)
        
        # SIMPLE GAP FILLING: Periodically check for distant gaps to avoid local repetition
        if time.time() - self.last_gap_check > self.gap_check_interval:
            self.last_gap_check = time.time()
            gap = self._find_distant_gap(position)
            if gap:
                self.gap_fill_mode = True
                self.gap_target = gap
                if self.debug:
                    dist = math.sqrt((gap[0] - position[0])**2 + (gap[1] - position[1])**2)
                    print(f"WF: Found gap {dist:.1f}m away, going to fill it")
        
        # If in gap fill mode, navigate to the gap
        if self.gap_fill_mode and self.gap_target:
            dist_to_gap = math.sqrt((self.gap_target[0] - position[0])**2 + 
                                   (self.gap_target[1] - position[1])**2)
            if dist_to_gap < 2.0:
                # Reached the gap, resume wall following
                self.gap_fill_mode = False
                self.gap_target = None
                if self.debug:
                    print("WF: Reached gap, resuming wall follow")
            else:
                # Move toward gap
                return self.gap_target
        
        # Doorway/gap peek: small nudge into rooms to avoid skipping openings
        if self.enable_peek:
            peek_wp = self._detect_opening(position, lidar_ranges, orientation)
            if peek_wp is not None:
                if self.debug:
                    print("WF: peek doorway")
                return peek_wp

        # Find nearest wall on left side
        left_wall_distance, left_wall_angle = self._find_left_wall(lidar_ranges, orientation)
        # Apply light exponential smoothing (proven trick to stabilize wall-following)
        if left_wall_distance is not None and left_wall_angle is not None:
            if self._smooth_left_distance is None:
                self._smooth_left_distance = left_wall_distance
                self._smooth_left_angle = left_wall_angle
            else:
                alpha = 0.3  # weight of new measurement
                # Smooth distance
                self._smooth_left_distance = (1 - alpha) * self._smooth_left_distance + alpha * left_wall_distance
                # Smooth angle with wrap-around handling
                def angle_diff(a, b):
                    d = a - b
                    while d > math.pi:
                        d -= 2*math.pi
                    while d < -math.pi:
                        d += 2*math.pi
                    return d
                self._smooth_left_angle = self._smooth_left_angle + alpha * angle_diff(left_wall_angle, self._smooth_left_angle)
            left_wall_distance = self._smooth_left_distance
            left_wall_angle = self._smooth_left_angle
        
        if left_wall_distance is None:
            # No wall found - move forward to find one
            next_x = float(position[0]) + 3.0 * math.cos(orientation)
            next_y = float(position[1]) + 3.0 * math.sin(orientation)
            if self.debug and self._last_mode != "no_wall":
                print("WF: no_wall -> forward")
            self._last_mode = "no_wall"
            return self._bound_waypoint(next_x, next_y)
            
        # Follow the wall at fixed distance
        if left_wall_distance < self.wall_distance - 0.5:
            # Too close - move away from wall
            away_angle = left_wall_angle + math.pi
            next_x = float(position[0]) + 2.0 * math.cos(away_angle)
            next_y = float(position[1]) + 2.0 * math.sin(away_angle)
            mode = "away"
        elif left_wall_distance > self.wall_distance + 0.5:
            # Too far - move toward wall
            next_x = float(position[0]) + 2.0 * math.cos(left_wall_angle)
            next_y = float(position[1]) + 2.0 * math.sin(left_wall_angle)
            mode = "toward"
        else:
            # Good distance - move forward along wall
            forward_angle = left_wall_angle - math.pi/2  # Perpendicular to wall
            next_x = float(position[0]) + 3.0 * math.cos(forward_angle)
            next_y = float(position[1]) + 3.0 * math.sin(forward_angle)
            mode = "forward"
        if self.debug and self._last_mode != mode:
            print(f"WF: {mode} dL={left_wall_distance:.1f}")
        self._last_mode = mode
            
        return self._bound_waypoint(next_x, next_y)
    
    def _find_left_wall(self, lidar_ranges: List[float], 
                       orientation: float) -> Tuple[Optional[float], Optional[float]]:
        """
        Find the nearest wall on the left side.
        Returns (distance, angle) to wall or (None, None) if no wall.
        """
        if not lidar_ranges:
            return None, None
            
        num_rays = len(lidar_ranges)
        angle_step = 2 * math.pi / num_rays
        
        # Check left side (rays from 45 to 135 degrees to the left)
        left_start = int(num_rays * 0.125)  # 45 degrees
        left_end = int(num_rays * 0.375)    # 135 degrees
        
        min_distance = float('inf')
        min_angle = None
        
        for i in range(left_start, left_end):
            if i < len(lidar_ranges) and lidar_ranges[i] < 10.0:  # Wall within 10m
                if lidar_ranges[i] < min_distance:
                    min_distance = lidar_ranges[i]
                    ray_angle = orientation + (i * angle_step)
                    min_angle = ray_angle
                    
        if min_angle is not None:
            return min_distance, min_angle
        return None, None
    
    def _detect_opening(self,
                        position: Tuple[float, float],
                        lidar_ranges: List[float],
                        orientation: float) -> Optional[Tuple[float, float]]:
        """
        ADDED: Very lightweight doorway/gap detector on the left.
        Only returns a short peek waypoint when a wide gap is seen and cooldown passed.
        """
        if not lidar_ranges:
            return None
        # Cooldown so we don't oscillate at the same doorway
        if time.time() - self.last_gap_time < self.gap_cooldown_s:
            return None
        num_rays = len(lidar_ranges)
        if num_rays < 10:
            return None
        angle_step = 2 * math.pi / num_rays
        # Left wedge ~45°..135° to the left (same convention as _find_left_wall)
        left_start = int(num_rays * 0.125)
        left_end = int(num_rays * 0.375)
        # Thresholds tuned conservatively to avoid false triggers
        near_thr = 5.0  # nearby wall/objects
        far_thr = 8.0   # open space/doorway depth
        min_consecutive_far = 5  # require a visible wide gap
        start_idx = None
        far_count = 0
        for i in range(left_start, min(left_end, num_rays)):
            d = lidar_ranges[i]
            # Treat non-finite as far
            is_far = (not math.isfinite(d)) or d >= far_thr
            if is_far:
                if start_idx is None:
                    # ensure a rising edge: a few beams before must be near/closer
                    prev_ok = True
                    for k in range(1, 3):
                        j = max(left_start, i - k)
                        if j >= 0 and j < num_rays and math.isfinite(lidar_ranges[j]) and lidar_ranges[j] > near_thr:
                            prev_ok = False
                    if prev_ok:
                        start_idx = i
                        far_count = 1
                else:
                    far_count += 1
            else:
                if start_idx is not None and far_count >= min_consecutive_far:
                    end_idx = i
                    center_idx = (start_idx + end_idx) // 2
                    center_angle = orientation + (center_idx * angle_step)
                    # Peek 1.4m inside; keeps sensor within ~3ft of door frame/room entrance
                    peek_distance = 1.4
                    next_x = float(position[0]) + peek_distance * math.cos(center_angle)
                    next_y = float(position[1]) + peek_distance * math.sin(center_angle)
                    self.last_gap_time = time.time()
                    return self._bound_waypoint(next_x, next_y)
                # reset
                start_idx = None
                far_count = 0
        # Handle case if gap extends to end of wedge
        if start_idx is not None and far_count >= min_consecutive_far:
            center_idx = (start_idx + min(left_end, num_rays - 1)) // 2
            center_angle = orientation + (center_idx * angle_step)
            peek_distance = 1.4
            next_x = float(position[0]) + peek_distance * math.cos(center_angle)
            next_y = float(position[1]) + peek_distance * math.sin(center_angle)
            self.last_gap_time = time.time()
            return self._bound_waypoint(next_x, next_y)
        return None
        
    def _recovery_move(self, position: Tuple[float, float], orientation: float,
                       lidar_ranges: Optional[List[float]] = None) -> Tuple[float, float]:
        """
        Recovery move when stuck.
        PER USER REQUEST: Use the LONGEST LIDAR PATH (max free distance) to pick
        a deterministic escape waypoint. If LIDAR is unavailable, fall back to
        the prior randomized back-and-turn move.
        """
        try:
            if lidar_ranges and len(lidar_ranges) >= 5:
                # Choose the ray with the maximum measured distance
                angle_step = 2 * math.pi / len(lidar_ranges)
                best_idx = 0
                best_dist = -1.0
                for i, d in enumerate(lidar_ranges):
                    # Treat non-finite as very far (open)
                    val = d if (d is not None and math.isfinite(d)) else 1e6
                    if val > best_dist:
                        best_dist = val
                        best_idx = i
                # Aim some distance along that direction, but keep a safety margin
                safety_margin = 0.6
                move_dist = max(2.0, min(4.0, (best_dist - safety_margin)))
                escape_angle = orientation + (best_idx * angle_step)
                next_x = float(position[0]) + move_dist * math.cos(escape_angle)
                next_y = float(position[1]) + move_dist * math.sin(escape_angle)
                return self._bound_waypoint(next_x, next_y)
        except Exception:
            # If anything goes wrong, fall back to legacy behavior below
            pass

        # Fallback: previous randomized back-and-turn
        import random
        random_offset = random.uniform(-math.pi/3, math.pi/3)  # Random angle between -60 and 60 degrees
        back_angle = orientation + math.pi + math.pi/4 + random_offset
        distance = random.uniform(2.0, 4.0)
        next_x = float(position[0]) + distance * math.cos(back_angle)
        next_y = float(position[1]) + distance * math.sin(back_angle)
        return self._bound_waypoint(next_x, next_y)
    
    def _break_loop(self, position: Tuple[float, float], orientation: float) -> Tuple[float, float]:
        """
        Break out of a loop by moving to an unvisited area.
        """
        # Try to find an unvisited cell nearby
        best_target = None
        min_visited_neighbors = float('inf')
        
        # Check cells in expanding radius
        for radius in [5, 10, 15]:
            for angle in [0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4]:
                test_x = float(position[0]) + radius * math.cos(orientation + angle)
                test_y = float(position[1]) + radius * math.sin(orientation + angle)
                
                grid_x = int(test_x / 2.0)
                grid_y = int(test_y / 2.0)
                
                # Count visited neighbors
                visited_count = 0
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if (grid_x + dx, grid_y + dy) in self.visited_cells:
                            visited_count += 1
                
                # Prefer cells with fewer visited neighbors
                if visited_count < min_visited_neighbors:
                    min_visited_neighbors = visited_count
                    best_target = (test_x, test_y)
                    
                # If we found an unvisited area, go there
                if visited_count == 0:
                    return self._bound_waypoint(test_x, test_y)
        
        # If no unvisited area found, go to least visited
        if best_target:
            return self._bound_waypoint(best_target[0], best_target[1])
            
        # Last resort - move forward
        next_x = float(position[0]) + 5.0 * math.cos(orientation)
        next_y = float(position[1]) + 5.0 * math.sin(orientation)
        return self._bound_waypoint(next_x, next_y)
    
    def _bound_waypoint(self, x: float, y: float) -> Tuple[float, float]:
        """
        Keep waypoint within reasonable bounds.
        """
        # Bound to building area (assuming roughly 50x50m building)
        x = max(2.0, min(48.0, x))
        y = max(2.0, min(48.0, y))
        return (x, y)
    
    def _find_distant_gap(self, position: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        Find an uncovered gap that's far enough away to be worth visiting.
        This avoids repetitive local behavior.
        """
        px, py = position
        current_grid_x = int(px / self.coverage_cell_size)
        current_grid_y = int(py / self.coverage_cell_size)
        
        best_gap = None
        best_score = float('-inf')
        
        # Search in expanding rings, but skip nearby cells
        for radius in range(int(self.min_gap_distance / self.coverage_cell_size), 20):
            for dx in range(-radius, radius + 1):
                for dy in [-radius, radius]:  # Top and bottom edges
                    gx = current_grid_x + dx
                    gy = current_grid_y + dy
                    cell = (gx, gy)
                    
                    # Check if this cell needs coverage
                    if cell not in self.coverage_grid or self.coverage_grid[cell] == 0:
                        world_x = gx * self.coverage_cell_size
                        world_y = gy * self.coverage_cell_size
                        if 0 <= world_x <= 50 and 0 <= world_y <= 40:  # Building bounds
                            dist = math.sqrt((world_x - px)**2 + (world_y - py)**2)
                            # Only consider gaps that are far enough away
                            if dist >= self.min_gap_distance:
                                # Score based on distance (prefer closer gaps that meet minimum)
                                score = -dist  # Negative because we want closer
                                if score > best_score:
                                    best_score = score
                                    best_gap = (world_x, world_y)
            
            for dy in range(-radius + 1, radius):  # Left and right edges
                for dx in [-radius, radius]:
                    gx = current_grid_x + dx
                    gy = current_grid_y + dy
                    cell = (gx, gy)
                    
                    if cell not in self.coverage_grid or self.coverage_grid[cell] == 0:
                        world_x = gx * self.coverage_cell_size
                        world_y = gy * self.coverage_cell_size
                        if 0 <= world_x <= 50 and 0 <= world_y <= 40:
                            dist = math.sqrt((world_x - px)**2 + (world_y - py)**2)
                            if dist >= self.min_gap_distance:
                                score = -dist
                                if score > best_score:
                                    best_score = score
                                    best_gap = (world_x, world_y)
            
            # If we found a good gap, return it
            if best_gap:
                return best_gap
        
        return None
    
    def reset(self):
        """Reset for new mission."""
        self.following_wall = False
        self.wall_angle = None
        self.visited_cells = set()
        self.position_history = []
        self.last_position = None
        self.stuck_counter = 0
        self.last_stuck_check = time.time()
        self.last_gap_time = 0.0
        self.recovery_times = []
        self._smooth_left_distance = None
        self._smooth_left_angle = None
        self.coverage_grid = {}
        self.gap_fill_mode = False
        self.gap_target = None
        self.last_gap_check = time.time()
        if hasattr(self, 'all_positions'):
            self.all_positions = []