"""
Search Method: Efficient Grid Coverage
VERSION: 11.6a - Fixed entry oscillation bug + better timing

=== CRITICAL: MUST RETURN HOME OR ALL DATA IS LOST ===

KEY PARAMETERS (V11.6a):
- Return speed assumption: 1.2 m/s 
- Safety margin: 70 seconds
- Formula: time_to_return = distance / 1.2 + 70
- Simple formula that doesn't scale badly at far distances

V11.6 BUG FIXES:
- entry_point was inside building (y≈2), but mission_complete needs y<-2
- New: exit_point set to START position (y=-3) - drone returns to where it started
- Return timing formula simplified - V11.6 formula scaled badly at far distances

SIMULATED TIME:
- Simulation runs at SIM_SPEED (3x real time)
- Search algorithm receives simulated time via set_simulated_time()
- All timing logic uses self.simulated_elapsed, NOT time.time()

SEARCH STRATEGY (V11.6 - optimized for complete coverage):
1. Visit nearest UNSEARCHED cell with SWEEP PREFERENCE
2. Prefer cells that continue current sweep direction (avoid zigzag)
3. Prefer cells adjacent to recently-searched cells (complete regions)
4. Rush mode (>180s simulated): Stronger bonus for clustered unsearched cells
5. Return when time_remaining < time_to_return (now less conservative)

=== FOR NEXT LLM: WHAT FAILED ===
- Angular partitioning: angles are relative, change as drone moves
- Euclidean world coords: broke single drone pathfinding  
- Position-based repulsion: caused oscillation with multiple drones
- time.time(): wrong time base - need SIMULATED time
- V11.5h (1.0m/s + 90s): TOO conservative, only got 81% coverage

=== WHAT WORKS ===
- Manhattan distance in GRID coordinates for frontier scoring
- Simple claim system (+5 penalty) for multi-drone coordination
- Sweep preference (continue in same direction)
- Region completion (adjacent to recently searched = bonus)
- Distance-adaptive return margin
"""

from typing import List, Tuple, Optional, Set, Dict
import math
import time

class SystematicMapper:
    """
    Efficient grid-based coverage with multi-drone coordination.
    Based on V9 which achieved 92% - keeping what worked, improving efficiency.
    """
    
    def __init__(self, coverage_radius: float = 3.0, drone_id: int = 0, **kwargs):
        self.grid_size = 3.0  # 3m grid matches IED detector range
        self.drone_id = drone_id  # For multi-drone identification
        
        # Grid tracking (local to this drone)
        self.free_cells: Set[Tuple[int, int]] = set()
        self.wall_cells: Set[Tuple[int, int]] = set()
        self.searched_cells: Set[Tuple[int, int]] = set()
        
        # Global searched cells (from gossip map - all drones combined)
        # This prevents drones from duplicating coverage
        self.global_searched: Set[Tuple[int, int]] = set()
        
        # Global free cells (frontiers from other drones via gossip)
        # This motivates drones to explore areas discovered by others
        self.global_free: Set[Tuple[int, int]] = set()
        
        # Cells claimed by other drones (avoid these)
        self.claimed_by_others: Set[Tuple[int, int]] = set()
        
        # V11.4: Other drones' current positions (for spatial separation)
        # Key insight: prefer frontiers FAR from other drones
        self.other_drone_positions: List[Tuple[float, float]] = []
        
        # Target persistence - don't change target every frame
        # V11.5d: Moderate - balance between responsiveness and stability
        self.target_persistence_time: float = 0.0
        self.min_target_hold_time: float = 1.0  # Hold target for 1.0 seconds
        
        # Stuck detection - track recent positions
        # V11.5d: Less aggressive - give drone time to navigate obstacles
        self.position_history: List[Tuple[float, float]] = []
        self.last_position_time: float = 0.0
        self.position_sample_interval: float = 0.3  # Sample every 0.3s
        self.stuck_threshold: float = 1.5  # Consider stuck if moved < 1.5m
        
        # Oscillation detection - track recent target cells
        self.recent_targets: List[Tuple[int, int]] = []
        self.max_recent_targets: int = 10
        
        # Failed targets - temporarily blacklist unreachable targets
        # V11.5d: Moderate timeout - balance retry speed vs stability
        self.failed_targets: Dict[Tuple[int, int], float] = {}  # cell -> failure_time
        self.failed_target_timeout: float = 12.0  # Blacklist for 12 seconds
        
        # Current target
        self.target: Optional[Tuple[float, float]] = None
        self.target_cell: Optional[Tuple[int, int]] = None
        
        # Timing for efficiency
        self.start_time: Optional[float] = None
        self.entry_point: Optional[Tuple[float, float]] = None
        # V11.6a: Separate exit point (START position) for return - fixes oscillation bug
        # entry_point = where we started exploring (inside, y≈2)
        # exit_point = START position (y=-3) where drone began, must return there
        self.exit_point: Optional[Tuple[float, float]] = None
        
        # V11.6: Simulated time (passed from simulation, not calculated internally)
        self.simulated_elapsed: float = 0.0
        self._mode: str = "SEARCH"  # SEARCH, RUSH, or RETURN for UI display
        
        # V11.6: Track sweep direction for boustrophedon-style coverage
        self.last_sweep_direction: Optional[Tuple[int, int]] = None  # (dx, dy) of last move
        self.last_searched_cells: List[Tuple[int, int]] = []  # Recent cells for region completion
        self.max_recent_searched: int = 20  # Track last N searched cells
        
        print(f"SYSTEMATIC_MAPPER V11.6a: Drone {drone_id} - fixed entry oscillation + better timing")

    def get_next_waypoint(self, position: Tuple[float, float], 
                         lidar_ranges: List[float], 
                         orientation: float) -> Tuple[float, float]:
        """Get next waypoint - efficient grid coverage."""
        
        px, py = position
        
        # Initialize entry point when drone enters building
        if self.entry_point is None and py > 2:
            self.entry_point = (px, py)
            # V11.6a: Set exit_point to START position (where drone began, outside building)
            # Drone starts at y=-3 (outside), enters through door at y=0
            # Must return to start position, not just the door
            self.exit_point = (px, -3.0)
        
        # Track position for debug info
        self._last_position = (px, py)
        
        # V11.5h: Use SIMULATED time, VERY conservative return timing
        elapsed = self.simulated_elapsed
        time_limit = 420.0  # 7 minutes simulated
        time_remaining = time_limit - elapsed
        
        # CRITICAL: Distance-based return - MUST make it back or data is lost!
        # V11.6a: Balanced timing with simple formula, return to EXIT (not entry)
        if self.exit_point:
            dist_to_exit = math.hypot(px - self.exit_point[0], py - self.exit_point[1])
            # V11.6a: Simple formula that doesn't scale badly
            # 1.2 m/s effective speed + 70s margin
            # At 10m: 78s, at 20m: 87s, at 30m: 95s (reasonable scaling)
            time_to_return = dist_to_exit / 1.2 + 70.0
            
            if time_remaining < time_to_return:
                if not getattr(self, '_return_announced', False):
                    cov = self.get_coverage_stats()['coverage_pct']
                    print(f"[RETURN] D{self.drone_id}: {int(elapsed)}s sim, {cov:.0f}% coverage, {dist_to_exit:.0f}m to exit")
                    self._return_announced = True
                # Set mode for UI display
                self._mode = "RETURN"
                # V11.6a: Return to EXIT point (door), not entry point (inside)
                return self.exit_point
        
        # Update map from LIDAR
        self._update_map(px, py, lidar_ranges, orientation)
        
        # Mark current grid cell as searched
        curr_cell = self._to_grid(px, py)
        if curr_cell in self.free_cells:
            self.searched_cells.add(curr_cell)
            # V11.6: Track recently searched cells for region completion preference
            if not self.last_searched_cells or self.last_searched_cells[-1] != curr_cell:
                self.last_searched_cells.append(curr_cell)
                if len(self.last_searched_cells) > self.max_recent_searched:
                    self.last_searched_cells = self.last_searched_cells[-self.max_recent_searched:]
        
        # Also mark nearby cells as searched (IED detector covers 3m radius)
        # BUT only if no wall blocks line-of-sight to that cell
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (curr_cell[0] + dx, curr_cell[1] + dy)
                if neighbor not in self.free_cells:
                    continue  # Not a free cell, skip
                if neighbor in self.wall_cells:
                    continue  # Is a wall, skip
                    
                # For diagonal neighbors, check that we can reach them
                # (not blocked by walls on cardinal directions)
                if dx != 0 and dy != 0:
                    # Diagonal: must be able to go around either way
                    cardinal1 = (curr_cell[0] + dx, curr_cell[1])
                    cardinal2 = (curr_cell[0], curr_cell[1] + dy)
                    if cardinal1 in self.wall_cells and cardinal2 in self.wall_cells:
                        continue  # Both paths blocked, can't reach diagonal
                
                self.searched_cells.add(neighbor)
        
        # Check if we reached target
        if self.target:
            dist = math.hypot(px - self.target[0], py - self.target[1])
            if dist < 2.5:
                self.target = None
                self.target_cell = None
        
        # Find next target if needed
        if self.target is None:
            # V11.6: Earlier rush mode (180s instead of 200s) for better late-game coverage
            rush_mode = elapsed > 180
            self._mode = "RUSH" if rush_mode else "SEARCH"
            self.target = self._find_best_target(px, py, rush_mode)
        
        if self.target:
            return self.target
        
        # No more targets - return to exit
        # V11.6a: Return to EXIT point (door at y=-0.5), not entry point (inside)
        if self.exit_point:
            return self.exit_point
        elif self.entry_point:
            return self.entry_point
        
        # Fallback: move forward
        return (px + 2 * math.cos(orientation), py + 2 * math.sin(orientation))

    def _update_map(self, px: float, py: float, lidar, ori: float):
        """Update grid map from LIDAR scan.

        Accepts dict (new STM 54×42 format) or plain list (legacy 360°).
        """
        if not lidar:
            return

        # Unpack new dict format or fall back to legacy list
        if isinstance(lidar, dict):
            ranges = lidar["ranges"]
            start_angle = lidar["start_angle"]
            angle_step = lidar["angle_step"]
            max_range = 9.0
        else:
            ranges = lidar
            n = len(ranges)
            start_angle = ori
            angle_step = 2 * math.pi / n if n > 0 else 0
            max_range = 12.0

        for i, dist in enumerate(ranges):
            if not math.isfinite(dist) or dist <= 0:
                continue

            angle = start_angle + i * angle_step
            dist = min(dist, max_range)

            # Mark cells along ray as FREE
            ray_steps = int(dist / (self.grid_size / 2))
            for s in range(ray_steps):
                r = s * (self.grid_size / 2)
                rx = px + r * math.cos(angle)
                ry = py + r * math.sin(angle)
                cell = self._to_grid(rx, ry)

                if cell not in self.wall_cells and cell[1] >= 0:
                    self.free_cells.add(cell)

            # Mark endpoint as WALL
            if dist < max_range - 1.0:
                wx = px + dist * math.cos(angle)
                wy = py + dist * math.sin(angle)
                wall_cell = self._to_grid(wx, wy)
                self.wall_cells.add(wall_cell)
                self.free_cells.discard(wall_cell)

    def set_global_searched(self, global_cells: Set[Tuple[int, int]]):
        """
        Update global searched cells from gossip map.
        Called by DroneManager to share knowledge between drones.
        
        Args:
            global_cells: Set of cells searched by ANY drone
        """
        self.global_searched = global_cells.copy()
    
    def set_global_free(self, global_cells: Set[Tuple[int, int]]):
        """
        Update global free cells from gossip map (frontiers from other drones).
        This motivates drones to explore areas discovered by others.
        
        Args:
            global_cells: Set of free cells known by ANY drone
        """
        self.global_free = global_cells.copy()
    
    def set_claimed_by_others(self, claimed_cells: Set[Tuple[int, int]]):
        """
        Update cells claimed by other drones (avoid targeting these).
        
        Args:
            claimed_cells: Set of cells currently claimed by other drones
        """
        self.claimed_by_others = claimed_cells.copy()
    
    def set_other_drone_positions(self, positions: List[Tuple[float, float]]):
        """
        Set current positions of OTHER drones (not used in V11.5e+, kept for compatibility).
        """
        self.other_drone_positions = list(positions)
    
    def set_simulated_time(self, elapsed_seconds: float):
        """
        V11.5g: Set the simulated elapsed time (called by simulation_main).
        
        CRITICAL: The search algorithm must use SIMULATED time, not real wall-clock time.
        The simulation runs at SIM_SPEED (e.g., 3x), so 140 real seconds = 420 simulated.
        
        Args:
            elapsed_seconds: Simulated seconds since mission start
        """
        self.simulated_elapsed = elapsed_seconds
    
    def _find_best_target(self, px: float, py: float, rush_mode: bool) -> Optional[Tuple[float, float]]:
        """Find best unsearched cell to visit.
        
        V11.2: Each drone acts like single drone, but:
        - Uses global_searched to skip cells already searched by ANY drone
        - Yields target if another drone claimed it (no competition)
        - Skips cells already claimed by others when picking new target
        """
        curr_cell = self._to_grid(px, py)
        now = time.time()
        
        # Update position history for stuck detection
        if now - self.last_position_time >= self.position_sample_interval:
            self.position_history.append((px, py))
            self.last_position_time = now
            if len(self.position_history) > 10:
                self.position_history = self.position_history[-10:]
        
        # Check if stuck (hasn't moved much in recent history)
        is_stuck = False
        if len(self.position_history) >= 5:
            old_pos = self.position_history[-5]
            dist_moved = math.hypot(px - old_pos[0], py - old_pos[1])
            is_stuck = dist_moved < self.stuck_threshold
        
        # Clean up expired failed targets
        expired = [c for c, t in self.failed_targets.items() if now - t > self.failed_target_timeout]
        for c in expired:
            del self.failed_targets[c]
        
        # What's been searched? Use global if available (other drones' work)
        searched_set = self.global_searched if self.global_searched else self.searched_cells
        
        # ============================================================
        # CRITICAL: TARGET PERSISTENCE - DO NOT REMOVE OR MODIFY
        # Without this, drones oscillate between targets every frame
        # The drone must STICK to its target until:
        #   1. Target reached (dist < 2.0)
        #   2. Stuck for 3+ seconds (can't reach it)
        #   3. Target already searched by someone else
        #   4. Target in failed list
        #   5. V11.2 FIX: Another drone claimed the same target
        # ============================================================
        if self.target_cell and self.target:
            dist_to_target = math.hypot(px - self.target[0], py - self.target[1])
            time_on_target = now - self.target_persistence_time
            
            # Check if target still valid
            target_reached = dist_to_target < 2.0
            target_searched = self.target_cell in searched_set
            target_failed = self.target_cell in self.failed_targets
            # V11.2 FIX: If another drone claimed this target, yield to them
            target_claimed_by_other = self.target_cell in self.claimed_by_others
            
            # If stuck for >2.5s on same target, blacklist it
            if is_stuck and time_on_target > 2.5:
                print(f"Drone {self.drone_id}: STUCK on target, blacklisting {self.target_cell}")
                self.failed_targets[self.target_cell] = now
                self.target_cell = None
                self.target = None
            # V11.2 FIX: If another drone claimed our target, abandon it and pick new
            elif target_claimed_by_other:
                self.target_cell = None
                self.target = None
            # KEEP current target if still valid (not reached, not searched, not failed)
            elif not target_reached and not target_searched and not target_failed:
                return self.target  # STICK WITH CURRENT TARGET
        
        # ===== FRONTIER SELECTION (like single drone) =====
        # Pick nearest unsearched cell, but skip cells claimed by other drones
        
        # Combine ALL known frontiers (mine + shared from gossip)
        all_frontiers = []
        
        # My frontiers - cells I've seen that aren't searched yet
        for c in self.free_cells:
            if (c not in searched_set and c[1] >= 0 and 
                c not in self.failed_targets):
                all_frontiers.append(c)
        
        # Other drones' frontiers (from gossip) - cells they've seen
        if self.global_free:
            for c in self.global_free:
                if (c not in searched_set and c[1] >= 0 and 
                    c not in self.failed_targets and c not in self.free_cells):
                    all_frontiers.append(c)
        
        if not all_frontiers:
            # Debug: show why no frontiers found
            if len(self.free_cells) == 0 and (not self.global_free or len(self.global_free) == 0):
                print(f"D{self.drone_id}: No free cells known (local or global)")
            else:
                local_unsearched = len([c for c in self.free_cells if c not in searched_set])
                global_unsearched = len([c for c in (self.global_free or set()) if c not in searched_set and c not in self.free_cells])
                print(f"D{self.drone_id}: 0 frontiers (local unsearched: {local_unsearched}, global unsearched: {global_unsearched}, failed: {len(self.failed_targets)})")
            return None
        
        # V11.6: Optimized scoring with sweep preference and region completion
        def frontier_score(c):
            # Base score: Manhattan distance in GRID coordinates (ORIGINAL WORKING LOGIC)
            dist = abs(c[0] - curr_cell[0]) + abs(c[1] - curr_cell[1])
            
            # Small penalty for cells claimed by other drones (not blocking, just preference)
            if c in self.claimed_by_others:
                dist += 5  # Small penalty, not +1000 blocking
            
            # V11.6: SWEEP PREFERENCE - prefer cells that continue current direction
            # This creates more efficient lawnmower-like coverage patterns
            if self.last_sweep_direction and len(self.last_searched_cells) >= 2:
                dx = c[0] - curr_cell[0]
                dy = c[1] - curr_cell[1]
                # If moving in same direction as last move, give bonus
                if (dx != 0 or dy != 0):
                    # Normalize direction
                    norm_dx = 1 if dx > 0 else (-1 if dx < 0 else 0)
                    norm_dy = 1 if dy > 0 else (-1 if dy < 0 else 0)
                    if (norm_dx, norm_dy) == self.last_sweep_direction:
                        dist -= 1.5  # Bonus for continuing sweep direction
            
            # V11.6: REGION COMPLETION - prefer cells adjacent to recently searched
            # This ensures we complete rooms before moving to new areas
            for recent in self.last_searched_cells[-5:]:  # Check last 5 searched
                if abs(c[0] - recent[0]) <= 1 and abs(c[1] - recent[1]) <= 1:
                    dist -= 1.0  # Strong bonus for completing regions
                    break  # Only apply once
            
            # RUSH MODE: stronger bonus for cells with unsearched neighbors
            # V11.6: Increased from 0.3 to 0.5 per neighbor
            if rush_mode:
                unsearched_neighbors = self._count_unsearched_neighbors(c)
                dist -= unsearched_neighbors * 0.5
            
            # NO position-based repulsion! This was causing the "spring" oscillation.
            # Drones spread naturally because:
            # 1. global_searched prevents going where others have already been
            # 2. claimed cells get small penalty
            # 3. Sweep preference keeps them moving efficiently
            
            return dist
        
        # Sort by score (distance + spatial repulsion)
        all_frontiers.sort(key=frontier_score)
        
        # Pick the best one (nearest unsearched, with sweep and region preferences)
        # Target persistence above ensures we stick with chosen target
        best = all_frontiers[0]
        
        # Track this target (for debugging, not for switching)
        self.recent_targets.append(best)
        if len(self.recent_targets) > self.max_recent_targets:
            self.recent_targets = self.recent_targets[-self.max_recent_targets:]
        
        # V11.6: Update sweep direction for next iteration
        dx = best[0] - curr_cell[0]
        dy = best[1] - curr_cell[1]
        if dx != 0 or dy != 0:
            norm_dx = 1 if dx > 0 else (-1 if dx < 0 else 0)
            norm_dy = 1 if dy > 0 else (-1 if dy < 0 else 0)
            self.last_sweep_direction = (norm_dx, norm_dy)
        
        # Update target
        self.target_cell = best
        self.target_persistence_time = now
        return self._to_world(best)

    def _count_unsearched_neighbors(self, cell: Tuple[int, int]) -> int:
        """Count unsearched neighbors - helps find clusters."""
        # Use global_searched if available (avoid re-searching what others found)
        searched_set = self.global_searched if self.global_searched else self.searched_cells
        # Only use OWN free_cells (cells we know are reachable)
        count = 0
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                neighbor = (cell[0] + dx, cell[1] + dy)
                if neighbor in self.free_cells and neighbor not in searched_set:
                    count += 1
        return count

    def _to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid cell."""
        return (int(x / self.grid_size), int(y / self.grid_size))

    def _to_world(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid cell to world coordinates (center of cell)."""
        return (cell[0] * self.grid_size + self.grid_size / 2,
                cell[1] * self.grid_size + self.grid_size / 2)

    def is_mission_complete(self) -> bool:
        """Check if search is complete (considering all drones' coverage)."""
        if len(self.free_cells) < 10:
            return False
        
        # Use global_searched if available (multi-drone coordination)
        searched_set = self.global_searched if self.global_searched else self.searched_cells
        interior_free = [c for c in self.free_cells if c[1] >= 0]
        unsearched = [c for c in interior_free if c not in searched_set]
        
        return len(unsearched) == 0

    def get_coverage_stats(self) -> dict:
        """Get ACCURATE coverage percentage."""
        interior_free = len([c for c in self.free_cells if c[1] >= 0])
        searched_interior = len([c for c in self.searched_cells if c[1] >= 0])
        
        if interior_free == 0:
            return {'coverage_pct': 0}
        
        pct = int(100 * searched_interior / interior_free)
        return {'coverage_pct': min(pct, 100)}

    def get_debug_info(self) -> dict:
        """Debug info for display - shows what user needs to understand search results."""
        # Count interior free cells and how many are searched
        interior_free = set(c for c in self.free_cells if c[1] >= 0)
        searched_interior = set(c for c in self.searched_cells if c[1] >= 0)
        
        # Uncovered = free cells that haven't been searched yet
        uncovered = len(interior_free - searched_interior)
        
        # Time remaining (use simulated time)
        time_remaining = max(0, 420 - self.simulated_elapsed)
        
        # Distance to exit for display (V11.6a: use exit_point for accurate return distance)
        dist_to_entry = 0
        if self._last_position:
            px, py = self._last_position
            if self.exit_point:
                dist_to_entry = math.hypot(px - self.exit_point[0], py - self.exit_point[1])
            elif self.entry_point:
                dist_to_entry = math.hypot(px - self.entry_point[0], py - self.entry_point[1])
        
        return {
            'mode': self._mode,  # Set by get_next_waypoint
            'coverage_pct': self.get_coverage_stats()['coverage_pct'],
            'uncovered': uncovered,
            'time_remaining': int(time_remaining),
            'elapsed': int(self.simulated_elapsed),
            'dist_to_entry': int(dist_to_entry),
        }

    def get_grid_data(self) -> dict:
        """Return grid data for minimap visualization."""
        # Convert grid cells to world coordinates for drawing
        searched_world = []
        for cell in self.searched_cells:
            if cell[1] >= 0:  # Interior only
                wx = cell[0] * self.grid_size + self.grid_size / 2
                wy = cell[1] * self.grid_size + self.grid_size / 2
                searched_world.append((wx, wy))
        
        # Find frontier cells (free cells next to unknown)
        frontier_world = []
        for cell in self.free_cells:
            if cell[1] < 0:
                continue
            for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
                neighbor = (cell[0] + dx, cell[1] + dy)
                if neighbor not in self.free_cells and neighbor not in self.wall_cells:
                    wx = cell[0] * self.grid_size + self.grid_size / 2
                    wy = cell[1] * self.grid_size + self.grid_size / 2
                    frontier_world.append((wx, wy))
                    break
        
        return {
            'searched_cells': searched_world,
            'frontier_cells': frontier_world,
            'grid_size': self.grid_size,
        }

    def reset(self):
        """Reset for new mission."""
        self.free_cells.clear()
        self.wall_cells.clear()
        self.searched_cells.clear()
        self.global_searched.clear()
        self.global_free.clear()
        self.claimed_by_others.clear()
        self.other_drone_positions.clear()
        self.target = None
        self.target_cell = None
        self.target_persistence_time = 0.0
        self.position_history.clear()
        self.last_position_time = 0.0
        self.failed_targets.clear()
        self.recent_targets.clear()
        self.start_time = None
        self.simulated_elapsed = 0.0
        self._return_announced = False
        self._last_position = None
        self._mode = "SEARCH"
        self.entry_point = None
        self.exit_point = None  # V11.6a: Reset exit point
        # V11.6: Reset sweep tracking
        self.last_sweep_direction = None
        self.last_searched_cells.clear()
        print(f"SYSTEMATIC_MAPPER V11.6a: Drone {self.drone_id} reset")
