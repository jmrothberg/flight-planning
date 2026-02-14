"""
Search Algorithm: BFS-Distance Frontier Exploration + Grid Coverage
VERSION: 11.8

=== CRITICAL: MUST RETURN HOME OR ALL DATA IS LOST ===

SEARCH STRATEGY:
All unsearched free cells reachable via BFS through known free space are
scored. BFS distance (not Manhattan) ensures the drone never targets cells
behind walls. Frontier cells (adjacent to unknown space — doorways, room
edges) get a -8 bonus with extra doorway detection. Wall-adjacent cells get
a +2 penalty. Unreachable cells are automatically excluded.

MULTI-DRONE:
- global_searched (from gossip) prevents re-scanning another drone's work
- global_free motivates exploring areas discovered by others
- claimed_by_others (+15 penalty) prevents two drones targeting the same cell
- Spatial separation penalty keeps drones apart

KEY PARAMETERS:
- Grid: 2m cells (matches 2m IED detector range)
- Return: speed 1.2 m/s, safety = min(70s, 20 + dist*2)
- Stuck: blacklist cell for 8s after 1s of no progress
- Drone: 0.2m square, 0.2m wall clearance
- LiDAR: 59° HFoV, 9m range, rotation scans every 1.5s for 360° awareness
"""

from typing import List, Tuple, Optional, Set, Dict
from collections import deque
import math
import time

class SystematicMapper:
    """
    Grid-based IED search with bonus-based frontier exploration.
    Single drone or multi-drone (via gossip protocol coordination).
    """
    
    def __init__(self, coverage_radius: float = 2.0, drone_id: int = 0,
                 building_width: float = 25.0, building_height: float = 25.0, **kwargs):
        self.grid_size = 2.0  # 2m grid matches IED detector range
        self.drone_id = drone_id  # For multi-drone identification
        # Building bounds (in grid coords) — don't mark cells outside as free
        self.max_grid_x = int(building_width / self.grid_size)
        self.max_grid_y = int(building_height / self.grid_size)
        
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
        
        # Other drones' current positions (for spatial separation penalty)
        self.other_drone_positions: List[Tuple[float, float]] = []

        # Target persistence - don't change target every frame
        self.target_persistence_time: float = 0.0
        self.min_target_hold_time: float = 1.5  # Hold target for 1.5 simulated seconds

        # Stuck detection - track recent positions
        self.position_history: List[Tuple[float, float]] = []
        self.last_position_time: float = 0.0
        self.position_sample_interval: float = 0.4  # Sample every 0.4 simulated seconds
        self.stuck_threshold: float = 0.8  # Consider stuck if moved < 0.8m
        
        # Oscillation detection - track recent target cells
        self.recent_targets: List[Tuple[int, int]] = []
        self.max_recent_targets: int = 10
        
        # Failed targets - temporarily blacklist unreachable targets
        self.failed_targets: Dict[Tuple[int, int], float] = {}  # cell -> failure_time
        self.failed_target_timeout: float = 15.0  # Blacklist for 15 simulated seconds
        
        # Current target
        self.target: Optional[Tuple[float, float]] = None
        self.target_cell: Optional[Tuple[int, int]] = None
        
        # Timing
        self.start_time: Optional[float] = None
        self.entry_point: Optional[Tuple[float, float]] = None
        # exit_point = START position (y=-3) where drone began, must return there
        # entry_point = where drone entered building (inside, y≈2)
        self.exit_point: Optional[Tuple[float, float]] = None

        # Simulated time (passed from simulation_main, not calculated internally)
        self.simulated_elapsed: float = 0.0
        self._mode: str = "SEARCH"  # SEARCH, RUSH, or RETURN for UI display

        # Sweep direction tracking for boustrophedon-style coverage
        self.last_sweep_direction: Optional[Tuple[int, int]] = None
        self.last_searched_cells: List[Tuple[int, int]] = []
        self.max_recent_searched: int = 20
        
        self._last_position = None
        self._needs_rotation_scan = False
        self._no_frontier_logged = False
        print(f"SYSTEMATIC_MAPPER V11.8: Drone {drone_id} - frontier-first exploration for arbitrary buildings")

    def get_next_waypoint(self, position: Tuple[float, float], 
                         lidar_ranges: List[float], 
                         orientation: float) -> Tuple[float, float]:
        """Get next waypoint - efficient grid coverage."""
        
        px, py = position
        
        # Initialize entry point when drone enters building (y >= 1.0 = inside)
        if self.entry_point is None and py >= 1.0:
            self.entry_point = (px, py)
            self.exit_point = (px, -3.0)
        
        # Track position for debug info
        self._last_position = (px, py)
        
        elapsed = self.simulated_elapsed
        time_limit = 420.0  # 7 minutes simulated
        time_remaining = time_limit - elapsed
        
        # Distance-based return - MUST make it back or data is lost!
        if self.exit_point:
            dist_to_exit = math.hypot(px - self.exit_point[0], py - self.exit_point[1])
            # Safety margin scales with distance: 20s base + 2s per meter, capped at 70s
            safety = min(70.0, 20.0 + dist_to_exit * 2.0)
            time_to_return = dist_to_exit / 1.2 + safety
            
            if time_remaining < time_to_return:
                if not getattr(self, '_return_announced', False):
                    cov = self.get_coverage_stats()['coverage_pct']
                    print(f"[RETURN] D{self.drone_id}: {int(elapsed)}s sim, {cov:.0f}% coverage, {dist_to_exit:.0f}m to exit")
                    self._return_announced = True
                self._mode = "RETURN"
                return self.exit_point
        
        # Update map from LIDAR
        self._update_map(px, py, lidar_ranges, orientation)
        
        # Mark current grid cell as searched
        curr_cell = self._to_grid(px, py)
        if curr_cell in self.free_cells:
            self.searched_cells.add(curr_cell)
            if not self.last_searched_cells or self.last_searched_cells[-1] != curr_cell:
                self.last_searched_cells.append(curr_cell)
                if len(self.last_searched_cells) > self.max_recent_searched:
                    self.last_searched_cells = self.last_searched_cells[-self.max_recent_searched:]
        
        # Also mark CARDINAL neighbors as searched (IED detector 2m range).
        # Diagonal neighbors are 2.83m away (center-to-center on 2m grid) = OUT OF RANGE.
        # Wall check: skip if EITHER curr_cell or neighbor is a wall cell.
        # Wall LiDAR endpoints can land in either cell at the boundary — checking
        # both sides ensures IED detection never "reaches through" a wall.
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            neighbor = (curr_cell[0] + dx, curr_cell[1] + dy)
            if neighbor not in self.free_cells:
                continue
            if neighbor in self.wall_cells:
                continue  # Wall endpoint in neighbor cell
            if curr_cell in self.wall_cells:
                continue  # Wall endpoint in our cell — wall might face this neighbor
            self.searched_cells.add(neighbor)
        
        # Check if we reached target
        if self.target:
            dist = math.hypot(px - self.target[0], py - self.target[1])
            if dist < 2.5:
                # If we're close but the cell wasn't actually searched
                # (wall between us and the target), blacklist it so we don't
                # loop forever picking the same unreachable cell.
                if self.target_cell and self.target_cell not in self.searched_cells:
                    self.failed_targets[self.target_cell] = self.simulated_elapsed
                self.target = None
                self.target_cell = None
        
        # Find next target if needed
        if self.target is None:
            rush_mode = elapsed > 180
            self._mode = "RUSH" if rush_mode else "SEARCH"
            self.target = self._find_best_target(px, py, rush_mode)
        
        if self.target:
            return self.target

        # No targets found by _find_best_target.
        # If mission is genuinely complete (all free cells searched), return to exit.
        if self.is_mission_complete():
            self._mode = "RETURN"
            if self.exit_point:
                return self.exit_point
            elif self.entry_point:
                return self.entry_point

        # NOT complete but no targets — return None so caller triggers rotation scan.
        # This is critical: returning exit_point here would send the drone BACK OUTSIDE
        # and the rotation scan (which discovers new cells) would never trigger.
        return None

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

            # Mark cells along ray as FREE (ray passes through = confirmed free)
            # Don't touch wall_cells — both sets are independent.
            # A cell can be in BOTH sets (doorway edge: wall nearby but navigable).
            ray_steps = int(dist / (self.grid_size / 2))
            for s in range(ray_steps):
                r = s * (self.grid_size / 2)
                rx = px + r * math.cos(angle)
                ry = py + r * math.sin(angle)
                cell = self._to_grid(rx, ry)

                if (cell[1] >= 0 and cell[0] >= 0 and
                    cell[0] < self.max_grid_x and cell[1] < self.max_grid_y):
                    self.free_cells.add(cell)

            # Mark endpoint as WALL (don't touch free_cells).
            # Both sets independent: doorway cells can be in both.
            if dist < max_range - 1.0:
                wx = px + dist * math.cos(angle)
                wy = py + dist * math.sin(angle)
                wall_cell = self._to_grid(wx, wy)
                self.wall_cells.add(wall_cell)

    def feed_lidar_scan(self, px: float, py: float, lidar, orientation: float):
        """Feed an additional LiDAR scan into the search algorithm's internal map.

        This MUST be called for every rotation scan so the search algorithm
        discovers doorways and rooms outside the forward 59° FoV cone.
        Without this, the drone only sees what's directly ahead and runs
        out of frontiers prematurely.
        """
        self._update_map(px, py, lidar, orientation)

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
        """Set current positions of OTHER drones (used for spatial separation penalty)."""
        self.other_drone_positions = list(positions)

    def _has_wall_between(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """Check if any wall cell lies on the grid-line between two world positions.

        Uses Bresenham-style grid walk to detect walls separating two drones.
        If a wall is between them, the spatial separation penalty should NOT apply
        because the drones can't actually reach each other's area.
        """
        g = self.grid_size
        gx1, gy1 = int(x1 // g), int(y1 // g)
        gx2, gy2 = int(x2 // g), int(y2 // g)
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx2 > gx1 else -1
        sy = 1 if gy2 > gy1 else -1
        err = dx - dy
        cx, cy = gx1, gy1
        steps = 0
        max_steps = dx + dy + 2
        while steps < max_steps:
            if (cx, cy) in self.wall_cells:
                return True
            if cx == gx2 and cy == gy2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy
            steps += 1
        return False
    
    def set_simulated_time(self, elapsed_seconds: float):
        """Set the simulated elapsed time (called by simulation_main each frame).

        Uses SIMULATED time, not wall-clock. SIM_SPEED=3x means 140 real = 420 sim.
        """
        self.simulated_elapsed = elapsed_seconds
    
    def mark_target_unreachable(self):
        """Immediately blacklist the current target and clear it.

        Called by simulation_main when A* pathfinding returns an empty path,
        so the search algorithm retargets instantly instead of waiting for
        slow stuck detection (which takes ~4.5s of wasted time).
        """
        if self.target_cell:
            self.failed_targets[self.target_cell] = self.simulated_elapsed
            self.target_cell = None
            self.target = None

    def _compute_bfs_distances(self, start_cell: Tuple[int, int]) -> Dict[Tuple[int, int], int]:
        """BFS flood-fill from start_cell through free_cells.

        Returns dict mapping each reachable cell to its BFS distance (in grid steps).
        Cells behind walls (not connected through free space) are simply absent
        from the result, which automatically excludes them as targets.
        """
        distances: Dict[Tuple[int, int], int] = {start_cell: 0}
        queue = deque([start_cell])
        # BFS through cells that are free (navigable)
        # wall_cells are NOT excluded — they may be navigable (doorway edges).
        # Only cells NOT in free_cells are impassable.
        navigable = self.free_cells | self.global_free

        while queue:
            cell = queue.popleft()
            d = distances[cell] + 1
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                neighbor = (cell[0] + dx, cell[1] + dy)
                if neighbor not in distances and neighbor in navigable:
                    distances[neighbor] = d
                    queue.append(neighbor)
        return distances

    def _find_best_target(self, px: float, py: float, rush_mode: bool) -> Optional[Tuple[float, float]]:
        """Find best unsearched cell to visit.

        Single-list nearest-unsearched with exploration bonus:
        - All unsearched free cells are candidates (never starved of targets)
        - Cells adjacent to unknown space get a bonus (discovers new rooms)
        - Sweep preference and region completion for efficient coverage
        - Multi-drone: global_searched prevents duplication, claimed gets penalty
        """
        curr_cell = self._to_grid(px, py)
        now = self.simulated_elapsed

        # Update position history for stuck detection (using simulated time)
        if now - self.last_position_time >= self.position_sample_interval:
            self.position_history.append((px, py))
            self.last_position_time = now
            if len(self.position_history) > 10:
                self.position_history = self.position_history[-10:]

        # Check if stuck
        is_stuck = False
        if len(self.position_history) >= 5:
            old_pos = self.position_history[-5]
            dist_moved = math.hypot(px - old_pos[0], py - old_pos[1])
            is_stuck = dist_moved < self.stuck_threshold

        # Clean up expired failed targets (using simulated time)
        expired = [c for c, t in self.failed_targets.items() if now - t > self.failed_target_timeout]
        for c in expired:
            del self.failed_targets[c]

        # What's been searched? Use global if available (other drones' work)
        searched_set = self.global_searched if self.global_searched else self.searched_cells

        # ============================================================
        # TARGET PERSISTENCE — prevents oscillation between targets
        # ============================================================
        if self.target_cell and self.target:
            dist_to_target = math.hypot(px - self.target[0], py - self.target[1])
            time_on_target = now - self.target_persistence_time

            target_reached = dist_to_target < 2.0
            target_searched = self.target_cell in searched_set
            target_failed = self.target_cell in self.failed_targets
            target_claimed_by_other = self.target_cell in self.claimed_by_others

            if is_stuck and time_on_target > self.min_target_hold_time:
                # Blacklist just this cell (not neighbors — too aggressive)
                self.failed_targets[self.target_cell] = now
                self.target_cell = None
                self.target = None
            elif target_claimed_by_other:
                self.target_cell = None
                self.target = None
            elif not target_reached and not target_searched and not target_failed:
                return self.target  # STICK WITH CURRENT TARGET

        # ===== COLLECT ALL UNSEARCHED FREE CELLS =====
        # Include ALL free cells (wall_cells too — _to_world offsets them).
        # Wall cells get a small penalty in scoring, not hard exclusion.
        # Hard exclusion starved the drone of targets (56% were wall cells!).
        all_candidates = []
        for c in self.free_cells:
            if (c not in searched_set and c[1] >= 0 and
                    c not in self.failed_targets):
                all_candidates.append(c)

        # Also include global_free from other drones
        if self.global_free:
            for c in self.global_free:
                if (c not in searched_set and c[1] >= 0 and
                    c not in self.failed_targets and c not in self.free_cells):
                    all_candidates.append(c)

        if not all_candidates:
            interior_free = len([c for c in self.free_cells if c[1] >= 0])
            if not getattr(self, '_no_frontier_logged', False):
                print(f"D{self.drone_id}: 0 targets ({interior_free} free, {len(self.failed_targets)} failed) — need rotation scan")
                self._no_frontier_logged = True
            self._needs_rotation_scan = True
            return None

        # Compute BFS distances from drone's current cell through free space.
        # Cells unreachable through known free space get no BFS distance,
        # which automatically excludes them as candidates (no more wall-bumping).
        bfs_distances = self._compute_bfs_distances(curr_cell)

        # Filter candidates to only BFS-reachable cells
        reachable_candidates = [c for c in all_candidates if c in bfs_distances]

        # If no reachable candidates, fall back to all candidates with Manhattan
        # (can happen early when map is sparse)
        use_bfs = len(reachable_candidates) > 0
        candidates = reachable_candidates if use_bfs else all_candidates

        # Pre-compute exploration frontiers (adjacent to truly unknown space)
        # Use combined free knowledge so gossip-known cells aren't "unknown"
        combined_free = self.free_cells | self.global_free
        exploration_cells = {}  # cell -> count of unknown neighbors
        for c in candidates:
            unknown_count = 0
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                neighbor = (c[0] + dx, c[1] + dy)
                if neighbor not in combined_free and neighbor not in self.wall_cells:
                    unknown_count += 1
            if unknown_count > 0:
                exploration_cells[c] = unknown_count

        # Pre-compute region richness for frontier cells:
        # Count unsearched free cells in a 3-cell radius (6m).
        # Frontiers near big white (unsearched) areas get a strong pull bonus.
        # This makes the drone go to the big unexplored region instead of
        # picking off small frontier pockets near already-explored areas.
        frontier_richness = {}
        for c in exploration_cells:
            count = 0
            for rx in range(-3, 4):
                for ry in range(-3, 4):
                    nc = (c[0] + rx, c[1] + ry)
                    if nc in combined_free and nc not in searched_set:
                        count += 1
            frontier_richness[c] = count

        def cell_score(c):
            # Base: BFS distance through free space (not Manhattan!)
            # BFS distance respects walls — a cell 3 steps through a doorway
            # scores better than a cell 2 steps with a wall in the way.
            if use_bfs:
                dist = float(bfs_distances.get(c, 999))
            else:
                dist = float(abs(c[0] - curr_cell[0]) + abs(c[1] - curr_cell[1]))

            # EXPLORATION BONUS: Strong preference for frontier cells
            # More unknown neighbors = stronger bonus (doorways to new rooms)
            if c in exploration_cells:
                unknown_count = exploration_cells[c]
                # Base frontier bonus: -8 (was -5)
                # Extra bonus for cells surrounded by unknown: up to -12
                dist -= 8 + unknown_count
                # "Doorway" detection: frontier cell with free cells visible
                # through it (has both free and unknown neighbors) gets extra bonus
                has_free_neighbor = False
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    neighbor = (c[0] + dx, c[1] + dy)
                    if neighbor in combined_free and neighbor not in self.wall_cells:
                        has_free_neighbor = True
                        break
                if has_free_neighbor and unknown_count >= 1:
                    dist -= 5  # Doorway bonus — aggressively targets room entrances

                # REGION RICHNESS: prefer frontiers near large unsearched areas.
                # A frontier next to 30 unsearched cells (big white space) is worth
                # traveling far for.  A frontier next to 3 unsearched cells (small
                # pocket near explored area) is not.
                richness = frontier_richness.get(c, 0)
                if richness > 3:
                    dist -= min(richness * 0.6, 18)  # Up to -18 for rich regions

            # WALL PENALTY: Slight penalty for wall_cells (may get stuck)
            if c in self.wall_cells:
                dist += 2

            # Multi-drone: strong penalty for claimed cells
            if c in self.claimed_by_others:
                dist += 15

            # Multi-drone: prefer cells FAR from other drones (spatial separation)
            # Only penalize if NO wall between us and the other drone (line-of-sight)
            if self.other_drone_positions and self._last_position:
                wx = c[0] * self.grid_size + self.grid_size / 2
                wy = c[1] * self.grid_size + self.grid_size / 2
                for op in self.other_drone_positions:
                    d = math.hypot(wx - op[0], wy - op[1])
                    if d < 15.0:
                        # Skip penalty if a wall separates us from the other drone
                        if self._has_wall_between(
                                self._last_position[0], self._last_position[1],
                                op[0], op[1]):
                            continue
                        dist += (15.0 - d) * 1.3  # Up to +20 penalty at 0m

            # Sweep preference — continue current direction
            if self.last_sweep_direction and len(self.last_searched_cells) >= 2:
                cdx = c[0] - curr_cell[0]
                cdy = c[1] - curr_cell[1]
                if cdx != 0 or cdy != 0:
                    norm_dx = 1 if cdx > 0 else (-1 if cdx < 0 else 0)
                    norm_dy = 1 if cdy > 0 else (-1 if cdy < 0 else 0)
                    if (norm_dx, norm_dy) == self.last_sweep_direction:
                        dist -= 1.5

            # Region completion — prefer cells adjacent to recently searched
            for recent in self.last_searched_cells[-5:]:
                if abs(c[0] - recent[0]) <= 1 and abs(c[1] - recent[1]) <= 1:
                    dist -= 1.0
                    break

            # Rush mode: prefer clusters of unsearched cells
            if rush_mode:
                unsearched_neighbors = self._count_unsearched_neighbors(c)
                dist -= unsearched_neighbors * 0.5

            return dist

        # Frontiers found — reset no-frontier flag
        self._no_frontier_logged = False
        self._needs_rotation_scan = False

        candidates.sort(key=cell_score)
        best = candidates[0]

        # Track target for debugging
        self.recent_targets.append(best)
        if len(self.recent_targets) > self.max_recent_targets:
            self.recent_targets = self.recent_targets[-self.max_recent_targets:]

        # Update sweep direction
        dx = best[0] - curr_cell[0]
        dy = best[1] - curr_cell[1]
        if dx != 0 or dy != 0:
            norm_dx = 1 if dx > 0 else (-1 if dx < 0 else 0)
            norm_dy = 1 if dy > 0 else (-1 if dy < 0 else 0)
            self.last_sweep_direction = (norm_dx, norm_dy)

        self.target_cell = best
        self.target_persistence_time = self.simulated_elapsed
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
        """Convert grid cell to world coordinates (center of cell).

        If the cell is also in wall_cells (wall-adjacent), offset 30% toward
        the nearest pure-free neighbor to avoid wall-hugging targets that
        cause the drone to get stuck.
        """
        cx = cell[0] * self.grid_size + self.grid_size / 2
        cy = cell[1] * self.grid_size + self.grid_size / 2

        if cell in self.wall_cells:
            # Find nearest pure-free neighbor (in free_cells but NOT in wall_cells)
            best_neighbor = None
            best_dist = float('inf')
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    neighbor = (cell[0] + dx, cell[1] + dy)
                    if neighbor in self.free_cells and neighbor not in self.wall_cells:
                        dist = abs(dx) + abs(dy)
                        if dist < best_dist:
                            best_dist = dist
                            best_neighbor = neighbor

            if best_neighbor is not None:
                # Offset 30% toward the pure-free neighbor
                nx = best_neighbor[0] * self.grid_size + self.grid_size / 2
                ny = best_neighbor[1] * self.grid_size + self.grid_size / 2
                cx = cx + 0.3 * (nx - cx)
                cy = cy + 0.3 * (ny - cy)

        return (cx, cy)

    def is_mission_complete(self) -> bool:
        """Check if search is complete (considering all drones' coverage).

        CRITICAL: Must use global_free (from gossip) so a drone doesn't
        declare complete when other drones have discovered more free cells
        that still need searching.
        """
        # Combine local + global knowledge of free space
        all_free = self.free_cells.copy()
        if self.global_free:
            all_free.update(self.global_free)

        interior_free = [c for c in all_free if c[1] >= 0]
        if len(interior_free) < 5:
            return False

        # Use global_searched if available (multi-drone coordination)
        searched_set = self.global_searched if self.global_searched else self.searched_cells
        unsearched = [c for c in interior_free if c not in searched_set]

        return len(unsearched) == 0

    def get_coverage_stats(self) -> dict:
        """Get ACCURATE coverage percentage.

        Only counts cells that are both free AND searched (intersection),
        not cells that were marked searched but never discovered as free.
        """
        interior_free = set(c for c in self.free_cells if c[1] >= 0)
        # Only count searched cells that are actually known free cells
        searched_interior = len(interior_free & self.searched_cells)

        if len(interior_free) == 0:
            return {'coverage_pct': 0}

        pct = int(100 * searched_interior / len(interior_free))
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
        
        # Distance to exit for display
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
            # Mapping stats for understanding exploration progress
            'free_cells': len(interior_free),
            'wall_cells': len(self.wall_cells),
            'searched_cells': len(searched_interior),
            'failed_targets': len(self.failed_targets),
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
        self.exit_point = None
        self.last_sweep_direction = None
        self.last_searched_cells.clear()
        print(f"SYSTEMATIC_MAPPER V11.8: Drone {self.drone_id} reset")
