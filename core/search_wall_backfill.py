"""
Search Method: Wall Backfill Coverage
GOAL: Follow walls reliably AND perform quick 3m interior backfill passes for full coverage
Algorithm: Left-wall follow primary + 3m offset "stripe" backfill targets interleaved
Status: NEW - Minimal, surgical, debug-friendly

==============================================================================
CRITICAL REQUIREMENT:
No pre-knowledge of layout. Use only LIDAR. Discover everything online.
==============================================================================
"""

from typing import List, Tuple, Optional
import math
import time


class CoverageNavigator:
    """
    Hybrid wall-following + interior backfill stripes:
    - Hug left wall for robust navigation
    - Every few meters, queue a 3m offset interior target (quick peek/stripe)
    - Visit queued stripes opportunistically to fill gaps for IED (3m range)
    """

    def __init__(self, wall_distance: float = 3.0, coverage_radius: float = 3.0, **kwargs):
        # Wall follow params
        self.wall_distance = wall_distance
        self._smooth_left_distance = None
        self._smooth_left_angle = None
        self._last_mode = None

        # Coverage/backfill params
        self.coverage_cell_size = max(1.5, min(coverage_radius, 3.0))  # ~IED radius cells
        self.coverage_grid = {}  # (gx,gy)->visits
        self.visited_cells = set()
        self.backfill_queue: List[Tuple[float, float]] = []
        self.backfill_active: Optional[Tuple[float, float]] = None
        self.backfill_cooldown_s = 6.0
        self._last_backfill_ts = 0.0
        self._segment_interval_m = 5.0
        self._last_segment_anchor: Optional[Tuple[float, float]] = None

        # Motion / stuck tracking
        self.last_position = None
        self.stuck_counter = 0
        self.last_stuck_check = time.time()
        self._recent_positions: List[Tuple[float, float]] = []

        # LIDAR usage
        self._max_considered_range = 22.0

        # Debug (concise)
        self.debug = True
        print("WALL_BACKFILL: initialized | wall_distance=%.1f | cell=%.1fm" % (self.wall_distance, self.coverage_cell_size))

    def get_next_waypoint(self,
                          position: Tuple[float, float],
                          lidar_ranges: List[float],
                          orientation: float) -> Optional[Tuple[float, float]]:
        now = time.time()
        px, py = float(position[0]), float(position[1])

        # Update coverage at current pose
        self._update_coverage((px, py))

        # Stuck detection (area of last positions small or no motion)
        if self._is_stuck((px, py), now):
            if self.debug:
                print("WALL_BACKFILL: STUCK -> recovery")
            return self._recovery_move((px, py), orientation, lidar_ranges)

        # If we are in an active backfill run, finish it
        if self.backfill_active is not None:
            tx, ty = self.backfill_active
            if self._distance((px, py), (tx, ty)) < 1.3:
                # Completed backfill hop
                self.backfill_active = None
                self._last_backfill_ts = now
                if self.debug:
                    print("WALL_BACKFILL: backfill done")
            else:
                return self._bound_waypoint(tx, ty)

        # Prefer nearest queued backfill target if cooldown passed
        if self.backfill_queue and (now - self._last_backfill_ts) > self.backfill_cooldown_s:
            # Pick nearest viable backfill target
            best = None
            best_d = 1e9
            for bx, by in list(self.backfill_queue):
                d = self._distance((px, py), (bx, by))
                if d < best_d and self._direction_is_clear((px, py), (bx, by), orientation, lidar_ranges):
                    best = (bx, by)
                    best_d = d
            if best is not None and best_d < 10.0:  # only opportunistic nearby hops
                self.backfill_active = best
                try:
                    self.backfill_queue.remove(best)
                except Exception:
                    pass
                if self.debug:
                    print("WALL_BACKFILL: backfill ->", best)
                return self._bound_waypoint(best[0], best[1])

        # Primary: wall-follow step
        dL, aL = self._find_left_wall(lidar_ranges, orientation)
        if dL is not None and aL is not None:
            # Smoothing
            dL, aL = self._smooth_left(dL, aL)
        
        # Opportunistically queue a new backfill stripe target along the path
        if dL is not None and aL is not None:
            self._maybe_queue_backfill((px, py), aL)

        # Compute wall-follow waypoint
        if dL is None:
            # No wall: move forward to find one
            nx = px + 3.0 * math.cos(orientation)
            ny = py + 3.0 * math.sin(orientation)
            if self.debug and self._last_mode != "no_wall":
                print("WALL_BACKFILL: no_wall -> forward")
            self._last_mode = "no_wall"
            return self._bound_waypoint(nx, ny)

        if dL < self.wall_distance - 0.5:
            # Too close -> move away from wall
            away = aL + math.pi
            nx = px + 2.0 * math.cos(away)
            ny = py + 2.0 * math.sin(away)
            mode = "away"
        elif dL > self.wall_distance + 0.5:
            # Too far -> move toward wall
            nx = px + 2.0 * math.cos(aL)
            ny = py + 2.0 * math.sin(aL)
            mode = "toward"
        else:
            # Good distance -> advance along wall
            forward = aL - math.pi/2.0
            nx = px + 3.0 * math.cos(forward)
            ny = py + 3.0 * math.sin(forward)
            mode = "forward"
        if self.debug and self._last_mode != mode:
            try:
                print(f"WALL_BACKFILL: {mode} dL={dL:.1f}")
            except Exception:
                print("WALL_BACKFILL: ", mode)
        self._last_mode = mode
        return self._bound_waypoint(nx, ny)

    # --- internals ---

    def _update_coverage(self, pos: Tuple[float, float]):
        gx = int(pos[0] / self.coverage_cell_size)
        gy = int(pos[1] / self.coverage_cell_size)
        cell = (gx, gy)
        self.coverage_grid[cell] = self.coverage_grid.get(cell, 0) + 1
        self.visited_cells.add(cell)
        # also pre-mark neighbors to reflect IED 3m sensing overlap
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (gx + dx, gy + dy)
                if n not in self.coverage_grid:
                    self.coverage_grid[n] = 0

        # Track positions for stuck detection
        self._recent_positions.append((pos[0], pos[1]))
        if len(self._recent_positions) > 12:
            self._recent_positions.pop(0)

    def _is_stuck(self, pos: Tuple[float, float], now_ts: float) -> bool:
        # periodic check
        if now_ts - self.last_stuck_check < 2.0:
            return False
        self.last_stuck_check = now_ts
        if self.last_position is None:
            self.last_position = (pos[0], pos[1])
            return False
        dist = self._distance(pos, self.last_position)
        self.last_position = (pos[0], pos[1])
        # also bounding box area of recent points
        stuck_area = 1e9
        if len(self._recent_positions) >= 8:
            xs = [p[0] for p in self._recent_positions[-8:]]
            ys = [p[1] for p in self._recent_positions[-8:]]
            stuck_area = (max(xs) - min(xs)) * (max(ys) - min(ys))
        if dist < 0.6 or stuck_area < 2.0:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        return self.stuck_counter >= 3

    def _recovery_move(self, pos: Tuple[float, float], orientation: float, lidar: List[float]) -> Tuple[float, float]:
        # Choose direction of max free distance
        try:
            if lidar and len(lidar) >= 8:
                best_i = 0
                best_d = -1.0
                step = 2 * math.pi / len(lidar)
                for i, d in enumerate(lidar):
                    val = d if (d is not None and math.isfinite(d)) else 1e6
                    if val > best_d:
                        best_d = val
                        best_i = i
                move = max(2.5, min(4.0, best_d - 0.7))
                ang = orientation + best_i * step
                nx = pos[0] + move * math.cos(ang)
                ny = pos[1] + move * math.sin(ang)
                return self._bound_waypoint(nx, ny)
        except Exception:
            pass
        # fallback forward
        nx = pos[0] + 3.0 * math.cos(orientation)
        ny = pos[1] + 3.0 * math.sin(orientation)
        return self._bound_waypoint(nx, ny)

    def _find_left_wall(self, lidar: List[float], orientation: float) -> Tuple[Optional[float], Optional[float]]:
        if not lidar:
            return None, None
        n = len(lidar)
        step = 2 * math.pi / n
        ls = int(0.125 * n)
        le = int(0.375 * n)
        min_d = float('inf')
        min_a = None
        for i in range(ls, min(le, n)):
            d = lidar[i]
            if d is None:
                continue
            if math.isfinite(d) and d < min_d and d < self._max_considered_range:
                min_d = d
                min_a = orientation + i * step
        if min_a is None:
            return None, None
        return min_d, min_a

    def _smooth_left(self, d: float, a: float) -> Tuple[float, float]:
        if self._smooth_left_distance is None:
            self._smooth_left_distance = d
            self._smooth_left_angle = a
            return d, a
        alpha = 0.3
        self._smooth_left_distance = (1 - alpha) * self._smooth_left_distance + alpha * d
        # angle wrap smoothing
        def angdiff(x, y):
            z = x - y
            while z > math.pi:
                z -= 2 * math.pi
            while z < -math.pi:
                z += 2 * math.pi
            return z
        self._smooth_left_angle = self._smooth_left_angle + alpha * angdiff(a, self._smooth_left_angle)
        return self._smooth_left_distance, self._smooth_left_angle

    def _maybe_queue_backfill(self, pos: Tuple[float, float], left_wall_angle: float):
        # Create stripe target ~1.5m ahead along wall and 3m inside (away from wall)
        forward = left_wall_angle - math.pi/2.0
        inward = left_wall_angle - math.pi  # away from wall into interior
        ahead = 1.5
        offset = 3.0
        target_x = pos[0] + ahead * math.cos(forward) + offset * math.cos(inward)
        target_y = pos[1] + ahead * math.sin(forward) + offset * math.sin(inward)
        # Only queue every ~_segment_interval_m along path
        if self._last_segment_anchor is None:
            self._last_segment_anchor = (pos[0], pos[1])
            return
        if self._distance(pos, self._last_segment_anchor) < self._segment_interval_m:
            return
        self._last_segment_anchor = (pos[0], pos[1])
        # Check bounds and whether it's already covered
        if not (2.0 <= target_x <= 48.0 and 2.0 <= target_y <= 48.0):
            return
        gx = int(target_x / self.coverage_cell_size)
        gy = int(target_y / self.coverage_cell_size)
        if (gx, gy) in self.coverage_grid and self.coverage_grid[(gx, gy)] > 0:
            return
        # Keep queue small & unique
        candidate = (float(target_x), float(target_y))
        if candidate in self.backfill_queue:
            return
        if len(self.backfill_queue) > 30:
            self.backfill_queue.pop(0)
        self.backfill_queue.append(candidate)
        if self.debug:
            print("WALL_BACKFILL: queued", candidate)

    def _direction_is_clear(self, pos: Tuple[float, float], target: Tuple[float, float], orientation: float, lidar: List[float]) -> bool:
        # Quick check via LIDAR beam in target direction
        if not lidar or len(lidar) < 16:
            return True
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        dist = max(0.1, math.hypot(dx, dy))
        ang = math.atan2(dy, dx)
        n = len(lidar)
        step = 2 * math.pi / n
        idx = int(round(((ang - orientation) % (2 * math.pi)) / step)) % n
        beam = lidar[idx]
        if beam is None or not math.isfinite(beam):
            return True
        return beam + 0.3 >= dist  # small safety margin

    def _bound_waypoint(self, x: float, y: float) -> Tuple[float, float]:
        x = max(1.0, min(49.0, x))
        y = max(1.0, min(49.0, y))
        return (x, y)

    @staticmethod
    def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def reset(self):
        print("WALL_BACKFILL: reset")
        self._smooth_left_distance = None
        self._smooth_left_angle = None
        self._last_mode = None
        self.coverage_grid.clear()
        self.visited_cells.clear()
        self.backfill_queue.clear()
        self.backfill_active = None
        self._last_backfill_ts = 0.0
        self._last_segment_anchor = None
        self.last_position = None
        self.stuck_counter = 0
        self.last_stuck_check = time.time()
        self._recent_positions.clear()


