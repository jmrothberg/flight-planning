# CLAUDE.md — Session Notes for AI Assistants

## What This Project Is

A pygame-based drone simulation for searching unknown buildings to locate IEDs.
Drones use LiDAR (59 deg FoV, 9m range) to map walls, navigate with A* pathfinding,
and cover every 2m grid cell (matching IED detector range). Multi-drone mode uses
a gossip protocol to share maps over simulated mesh radio.

## How to Run

```bash
python3 simulation_main.py
```

Auto-starts mission. Press D to add drones (cycles 1-12). Press R to reset.
SIM_SPEED=3x (140 real seconds = 420 simulated = 7 minutes).

## Key Files (in order of importance for search issues)

| File | What it does |
|------|-------------|
| `core/search_systematic_mapper.py` | **THE search algorithm** — target selection, scoring, stuck detection, return timing |
| `simulation_main.py` | Orchestrator — entry sequence, LiDAR feeding, A* navigation, rotation scans |
| `core/drone_manager.py` | Multi-drone — gossip sync, target claiming, spatial separation, global coverage |
| `core/gossip_map.py` | Distributed map sharing — searched/free/wall cells per drone |
| `core/navigation.py` | A* pathfinder (0.3m grid, 0.2m nav radius, 5000 max iterations) |
| `simulation/physics.py` | Wall collision (0.2m radius pushback) |
| `simulation/environment.py` | Building layouts (5 generators), LiDAR simulation |
| `simulation/graphics.py` | Rendering — minimaps, drone maps, UI panels |

## Current State (V11.8)

### What Works
- Entry sequence (fly to door, enter building, 360 deg scan on entry)
- LiDAR mapping (59 deg FoV cone + rotation scans every 1.5s for 360 deg awareness)
- A* pathfinding with wall collision avoidance
- Bonus-based target scoring (replaces old hard two-phase separation)
- Return timing with distance-based safety margin
- Multi-drone gossip protocol (data flow is correctly wired)
- Per-drone independent timers: 6 min forced return, 7 min battery death
- Wall-aware spatial penalty (drones separated by walls don't interfere)
- Global coverage computed from search algorithms directly (not gossip)
- Door-stuck blacklisting (unreachable targets auto-blacklisted)
- UI: discovered map, per-drone maps, coverage stats, sync indicators

## Architecture Notes

### Building Generation
- 5 layout generators in `environment.py`, randomly selected
- Design scale: 50m, then `building_scale = 0.5` -> actual 25m
- Door gap at x=10 to x=15 (5m wide), `_door_x = 12.5`
- Drone starts at `(_door_x, -3, 3)` (outside, 3m south of wall)

### LiDAR
- ST 3D ToF: 54 horizontal rays, 59 deg HFoV, 9m max range
- Returns dict: `{ranges, start_angle, angle_step, hfov, num_rays}`
- Rotation scans: every 1.5s simulated, ~5 extra scans at rotated orientations
- CRITICAL: `feed_lidar_scan()` MUST be called for rotation scans so the search
  algorithm discovers doorways/rooms outside the forward cone

### Search Algorithm Scoring
```
score = bfs_distance (through free cells, respects walls)
      - 8   if frontier cell (adjacent to unknown space)
      - 5   extra "doorway" bonus if frontier has both free+unknown neighbors
      - min(richness*0.6, 18)  REGION RICHNESS: unsearched cells in 3-cell radius
      + 2   if wall cell
      + 15  if claimed by another drone
      + (15-d)*1.3  if within 15m of another drone AND no wall between (wall-aware!)
      - 1.5 if continues current sweep direction
      - 1.0 if adjacent to recently searched cell
      - 0.5*N if rush mode (N = unsearched neighbors count)
```
Lowest score wins. Region richness is the key to making the drone go to big
unexplored areas instead of picking off small frontier pockets near explored space.

### Stuck Detection
- Position sampled every 0.4s simulated, keep last 10 samples
- Stuck if moved < 0.8m net displacement over last 5 samples (~2.0s)
- Blacklist just the target cell for 15s
- Target hold time: 1.5s

### Navigation Escape (_nav_escape_if_stuck)
- Triggers when drone hasn't moved 0.5m in 1.5 REAL seconds
- 3-second cooldown between escapes to prevent oscillation
- CRITICAL: too-aggressive escape (short timer, no cooldown) causes oscillation!
  The drone escapes backward, retargets same area, goes forward, stuck, escape.
  Breadcrumbs show clustered overlapping dots = oscillation signature.

### A* Navigation Stride (ADAPTIVE — DO NOT USE FIXED STRIDE)
- A* grid: 0.3m resolution, each path step ~0.3m apart
- **Adaptive stride**: try 3, then 2, then 1. Pick the FURTHEST step that has
  a clear straight-line path (`is_path_clear`). Fall back to 1 at corners.
- CRITICAL: stride=1 FIXED causes door oscillation (drone can't commit through doors)
- CRITICAL: stride=3 FIXED causes corner stuck (drone aims through wall at turns)
- The ONLY correct approach is adaptive: try far first, check `is_path_clear`,
  fall back to close. This commits through doors AND follows corners.

### Multi-Drone Timers (HARD DEADLINES)
```
Per-drone (independent timers, start when drone LAUNCHES — first frame):
  360s (6 min): Force returning_home = True (no exceptions)
  420s (7 min): Force mission_complete = True + emergency_stop (battery dead)
```
- All drone clocks start on first frame of _update_multi_drone (NOT when entering building)
- At 7 min the drone is DEAD — no more navigation, no more search
- Mission ends when ALL drones are either home or dead
- Video saved once at mission end

### Multi-Drone Spatial Penalty (Wall-Aware)
- Penalty only applies if there is NO wall between the current drone and the
  other drone (checked via Bresenham grid walk through wall_cells)
- If a wall separates two drones, they are in different rooms and should NOT
  penalize each other's target selection
- Without this, a drone in room A avoids searching cells near the shared wall
  because drone B is on the other side — even though B can't reach those cells

### Global Coverage Calculation
- Uses search algorithm data directly (`search.searched_cells`, `search.free_cells`)
- Does NOT use gossip maps (which are incomplete when drones are out of comm range)
- Formula: `interior_free & all_searched` / `interior_free` (intersection, matching per-drone formula)

### Return Home (Multi-Drone)
- First tries A* path to home (entry_point at y=-3)
- If A* fails (path too long for 5000 iterations), tries intermediate hops
  at 8m, 5m, 3m along the direction toward home
- A* can reliably handle ~8m paths through rooms, but fails for 25m cross-building paths
- Fallback: 8-angle direct navigation, then 24-angle emergency escape

### Physics
- Max drone speed: 2.0 m/s, waypoint tolerance: 1.5m
- Wall collision: 0.2m radius pushback from all wall segments
- A* nav radius: 0.2m, grid resolution: 0.3m

## CRITICAL BUGS FOUND AND FIXED — DO NOT REINTRODUCE

### Bug: Door-Stuck Loop (target cleared but not searched)
**File:** `core/search_systematic_mapper.py`, `get_next_waypoint()` lines ~177-181
**Symptom:** Drone circles at the edge of a doorway forever, never passes through.
**Root cause:** Target is cleared when drone is within 2.5m Euclidean distance.
But 2.5m Euclidean IGNORES WALLS. The drone can be 2.5m from the target center
but on the WRONG SIDE of a wall. The target cell isn't marked as searched (wall
blocks the neighbor search). Then `_find_best_target` selects the same cell again.
Infinite loop.
**Fix:** When clearing a target at 2.5m proximity, check if the cell was actually
searched. If not, add it to `failed_targets` (blacklisted for 15s).
**NEVER:** Remove the `failed_targets` blacklist on target-clear. Without it, the
drone will loop at every doorway.

### Bug: Door-Edge Stuck (A* succeeds but drone can't follow path)
**File:** `simulation_main.py`, `_nav_escape_if_stuck()`
**Symptom:** Drone stuck at door/wall edge for 5-30+ seconds, oscillating.
**Root cause:** A* finds a valid path through the doorway, so `mark_target_unreachable`
is NEVER called (it only fires when A* fails). But wall-collision pushback physically
prevents the drone from following the path. A* keeps succeeding, drone keeps failing.
The search algorithm's stuck detection (1.5s) only blacklists one target at a time,
and there are many targets through the same door = 5-30s wasted.
**Fix:** `_nav_escape_if_stuck()` tracks actual drone position in the main loop.
If the drone hasn't moved 0.3m in 0.5 real seconds, it:
  1. Calls `mark_target_unreachable()` to blacklist the current target
  2. Forces an escape move AWAY from facing direction (back into the room)
  3. Tries 16 angles × 3 step sizes to find a valid escape position
This physically moves the drone away from the wall, then lets A* re-plan.
**NEVER:** Rely solely on the search algorithm's stuck detection for door-edge stuck.
It's too slow (1.5s per target) and doesn't move the drone. The main loop MUST
detect navigation stuck independently and force physical escape.

### Bug: IED Detection Through Walls (green squares behind walls)
**File:** `core/search_systematic_mapper.py`, neighbor marking ~lines 155-174
**Symptom:** A cell on the other side of a wall turns green (marked as searched).
**Root cause (1):** Diagonal neighbors were marked as searched, but diagonal cell
centers are 2.83m apart on a 2m grid — beyond the 2m IED detector range. Removed.
**Root cause (2):** Wall LiDAR endpoints can land in the drone's cell OR the neighbor
cell depending on scan angle. The old code only checked `if neighbor in wall_cells`.
If the wall endpoint was in the drone's cell, the neighbor passed and got marked.
**Fix:** Only mark CARDINAL neighbors (not diagonals). Skip if EITHER curr_cell OR
neighbor is in wall_cells. This catches walls regardless of which side the endpoint
falls on.
**NEVER:** Mark diagonal neighbors as searched (out of IED detector range).
**NEVER:** Only check one side of the wall boundary — always check BOTH cells.

### Bug: Ghost Drone (dead drone keeps flying)
**File:** `simulation_main.py`, `_update_multi_drone()` per-drone loop
**Symptom:** Drone times out (battery dead) but keeps flying around as a dot.
**Root cause:** `drone.update(dt)` and `physics.update_drone()` ran BEFORE the
`mission_complete` check. A dead drone's velocity was still being applied.
**Fix:** Move `mission_complete` check BEFORE physics updates. Dead drones skip
everything: no physics, no movement, no map updates.

### Bug: A* Fixed Stride Causes Either Door Oscillation OR Corner Stuck
**File:** `simulation_main.py`, single-drone ~line 700, multi-drone ~line 1556
**Symptom (stride=1):** Drone oscillates at doorway edge, barely moving.
**Symptom (stride=3):** Drone gets stuck at first corner — aims through wall.
**Root cause:** A* path follows corners in small 0.3m steps. stride=1 is too
timid for doorways (collision pushback bounces it). stride=3 skips the turn
at corners and aims at a point behind the wall.
**Fix:** ADAPTIVE stride: try 3, 2, 1 in order, pick the furthest that passes
`is_path_clear()` check. Commits through doors (stride 3 on straight paths),
follows corners (falls back to stride 1 at turns).
**NEVER:** Use a fixed stride. It will ALWAYS fail at either doors or corners.

### Bug: Multi-Drone Spatial Penalty Through Walls
**File:** `core/search_systematic_mapper.py`, `cell_score()` spatial penalty
**Symptom:** Drone in room A refuses to search cells near wall because drone B
is on the other side. Both drones do nothing near the shared wall.
**Root cause:** `math.hypot` distance ignores walls. Two drones 3m apart but
separated by a wall were penalizing each other's targets by +15.6 points.
**Fix:** `_has_wall_between()` Bresenham grid walk checks for wall cells on the
line between two drones. If wall found, skip spatial penalty entirely.
**NEVER:** Use Euclidean distance alone for spatial penalty. Always check walls.

### Bug: Global Coverage 12% (should be ~60%)
**File:** `core/drone_manager.py`, `get_global_coverage_stats()`
**Root cause:** Old code read from gossip maps, which depend on inter-drone sync.
When drones are out of comm range, gossip data is stale/incomplete.
**Fix:** Read from `search.searched_cells` and `search.free_cells` directly.
These are always accurate regardless of gossip state.
**NEVER:** Use gossip maps for coverage stats. They depend on radio range.

### Bug: D1 Stuck Returning Home
**File:** `simulation_main.py`, `_update_multi_drone()` return path
**Root cause:** A* with 5000 max iterations on 0.3m grid can't pathfind 25m
across an entire building. The drone gets a path failure and inches along via
emergency escape at 0.15m/frame.
**Fix:** When A* fails for home, try intermediate waypoints at 8m, 5m, 3m hops
along the direction toward home. A* handles 5-8m paths reliably.
**NEVER:** Assume A* can pathfind across the entire building in one shot.

### Bug: D1 Clock Starts Late (10-15s delay)
**File:** `simulation_main.py`, `_update_multi_drone()` + entry sequence
**Symptom:** D1 clock shows less time than D0 even though both launched together.
**Root cause:** `mission_start_times[i]` was set when drone enters building (completes
entry sequence), not when it launches. Entry takes 5-15 seconds.
**Fix:** Set ALL `mission_start_times` on the first call to `_update_multi_drone()`.
Removed the overwrite on building entry.
**NEVER:** Set `mission_start_times` on building entry. Battery drains from launch.

### Bug: Combined Map Missing Frontier Markers
**File:** `simulation_main.py`, minimap data construction (~line 1024)
**Symptom:** Individual drone maps (D0 Map, D1 Map) show yellow frontier markers
that do NOT appear on the combined "Discovered Map."
**Root cause:** Two different frontier definitions:
  - Individual maps: `frontier = free - searched - walls` (any unsearched free cell)
  - Combined map: `get_grid_data()` returns only cells adjacent to truly unknown space
The combined map used a STRICTER definition, missing unsearched cells surrounded by
known-but-unsearched space. NOT a communication/gossip issue.
**Fix:** Changed combined map to compute frontiers the same way as individual maps:
collect all free/searched/wall cells from all drones, then frontier = free - searched - walls.
**NEVER:** Use `get_grid_data()['frontier_cells']` for the combined map. Its "frontier"
means "adjacent to unknown" which is not the same as "unsearched."

### Bug: Escape-Induced Oscillation (breadcrumb clusters)
**File:** `simulation_main.py`, `_nav_escape_if_stuck()`
**Symptom:** Drone oscillates back and forth near unexplored areas, wasting time.
Breadcrumbs show overlapping clustered dots.
**Root cause:** Nav escape timer was too aggressive (0.5s real, no cooldown). Drone
gets stuck → escape backward → retarget same area → go forward → stuck → escape.
Infinite loop creating back-and-forth breadcrumb clusters.
**Fix:** Increased stuck timer to 1.5s real. Added 3-second cooldown between escapes.
Search algorithm stuck params: sample 0.4s, threshold 0.8m, hold 1.5s.
**NEVER:** Set nav escape timer < 1.0s or remove the cooldown. Will cause oscillation.

## Things That Were Tried and Failed
- **Hard two-phase separation** (explore ONLY frontiers, then ONLY coverage): Starved
  drone of targets when frontiers were behind walls. Replaced with bonus-based scoring.
- **Wall_cells hard exclusion**: Removed 56% of cells near door. Now penalty-based.
- **5-cell blacklist** (target + 4 neighbors): Too aggressive, blacklisted most of
  small rooms. Reduced to single cell.
- **Breadcrumb-based return fallback**: Disabled — didn't improve return reliability.
- **RRT fallback pathfinding**: Disabled — A* alone is sufficient.
- **A* stride=1 inside building**: Causes door oscillation (can't commit through doors).
- **A* stride=3 FIXED inside building**: Causes corner stuck (cuts corners through walls).
  Must use ADAPTIVE stride with `is_path_clear` check.
- **Euclidean spatial penalty (no wall check)**: Drones on opposite sides of a wall
  interfere with each other. Must use `_has_wall_between()`.
- **Gossip-based coverage stats**: Inaccurate when drones out of comm range.
  Must use search algorithm data directly.
- **min_target_hold_time = 2.0s**: Too slow to retarget when stuck. Reduced to 1.0s.
- **stuck_threshold = 1.0m**: Didn't catch small door oscillations. Reduced to 0.5m.
