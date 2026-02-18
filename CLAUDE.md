# CLAUDE.md — Session Notes for AI Assistants

## What This Project Is

A pygame-based drone simulation for searching unknown buildings to locate IEDs.
Drones use LiDAR (59 deg FoV, 9m range) to map walls, navigate with A* pathfinding,
and cover every 2m grid cell (matching IED detector range). Multi-drone mode uses
a gossip protocol to share maps over simulated mesh radio.

## How to Run

```bash
python3 simulation_gui.py
```

Auto-starts mission. Press D to add drones (cycles 1-12). Press R to reset.
Click a drone + M for manual control. Click [MAP]/[DSTR] to change mission.
SIM_SPEED=3x (140 real seconds = 420 simulated = 7 minutes).

## Key Files (in order of importance for search issues)

| File | What it does |
|------|-------------|
| `core/stm32n6/search_systematic_mapper.py` | **THE search algorithm** — target selection, scoring, stuck detection, return timing |
| `simulation_gui.py` | Orchestrator — entry sequence, LiDAR feeding, A* navigation, rotation scans |
| `core/drone_manager.py` | Multi-drone — gossip sync, target claiming, spatial separation, global coverage |
| `core/stm32wl/gossip_map.py` | Distributed map sharing — searched/free/wall cells per drone |
| `core/stm32n6/navigation.py` | A* pathfinder (0.3m grid, 0.2m nav radius, 5000 max iterations) |
| `core/processor_bus.py` | Inter-processor message types (SPI/UART data channels) |
| `core/stm32h7/stm32h7_main.py` | Flight controller app — PID, motors, GPS, battery |
| `core/stm32n6/stm32n6_main.py` | Neural processor app — LiDAR, SLAM, search, A* |
| `core/stm32wl/stm32wl_main.py` | Mesh radio app — gossip protocol, inter-drone comms |
| `simulation/physics.py` | Wall collision (0.2m pushback), returns blocked speed for collision reporting |
| `simulation/environment.py` | Building layouts (5 generators), LiDAR simulation |
| `simulation/graphics.py` | Rendering — minimaps, drone maps, UI panels, mission buttons |
| `simulation/joystick_widget.py` | Manual flight control — dual-stick joystick panel |

## Current State (V12.3)

### What Works
- Entry sequence (fly to door, enter building, 360 deg scan on entry)
- LiDAR mapping (59 deg FoV cone + rotation scans every 1.5s for 360 deg awareness)
- A* pathfinding with wall collision avoidance
- Bonus-based target scoring with region richness (pulls to big unsearched areas)
- Return timing with distance-based safety margin
- Multi-drone gossip protocol (data flow is correctly wired)
- Per-drone independent timers: 6 min forced return, 7 min battery death
- Distance-based spatial penalty (pushes drones apart, even through walls)
- Global coverage computed from search algorithms directly (not gossip)
- Door-stuck blacklisting (unreachable targets auto-blacklisted)
- IED detection at actual IED position (not drone position)
- Screenshots: per-drone mid (3 min) + per-drone done (no global sim screenshots)
- UI: discovered map, per-drone maps, coverage stats, sync indicators
- **Manual drone control** (click drone, press M, drag joystick sticks)
- **Per-drone mission system** — MAP (default search) or DESTROY (fly to IED and detonate)
- **IED destruction** — drones in DESTROY mode fly to known IED positions, eliminate them
- **Wall collision reporting** — counts real impacts (>0.5 m/s), shown in per-drone status
- **Unified single/multi UI** — per-drone status, map, and mission buttons always shown
- **Base station (radio-only)** — Discovered Map shows only what was radioed back
- **Radio-filtered walls** — base station walls are LiDAR segments filtered to radio-known areas
- **Objects Found from radio** — UI panel reads from base station gossip, not direct drone data
- **Base station data preserved** — adding/removing drones doesn't wipe the discovered map
- **Manual radio mode toggle** — L key or click button to switch Long-Range (H7) vs Mesh (WL)

## DESIGN PRINCIPLES

### LiDAR-Only Search
Drones must navigate and search using ONLY what they discover via LiDAR.
NEVER use pre-programmed building knowledge (zones, floor plans, room layouts)
for search decisions. The building is UNKNOWN until scanned. Shared data from
other drones (via gossip radio) is allowed because it's sensor-derived.

### Scalability (N Drones)
All multi-drone logic MUST scale to 3, 4, ... N drones. Never hard-code
assumptions about 2 drones. The spatial penalty, gossip protocol, coverage
stats, and target scoring all use loops over `drone_manager.count`.
When adding multi-drone features, always think: "does this work for 12 drones?"

### Future: 3D, Multi-Floor, Hallways
The current simulation is 2D for simplification. Future versions will add:
- Multiple floors (stairs/elevators as vertical connections)
- Hallways and corridors (long narrow spaces)
- Tunnels and underground structures
- Full 3D navigation (altitude as third coordinate)

Design search algorithms so they don't assume 2D. The BFS flood-fill,
A* pathfinding, and grid-based coverage can all extend to 3D by adding
a Z coordinate to grid cells. The gossip protocol is already dimension-agnostic.
Keep the grid abstraction clean — when 3D arrives, the grid becomes voxels.

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

### Camera (Vision System)
- OV2640 simulated: 60 deg FoV, 8m range
- Detects objects: person, equipment, hazard, weapon, furniture
- Occlusion-aware (can't see through walls), confidence decays with distance
- `environment.get_camera_view()` returns visible objects in FoV
- `vision.process_frame()` converts to Detection objects (type, position, confidence)
- Single-drone: detections stored via `minimap.add_vision_detection()`
- Multi-drone: detections stored via `gossip.add_local_feature()` (shared between drones)
- Building has 7 randomized objects: 3 people, 1 equipment, 1 hazard, 1 weapon, 1 IED

### IED Sensor
- 2m proximity range, triggers when drone passes near an IED
- Returns `IEDReading` with `ied_position` (actual IED location, not drone position)
- Grid uses 2m cells to match sensor range — visiting every cell = full coverage

### Search Algorithm Scoring
```
score = bfs_distance (through free cells, respects walls)
      - 8   if frontier cell (adjacent to unknown space)
      - 5   extra "doorway" bonus if frontier has both free+unknown neighbors
      - min(richness*0.7, 22)  REGION RICHNESS: unsearched cells in 3-cell radius (ALL candidates)
      + 2   if wall cell
      + 15  if claimed by another drone
      + (20-d)*0.8  if within 20m of another drone (pushes apart even through walls)
      - 1.5 if continues current sweep direction
      - 1.0 if adjacent to recently searched cell
      - 0.5*N if rush mode (N = unsearched neighbors count)
```
Lowest score wins. Region richness applies to ALL candidates (not just frontiers)
because gossip can de-frontier cells by filling in "unknown" neighbors with other
drones' free_cells. Without richness on non-frontier cells, drones ignore big
unsearched areas that lost frontier status via gossip sync.

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

### Multi-Drone Spatial Penalty (Distance-Based)
- Penalty applies within 20m based on Euclidean distance to other drones
- Applied EVEN through walls — drones in different rooms still push apart
  so they explore different areas of the building
- The richness bonus (-22 max) ensures wall-adjacent cells still get explored
- Uses only shared sensor data (positions from radio) — no pre-knowledge
- CRITICAL: Never use building layout knowledge for search decisions.
  Drones must navigate using ONLY what they discover via LiDAR.

### Base Station (Ground Controller)
- Fixed position at drone launch point (entry_point, typically `(_door_x, -3)`)
- Has its own GossipMap + MeshProtocol + MeshNode in DroneManager
- Participates in gossip exchange: drones in comm range sync directly with base station
- Drone positions propagate through gossip (`known_positions` dict in GossipMap)
- The "Discovered Map" reads ONLY from the base station's gossip — not from search
  algorithms. This means it shows only what was physically radioed to the operator.
- If a drone is out of comm range and no relay exists, base station has no data from it
- Single-drone mode: lightweight base station gossip syncs when drone is in range
- `BASE_STATION_ID = -1` constant in `drone_manager.py`
- **Wall segments**: LiDAR segments are FILTERED to only show walls in areas the base
  station has received gossip data about. Uses expanded cell neighborhood check:
  precompute `known_area` from gossip cells ± 1, then include only segments whose
  midpoint grid cell is in `known_area`. This gives LiDAR-quality wall rendering
  without showing walls the radio hasn't delivered yet.
- **Objects Found panel**: reads from base station gossip features, not drone-local data.
  Only objects that were radioed to the base station appear in the UI.
- **Data preservation**: when adding/removing drones (pressing D), the base station's
  accumulated gossip data is saved and merged into the new DroneManager's base station.
  Uses `get_sync_payload()` → change sender_id → `merge_remote_update()`.
- NEVER use search algorithm data for the Discovered Map — that would be cheating.
  The operator can only see what the radio delivered.
- NEVER use minimap wall_segments directly — they are drone-local LiDAR data.
  Always filter through the base station's known cell set.

### Global Coverage Calculation
- Uses search algorithm data directly (`search.searched_cells`, `search.free_cells`)
- Does NOT use gossip maps (which are incomplete when drones are out of comm range)
- Formula: `interior_free & all_searched` / `interior_free` (intersection, matching per-drone formula)
- NOTE: Global coverage is a debug/analysis metric, not what the operator sees.
  The operator's view (Discovered Map) comes from the base station gossip.

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

### Manual Drone Control
- Click a drone on the main map to select it (1m hit radius, yellow ring)
- Press **M** to toggle manual mode (joystick panel appears)
- Left stick: forward/back + strafe. Right stick: yaw rotation
- Drone cells are still marked as searched during manual flight (IED sensor stays active)
- 6-min timer auto-releases manual mode for forced return
- Only one drone in manual mode at a time. Selecting a new drone releases the old one.
- **File:** `simulation/joystick_widget.py` (JoystickPanel), `simulation_gui.py` (_apply_manual_input_to_drone)

### Manual Control Radio Modes
- **Long-Range Radio (default)** — Direct 900 MHz link through STM32H7. Unlimited range.
  Current behavior: joystick commands always reach the drone regardless of distance.
- **Mesh Radio** — Joystick commands travel through STM32WL mesh network. Commands hop
  from base station through intermediate drones to reach the controlled drone. Only works
  while there is a radio path (direct or multi-hop) from the base station.
- Press **L** or click the radio mode button in Controls section to toggle.
- When mesh mode is active and the link drops (drone out of range), the drone hovers
  (zero velocity) — the pilot has no control until the link is re-established.
- Joystick panel shows "MESH: LINKED" (green) or "MESH: NO LINK" (red) when in mesh mode.
- Reachability check: BFS from BASE_STATION_ID through `get_gossip_links()` adjacency.
  Single-drone mode: simple distance check to base station within `comm_range`.
- **State:** `_manual_radio_mode` — "longrange" or "mesh". Reset to "longrange" on sim reset.
- **File:** `simulation_gui.py` (_is_drone_mesh_reachable, _manual_radio_mode)

### Per-Drone Mission System
- Two missions: `"map"` (default autonomous search) and `"destroy"` (fly to IED, detonate)
- Stored in `_drone_missions: Dict[int, str]` — default is `"map"` for every drone
- UI: clickable `[MAP]` / `[DSTR]` button per drone in the Per-Drone Status panel
- **Destroy behavior:**
  1. On click [DSTR], `_find_known_ied(drone_id)` checks gossip features (multi-drone)
     or minimap discovered_features (single-drone) for IED positions not yet destroyed
  2. If IED is known, stores position in `_drone_destroying[drone_id]` and overrides
     navigation target to fly there using A*
  3. Each frame in DESTROY mode: if no target yet, re-checks map (gossip may have
     delivered new IED data from another drone)
  4. When drone arrives within 0.5m of IED: removes it from `environment.objects_of_interest`,
     adds position to `_destroyed_ieds` set, reverts drone to MAP mission
  5. Destroyed IEDs shown as red X circles on the main map
  6. Objects Found panel shows "IED destroyed: N" in red when N > 0
- **NEVER:** Remove IED from gossip/minimap — only from environment.objects_of_interest.
  The sensor reads from that list, so removed IEDs won't be re-detected.

### Wall Collision Reporting
- `physics.update_drone()` returns blocked speed (float, m/s) — the speed at which
  the drone was moving when the physics engine zeroed its velocity to prevent wall entry
- Proximity pushback from `_handle_collisions()` is NOT counted (that's normal clearance)
- A collision is counted only when blocked_speed > 0.5 m/s AND 2-second per-drone cooldown
- In autonomous mode with LiDAR, collision count should be ~0 (A* avoids walls)
- Manual flight into walls will increment the counter
- Shown as `W:N` in per-drone status (orange if > 0, gray if 0)
- Logged to console: `[COLLISION] D{i}: wall hit #{count} at (x,y) (speed m/s)`

### Unified Single/Multi-Drone UI
- Per-Drone Status section always renders (even with 1 drone)
- `_build_single_drone_ui_data()` produces the same dict format as `_build_multi_drone_ui_data()`
- Per-drone map always rendered via `_render_single_drone_map()` for 1 drone
- Mission buttons `[MAP]`/`[DSTR]` and wall collision count `W:N` shown for all drones
- WL Mesh Radio section shows "Single drone — no mesh active" for 1 drone,
  link count for 2+ drones

## CRITICAL BUGS FOUND AND FIXED — DO NOT REINTRODUCE

### Bug: IED Labels at Wrong Positions (drone pos instead of IED pos)
**File:** `core/stm32n6/sensors.py`, `simulation_gui.py`
**Symptom:** "IED!" labels appear at multiple wrong locations on maps.
The actual IED is in one room but labels scatter across the building.
**Root cause:** `add_ied_detection()` and `add_local_feature("ied", ...)` were
called with the DRONE's position, not the actual IED position. The IED sensor
knew the IED location (to calculate distance) but didn't return it.
**Fix:** Added `ied_position` field to `IEDReading`. Both single-drone and
multi-drone detection paths now use `ied_reading.ied_position` as the label
coordinate. The IED sensor already had the position — it just wasn't returned.
**NEVER:** Use drone position as IED detection position. The sensor knows where
the IED is — always use `ied_reading.ied_position`.

### Bug: Door-Stuck Loop (target cleared but not searched)
**File:** `core/stm32n6/search_systematic_mapper.py`, `get_next_waypoint()`
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
**File:** `simulation_gui.py`, `_nav_escape_if_stuck()`
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
**File:** `core/stm32n6/search_systematic_mapper.py`, neighbor marking
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
**File:** `simulation_gui.py`, `_update_multi_drone()` per-drone loop
**Symptom:** Drone times out (battery dead) but keeps flying around as a dot.
**Root cause:** `drone.update(dt)` and `physics.update_drone()` ran BEFORE the
`mission_complete` check. A dead drone's velocity was still being applied.
**Fix:** Move `mission_complete` check BEFORE physics updates. Dead drones skip
everything: no physics, no movement, no map updates.

### Bug: A* Fixed Stride Causes Either Door Oscillation OR Corner Stuck
**File:** `simulation_gui.py`, A* stride logic
**Symptom (stride=1):** Drone oscillates at doorway edge, barely moving.
**Symptom (stride=3):** Drone gets stuck at first corner — aims through wall.
**Root cause:** A* path follows corners in small 0.3m steps. stride=1 is too
timid for doorways (collision pushback bounces it). stride=3 skips the turn
at corners and aims at a point behind the wall.
**Fix:** ADAPTIVE stride: try 3, 2, 1 in order, pick the furthest that passes
`is_path_clear()` check. Commits through doors (stride 3 on straight paths),
follows corners (falls back to stride 1 at turns).
**NEVER:** Use a fixed stride. It will ALWAYS fail at either doors or corners.

### Bug: Multi-Drone Drones Not Spreading Apart
**File:** `core/stm32n6/search_systematic_mapper.py`, `cell_score()` spatial penalty
**Symptom:** Drones go to same rooms instead of splitting up across building.
**Root cause (original V11.7):** Wall check in spatial penalty skipped the penalty
when drones were in different rooms. Drones in adjacent rooms had zero incentive
to explore different areas. Combined with stale gossip (out of comm range), both
drones targeted the same unexplored regions.
**Root cause (V11.7 wall-check fix):** Wall check was too conservative — ANY wall
cell on the line between drones disabled the penalty, even if drones could reach
each other through doors. Drones still clustered.
**Fix (V11.9):** Removed wall check entirely. Penalty applies within 20m based
purely on Euclidean distance. Strength reduced from 1.3 to 0.8 per meter to
compensate (max +16 at 0m). The richness bonus (-22) ensures wall-adjacent cells
still get explored, just with lower priority than cells far from the other drone.
**NEVER:** Use pre-programmed building knowledge (zones, sectors) for separation.
Drones must decide using ONLY LiDAR-discovered data + shared positions.

### Bug: Global Coverage 12% (should be ~60%)
**File:** `core/drone_manager.py`, `get_global_coverage_stats()`
**Root cause:** Old code read from gossip maps, which depend on inter-drone sync.
When drones are out of comm range, gossip data is stale/incomplete.
**Fix:** Read from `search.searched_cells` and `search.free_cells` directly.
These are always accurate regardless of gossip state.
**NEVER:** Use gossip maps for coverage stats. They depend on radio range.

### Bug: D1 Stuck Returning Home
**File:** `simulation_gui.py`, `_update_multi_drone()` return path
**Root cause:** A* with 5000 max iterations on 0.3m grid can't pathfind 25m
across an entire building. The drone gets a path failure and inches along via
emergency escape at 0.15m/frame.
**Fix:** When A* fails for home, try intermediate waypoints at 8m, 5m, 3m hops
along the direction toward home. A* handles 5-8m paths reliably.
**NEVER:** Assume A* can pathfind across the entire building in one shot.

### Bug: D1 Clock Starts Late (10-15s delay)
**File:** `simulation_gui.py`, `_update_multi_drone()` + entry sequence
**Symptom:** D1 clock shows less time than D0 even though both launched together.
**Root cause:** `mission_start_times[i]` was set when drone enters building (completes
entry sequence), not when it launches. Entry takes 5-15 seconds.
**Fix:** Set ALL `mission_start_times` on the first call to `_update_multi_drone()`.
Removed the overwrite on building entry.
**NEVER:** Set `mission_start_times` on building entry. Battery drains from launch.

### Bug: Drone Ignores Big Unsearched Areas (gossip de-frontiers cells)
**File:** `core/stm32n6/search_systematic_mapper.py`, `_find_best_target()` richness computation
**Symptom:** Yellow frontier markers visible but drone re-covers explored territory.
**Root cause:** Region richness bonus only applied to FRONTIER cells (adjacent to unknown
space). In multi-drone mode, gossip syncs other drones' free_cells, which fills in the
"unknown" neighbors of frontier cells. They lose frontier status AND the richness bonus.
A de-frontiered cell at BFS distance 15 with no bonus (score: 15) loses to a nearby
non-frontier cell at BFS distance 2 (score: 2). The drone picks the nearby explored area.
**Fix:** Compute richness for ALL candidates (not just frontiers). Moved richness bonus
out of the `if c in exploration_cells:` block. Increased cap from -18 to -22 so big
unsearched regions pull the drone even from across the building (max BFS ~24 on 2m grid).
**NEVER:** Limit richness bonus to frontier cells only. Gossip WILL de-frontier cells.

### Bug: Combined Map Missing Frontier Markers
**File:** `simulation_gui.py`, minimap data construction
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
**File:** `simulation_gui.py`, `_nav_escape_if_stuck()`
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
