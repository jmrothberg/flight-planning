# Drone IED Search System (LEGACY — V11.6a)

> **NOTE:** This document is from V11.6a and is OUTDATED. See **README.md** for current
> documentation and **CLAUDE.md** for technical details. Key differences since this was written:
> - Grid changed from 3m to 2m (matching IED sensor range)
> - LiDAR changed from 360-ray/25m to 54-ray/59° FoV/9m (realistic ST ToF sensor)
> - Entry point is now `simulation_gui.py` (not `simulation_main.py`)
> - Drones scale to 12 (not 4)
> - 3-processor architecture (STM32H7/N6/WL) with realistic bus communication
> - Pose estimation pipeline with IMU + optical flow dead reckoning
> - Base station with radio-only discovered map
> - Per-drone missions (Map/Destroy), manual control, radio link modes
>
> This file is kept for historical debugging context and lessons learned.

## Mission
Search an UNKNOWN building for IEDs within 7 minutes. Return before battery dies.

## How It Works

### Search Strategy (3m Grid Coverage)
1. **LIDAR maps walls** → identifies rooms and free space
2. **Divide into 3m grid** → IED detector has 3m range, so 3m grid = full coverage
3. **Visit each grid cell** → nearest unsearched cell first
4. **Track coverage** → searched cells / total free cells
5. **Return before 7 min** → battery limit

### Key Components
- **LIDAR**: 360 rays, sees walls up to 25m
- **IED Detector**: 3m range, must pass within 3m of IED to detect
- **Grid Search**: 3m spacing ensures every point is within 3m of a visited cell

## Quick Start

```bash
cd flight_planning
source .venv/bin/activate
pip install -r requirements.txt
python3 simulation_main.py
```

## Controls
- **SPACE**: Start/stop mission
- **R**: Reset
- **B**: New building layout
- **D**: Cycle drone count (1-4) for multi-drone mode
- **+/=**: Increase communication range
- **-**: Decrease communication range
- **P**: Screenshot
- **ESC**: Exit

## Multi-Drone Mode (V11.1)

Press **D** to cycle through drone counts (1→2→3→4→1). In multi-drone mode:

### Features
- **Range-limited mesh communication**: Drones only talk when within comm range
- **Multi-hop routing**: Messages route through nearest neighbors (D1→D2→D3)
- **Gossip-based map sharing**: Drones exchange discoveries when in range
- **Color-coded visualization**: Each drone's coverage shown in its color
- **Coordinated search**: Drones share frontiers and avoid duplicating coverage
- **Per-drone maps**: Individual map panels show each drone's knowledge (merged after communication)

### How Knowledge Sharing Works
**Important**: Drones only learn what they discover themselves OR receive via radio:
1. Each drone explores and builds its own map (free cells, searched cells, walls)
2. When two drones are in comm range, they exchange map data via gossip protocol
3. After communication, drone's `global_free` and `global_searched` include shared data
4. No "magic" sharing - if drones never communicate, they have independent maps

### Coordinated Frontier Selection
- Drones share their **frontiers** (unsearched free cells = yellow markers)
- When selecting a target, each drone considers ALL known frontiers (own + shared)
- Picks the **nearest frontier**, whether discovered by self or received from others
- Different frontier shapes per drone: circles (D0), triangles (D1), squares (D2), diamonds (D3)

### Visualization
- **Right column maps**: Per-drone individual maps (show merged knowledge after sync)
- **Bottom minimap**: Combined/composite map from all drones
- **Green lines**: Active mesh links between drones
- **Colored circles**: Communication range per drone
- **Status panel**: Per-drone status (ENTRY/SEARCH/RETURN/COMPLETE)

### Mesh Networking
```
Example: 3 drones searching building

    [D0-Red] ----10m---- [D1-Blue] ----10m---- [D2-Green]
              in range              in range

D0 shares map with D1, D1 shares with D2
D2 receives D0's map via D1 (multi-hop)

After sync: D1's map includes D0's and D2's discoveries
```

### Stuck Detection & Recovery
- Drones track position history to detect if stuck (not moving for 3+ seconds)
- Stuck targets are blacklisted for 30 seconds
- Fallback: explore toward most open LIDAR direction if no valid frontiers

### Hardware Integration
To use real radio hardware:
1. Implement `MeshProtocolInterface` in `core/mesh_network.py`
2. Replace `SimulatedMeshProtocol` with your hardware class
3. All routing/gossip logic remains unchanged

### Known Limitations & Potential Improvements
- **Coordination**: Drones may still overlap if they start exploring same area before communicating
- **Target claiming**: Basic claiming exists but could be improved with lease-based coordination
- **Path through walls**: Occasional pathfinding issues when LIDAR has gaps - needs investigation
- **Coverage**: Multi-drone doesn't yet achieve same 100% coverage as single drone consistently

### Planned Multi-Drone Coordination (V11.7+)
Based on research into auction-based task allocation and boustrophedon coverage:

1. **Initial Spatial Assignment**: 
   - When drones enter, divide building into quadrants based on discovered area
   - Each drone assigned primary responsibility for a quadrant
   - Prevents early overlap before communication is established

2. **Auction-Based Reallocation**:
   - When drones communicate, they can bid on frontier regions
   - Drone closest to a frontier "wins" it
   - Prevents two drones racing to the same area

3. **Coordinated Sweep Patterns**:
   - Within their assigned region, drones use boustrophedon sweep
   - Adjacent drones coordinate sweep direction to avoid collision
   - E.g., Drone 0 sweeps left-to-right, Drone 1 sweeps right-to-left

4. **Coverage Guarantee**:
   - Track global coverage via gossip protocol
   - If any region falls behind, reassign idle drones
   - Ensures all areas get searched even if one drone gets stuck

## Module Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          simulation_main.py                             │
│                    (Main loop, orchestrates all systems)                │
└──────────────┬──────────────────────────────────────────────────────────┘
               │
       ┌───────┴───────┐
       ▼               ▼
┌─────────────┐  ┌─────────────────────────────────────────────────────────┐
│ simulation/ │  │                        core/                            │
│ (sim-only)  │  │               (portable to real hardware)               │
└─────────────┘  └─────────────────────────────────────────────────────────┘

SIMULATION LAYER (replace for real hardware):
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  environment.py  │  │   graphics.py    │  │    physics.py    │
│  - Fake building │  │  - pygame GUI    │  │  - Flight model  │
│  - LIDAR sim     │  │  - Minimap       │  │  - Collisions    │
│  - Camera sim    │  │  - UI panels     │  │  - Wind sim      │
└──────────────────┘  └──────────────────┘  └──────────────────┘

CORE LAYER (keep for real hardware):
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│     drone.py     │  │     slam.py      │  │   navigation.py  │
│  - Flight ctrl   │  │  - Grid mapping  │  │  - A* pathfinder │
│  - Battery mgmt  │  │  - Localization  │  │  - RRT explorer  │
│  - Waypoints     │──│  - Entry point   │──│  - Door detect   │
└──────────────────┘  └──────────────────┘  └──────────────────┘
         │                     │                     │
         ▼                     ▼                     ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│    sensors.py    │  │    minimap.py    │  │ communication.py │
│  - IED detector  │  │  - Discovered    │  │  - Message queue │
│  - 3m range      │  │    features      │  │  - Priority TX   │
│  - Components    │  │  - Wall segments │  │  - Compression   │
└──────────────────┘  └──────────────────┘  └──────────────────┘
                               │
                               ▼
┌──────────────────┐  ┌──────────────────┐
│    vision.py     │  │search_systematic │
│  - Object detect │  │   _mapper.py     │
│  - YOLO-ready    │  │  - Grid coverage │
│  - Cooldowns     │  │  - Rush mode     │
└──────────────────┘  └──────────────────┘
```

### Data Flow
```
LIDAR → slam.py → navigation.py → drone.py → physics.py
          ↓              ↓
     minimap.py    search_systematic_mapper.py
          ↓              ↓
     graphics.py ← ← ← ←┘
          ↓
    communication.py → (radio TX)
```

## Module Reference

### Core Modules (portable to real hardware)

| Module | Purpose | Key Classes/Functions |
|--------|---------|----------------------|
| `drone.py` | Flight control, waypoint navigation, battery | `Drone`, `FlightController`, `PIDController` |
| `slam.py` | Occupancy grid mapping, particle filter localization | `SLAMSystem`, `GridMap`, `ParticleFilter` |
| `navigation.py` | A* pathfinding, RRT exploration, door detection | `AStarPathfinder`, `RRTExplorer`, `PathPlanner` |
| `sensors.py` | IED detection with 3m range, component analysis | `IEDSensor`, `IEDReading` |
| `vision.py` | Object detection (person, hazard, weapon, furniture) | `VisionSystem`, `SimpleObjectDetector`, `Detection` |
| `communication.py` | Priority message queue, data compression, TX | `CommSystem`, `Message`, `MessagePriority` |
| `minimap.py` | Real-time discovered map, wall/door/object tracking | `MinimapSystem`, `DiscoveredFeature` |
| `search_systematic_mapper.py` | Grid-based 100% coverage algorithm (V11.1) | `SystematicMapper` |
| `drone_manager.py` | Multi-drone management with mesh networking | `DroneManager` |
| `mesh_network.py` | Range-limited mesh with AODV routing | `MeshNode`, `MeshProtocolInterface`, `SimulatedMeshProtocol` |
| `gossip_map.py` | Distributed map sharing via gossip protocol | `GossipMap`, `MapFeature` |

### Simulation Modules (not for hardware)

| Module | Purpose | Key Classes/Functions |
|--------|---------|----------------------|
| `environment.py` | Fake building layouts, LIDAR/camera simulation | `Environment`, `get_lidar_scan()` |
| `graphics.py` | pygame rendering, UI panels, minimap display | `GraphicsEngine`, `Colors` |
| `physics.py` | Drone flight dynamics, RK4 integration, collisions | `PhysicsEngine`, `DronePhysics` |

### Entry Point

| File | Purpose |
|------|---------|
| `simulation_main.py` | Main loop, system orchestration, event handling |
| `algorithm_config.py` | Search algorithm selection |

## Hardware Integration

### LIDAR
Replace `environment.py:get_lidar_scan()` with your LIDAR driver.
Must return `List[float]` with 360 distances in meters.

### IED Sensor
Replace `sensors.py:read()` with your sensor hardware.
Return `IEDReading` with component percentages.

## Algorithm Details

### Grid Search (search_systematic_mapper.py V11)

```
LIDAR sees:        Grid cells:        Visit order:
+--------+         +--+--+--+         [1][4][7]
|        |   ->    |  |  |  |    ->   [2][5][8]
|  room  |         +--+--+--+         [3][6][9]
+--------+         |  |  |  |         
                   +--+--+--+         (nearest first)
```

Each 3m cell visited = that area searched for IEDs.
Coverage % = searched cells / discovered free cells (interior only, y >= 0).

### Why 3m Grid?
- IED detector range = 3m
- If drone visits center of each 3m cell, every point is within 3m
- This guarantees 100% coverage if all cells are visited

### Discovered Map (bottom-left panel)
- **Light green squares**: 3m grid cells that have been searched (IED-scanned)
- **Yellow dots**: Frontier cells (unknown areas beyond - potential unexplored rooms)
- **Blue dots + labels**: Detected objects (person, weapon, furniture, etc.)
- **Red dots + "IED!"**: Detected IEDs
- **Black lines**: Discovered walls

### Coverage Stats (bottom-right panel)
- **Coverage %**: (searched interior cells) / (discovered interior free cells) × 100
- **Uncovered**: Free cells not yet visited (should be 0 at 100% coverage)
- **Found**: List of object types detected

## Mission Flow
1. Drone starts outside at entry point
2. Enters through door
3. LIDAR maps walls → creates 3m grid
4. Visits grid cells (nearest unsearched first, rush mode after 200s)
5. IED/object sensors report findings
6. Map auto-saves when returning home
7. Returns to entrance before 7 minutes

## Version History

### V11.6a - Fixed Entry Oscillation Bug (CURRENT)
**CRITICAL: Drone MUST return home or all data is lost.**

**The bugs (V11.6):**
1. entry_point was set inside building (y≈2), but mission_complete requires y<0.5
   - Drone would oscillate at entry forever, wasting 30+ seconds
2. Return timing formula scaled badly: `distance * 2` term dominated at far distances

**The fix (V11.6a):**
- **New exit_point**: Set to START position (y=-3) - drone returns to where it began
- **Simple timing formula**: `time_to_return = distance / 1.2 + 70`
  - At 10m: 78s, At 20m: 87s, At 30m: 95s (reasonable scaling)
- **Sweep preference**: Prefers continuing in same direction (boustrophedon-style)
- **Region completion**: Bonus for cells adjacent to recently-searched cells

**Key files:**
- `core/search_systematic_mapper.py` - Search algorithm with return logic
- `simulation_main.py` - Passes simulated time via `set_simulated_time()`

### V11.6 - Optimized Coverage with Sweep Patterns (BUGGY)
**BUG**: Drone oscillated at entrance because entry_point (y≈2) != mission_complete (y<0.5)
**BUG**: Return timing scaled badly at far distances (76% coverage, worse than V11.5h)

### V11.5h - Conservative Return Timing (PREVIOUS)
**CRITICAL: Drone MUST return home or all data is lost.**

**The bug (V11.5g):** Used 1.5 m/s speed + 60s margin - drone didn't make it home!

**The fix (V11.5h):**
- Return speed: 1.0 m/s (drone slows near obstacles)
- Safety margin: 90 seconds
- Formula: `time_to_return = distance / 1.0 + 90`

**Status**: Too conservative - only achieved 81% coverage

### FAILED APPROACHES (DO NOT REPEAT)

| Version | Approach | Result | Why It Failed |
|---------|----------|--------|---------------|
| V11.3 | Angular partitioning (90° sectors) | Oscillation | Angles are RELATIVE - change as drone moves |
| V11.4 | Euclidean world coords | 40% coverage | Broke pathfinding interaction |
| V11.5 | Strong position repulsion | 61% coverage | Dynamic penalties cause constant switching |
| V11.5c | Aggressive timeouts (0.5s, 8s) | 75% coverage | Gave up on reachable targets too fast |
| V11.5f | time.time() for return | Never returned | Wrong time base (real vs simulated) |
| V11.5g | 1.5 m/s + 60s margin | Didn't return | Not conservative enough |

### V11.1 - Multi-Drone Support (BROKEN - foundation only)
**New modules created:**
- `core/mesh_network.py` - Range-limited mesh with MeshNode, MeshProtocolInterface, SimulatedMeshProtocol
- `core/gossip_map.py` - Distributed map sharing via gossip protocol
- `core/drone_manager.py` - Multi-drone orchestration

**Key features implemented:**
- Add/remove drones at runtime (D key cycles 1-4)
- Adjustable comm range (+/- keys)
- Per-drone individual map display
- Gossip-based frontier sharing (no magic - only via radio)
- Coordinated target selection (nearest frontier wins)
- Stuck detection with target blacklisting
- Preserves drone state when adding new drones

**Files modified:**
- `simulation_main.py` - Multi-drone loop, render, setup
- `core/search_systematic_mapper.py` - Added global_free/global_searched, coordinated selection
- `simulation/graphics.py` - Per-drone maps, frontier shapes

### V11.0 - Single Drone 100% Coverage
- Grid-based systematic coverage achieving 100%
- A* pathfinding
- Rush mode for time-critical completion

## Development Notes for Future LLM Sessions

### Current Status (V11.6a)
- Single drone targeting 90%+ coverage
- V11.6a fixes: entry oscillation bug, simpler timing formula
- V11.6 features: sweep patterns, region completion
- Drone returns to START position (y=-3), not entry point (inside at y≈2)
- Multi-drone coordination has foundation but not fully working

### Key Files to Understand
1. `core/search_systematic_mapper.py` - Main search algorithm
   - `get_next_waypoint()` - Returns next target, handles return logic
   - `frontier_score()` - Scores unsearched cells by distance
   - `set_simulated_time()` - Receives simulated elapsed time from simulation
2. `simulation_main.py` - Main loop, calls search algorithm
   - Passes simulated time via `set_simulated_time()`
   - SIM_SPEED = 3.0 (simulation runs 3x real time)

### Areas Needing Improvement
1. **Coverage not reaching 100%** - Some rooms/corners missed
2. **Return timing** - Currently very conservative (1.0 m/s + 90s margin)
3. **Multi-drone** - Foundation exists but oscillation issues with 2+ drones

### Architecture Decisions
- Transport layer (MeshProtocolInterface) separate from routing (MeshNode) - allows hardware swap
- Gossip uses version vectors for efficient sync
- Each drone maintains own SearchMapper instance with optional global knowledge
- Frontiers = free_cells - searched_cells (unsearched edges of explored area)

---

## CRITICAL DEBUGGING NOTES FOR NEXT LLM

**READ THIS BEFORE MAKING ANY CHANGES**

### BUG 1: Maps Not Syncing When Communicating (UNSOLVED)
**Symptom**: When Mesh Links > 0 (drones in comm range with green lines visible), the per-drone maps (D0, D1, D2) show DIFFERENT content. They should show IDENTICAL walls, searched cells, etc after sync.

**What was tried (FAILED)**:
- Sped up gossip_interval from 0.3s to 0.1s - WRONG APPROACH, timing is not the issue
- Added debug print in merge_remote_update - helpful for diagnosis but didn't fix

**Where to look**:
- `core/gossip_map.py`: `get_sync_payload()` only sends THIS drone's data, `merge_remote_update()` merges remote data
- `core/drone_manager.py`: `broadcast_map_updates()` sends, `process_gossip()` receives
- `simulation_main.py` lines 830-854: Display code merges local + gossip data

**Likely root causes to investigate**:
1. The mesh MESSAGE RECEIVING may be broken - check `SimulatedMeshProtocol.receive()` in `core/mesh_network.py`
2. Version check in `merge_remote_update` may be rejecting valid updates (line ~180 gossip_map.py)
3. The order of operations: `update_mesh()` → `process_gossip()` → `broadcast_map_updates()` - broadcast happens AFTER receive, so updates propagate next frame. But if receive is broken, nothing works.

**How to verify gossip is working**:
- Console should print `[GOSSIP] D0 merged from D1: +X searched, +Y walls` when drones communicate
- If NO gossip messages appear when Mesh Links > 0, the mesh receive pipeline is broken

### BUG 2: Drones Going Through Walls (PARTIALLY FIXED, STILL OCCURS)
**Symptom**: Drones visually move through walls, or cells are marked searched on wrong side of wall.

**What was tried**:
- Made fallback navigation stricter (wider cone ±30°, require clearance, smaller step)
- But A* pathfinder may be returning paths through walls if SLAM occupancy grid is wrong

**Where to look**:
- `core/search_systematic_mapper.py`: `_update_map()` does ray tracing to mark walls
- `core/navigation.py`: A* uses occupancy grid - if walls aren't in grid, A* routes through
- `simulation_main.py`: Fallback navigation when A* fails (search for "LIDAR-based fallback")

**Likely root causes**:
1. SLAM occupancy grid `slam.get_occupancy_map()` may not match search algorithm's wall_cells
2. A* grid resolution vs SLAM resolution mismatch
3. Diagonal neighbor marking in `_update_map()` may be too aggressive

### BUG 3: Drone Oscillation / Getting Stuck
**What was implemented** (partially working):
- Position history for stuck detection (3s threshold)
- Failed target blacklist (30s timeout)
- Target claiming with timeout
- Recent targets penalty for oscillation prevention

**Still happening when**:
- Multiple drones compete for same frontier
- Pathfinding fails repeatedly to same target
- Communication causes target switching

### MISTAKES TO AVOID
1. **DON'T speed up timers** - gossip_interval, hello_interval are NOT the issue
2. **DON'T add more debug spam** - terminal gets unreadable, focus on specific issues
3. **DON'T refactor working code** - single drone works fine, problem is multi-drone integration
4. **DON'T guess** - trace the actual data flow with targeted prints

### RECOMMENDED DEBUGGING APPROACH
1. Add a SINGLE focused print in `SimulatedMeshProtocol.receive()` to verify messages are being received
2. Print what `get_sync_payload()` returns before broadcast
3. Print what `merge_remote_update()` receives and whether it returns True
4. Compare drone gossip_map.global_walls sets directly after sync attempt

### KEY FILES TO UNDERSTAND
| File | What it does | Critical functions |
|------|--------------|-------------------|
| `core/mesh_network.py` | Message transport | `SimulatedMeshProtocol.receive()`, `MeshNode._process_incoming()` |
| `core/gossip_map.py` | Map merging | `get_sync_payload()`, `merge_remote_update()` |
| `core/drone_manager.py` | Orchestration | `update_mesh()`, `process_gossip()`, `broadcast_map_updates()` |
| `simulation_main.py` | Main loop | `_update_multi_drone()`, lines 830-854 (display merge) |

### DATA FLOW FOR MAP SYNC
```
1. Drone explores → search.wall_cells updated
2. sync_local_to_gossip() → gossip.add_local_walls()
3. broadcast_map_updates() → mesh.broadcast("MAP_UPDATE", payload)
4. [NEXT FRAME] update_mesh() → mesh.update() → protocol.receive()
5. process_gossip() → gossip.merge_remote_update()
6. [DISPLAY] merged_walls = search.wall_cells + gossip.get_global_walls()
```

If maps aren't syncing, one of these steps is failing. Find which one.

---

## CRITICAL: MULTI-DRONE SEARCH ALGORITHM ISSUES (Jan 2026)

**THE FUNDAMENTAL PROBLEM**: Single drone works perfectly. Multi-drone fails to explore efficiently.

### What Single Drone Does (WORKS)
1. LIDAR scans → adds cells to `free_cells` (passable) and `wall_cells`
2. Current position → added to `searched_cells`
3. Frontier = `free_cells - searched_cells` (unsearched edges)
4. Target = nearest frontier
5. STICK with target until reached, then pick next nearest
6. Achieves 100% coverage reliably

### What Multi-Drone SHOULD Do (BROKEN)
Each drone should act EXACTLY like single drone, but:
1. Use `global_searched` (from gossip) instead of just own `searched_cells`
2. Use `global_free` (from gossip) to see frontiers discovered by other drones
3. Avoid targeting EXACT cell another drone is already targeting

### V11.2 FAILED ATTEMPT - DO NOT USE
**Date**: 2026-01-29
**What was tried**: 
- Added "target yielding" - if another drone claimed same target, abandon it
- Added check `target_claimed_by_other = self.target_cell in self.claimed_by_others`
- Skip claimed cells with +1000 penalty

**Result**: TOTAL FAILURE - drones oscillate after going a few meters in. Worse than V11.1.

**Why it failed**: Unknown. The logic seemed correct but drones still bounce back and forth.

**Key insight missed**: Single drone works perfectly. Multi-drone coordination attempts keep breaking the working single-drone behavior. The problem is NOT in target selection but somewhere else (possibly in how claims are shared, or timing issues).

**RECOMMENDATION**: Revert to V11.1 or earlier, do NOT trust any "coordination" fixes without understanding WHY single drone works and multi-drone fails.

**V11.3 SOLUTION**: ANGULAR PARTITIONING - Each drone prefers frontiers in different 90° angular sectors from their current position. This ensures drones explore in different directions from the start, preventing clustering and oscillation. Simple, effective, and maintains single-drone behavior.

### CRITICAL BUGS ENCOUNTERED

#### BUG: TARGET OSCILLATION (FIXED in V11.3)
**Symptom**: Drones rapidly switch between targets, never making progress
**Root Cause**: Code that switches targets based on recent history
**WRONG**:
```python
if best in self.recent_targets[-3:]:
    best = all_frontiers[1]  # CAUSES OSCILLATION!
```
**FIX**: Use TARGET PERSISTENCE - once a target is chosen, STICK WITH IT until:
- Target reached (within 2m)
- Target searched by another drone (in global_searched)
- Stuck for 3+ seconds (then blacklist)

**NEVER REMOVE THIS CODE** (in search_systematic_mapper.py):
```python
# KEEP current target if still valid
elif not target_reached and not target_searched and not target_failed:
    return self.target  # STICK WITH CURRENT TARGET
```

#### BUG: GOSSIP SYNC TIMING
**Symptom**: Drones have stale data when picking targets
**Root Cause**: `update_search_from_gossip()` called BEFORE `broadcast_map_updates()`
**FIX**: Correct order in `_update_multi_drone()`:
1. Phase 1: ALL drones `sync_local_to_gossip()` (push local data)
2. Phase 2: `broadcast_map_updates()` (sync between drones)
3. Phase 3: Each drone `update_search_from_gossip()` then navigate

#### BUG: HEAVY PENALTIES BLOCK EXPLORATION
**Symptom**: Drones avoid good frontiers, get stuck in small areas
**Root Cause**: Penalties for "nearby" claimed cells too heavy
**WRONG**:
```python
if cdist <= 2: claimed_penalty += 100  # TOO HEAVY!
```
**FIX**: Only penalize EXACT claimed cell, not nearby:
```python
if c in self.claimed_by_others:
    return dist + 10  # Small penalty, not blocking
return dist  # Just use distance
```

### WHAT STILL NEEDS FIXING

1. **Drones not using `global_free` effectively** - They have shared frontier data but still don't explore to distant areas. Need to verify `global_free` is being populated and used.

2. **Blacklisting too aggressive** - Drones may be blacklisting good targets because pathfinding temporarily fails. Consider: shorter blacklist time, or only blacklist if truly unreachable.

3. **Entry handling** - Later drones sometimes get stuck at entry or overlap with existing drones.

### LESSONS LEARNED (DO NOT REPEAT)

| Mistake | Why It Fails | What To Do Instead |
|---------|--------------|-------------------|
| Switching targets every frame | Oscillation, no progress | Target persistence |
| Heavy penalties for "nearby" cells | Blocks good frontiers | Only penalize exact conflicts |
| Complex direction preferences | Restricts exploration | Just use distance |
| Syncing gossip AFTER picking targets | Stale data | Sync BEFORE target selection |
| Adding complexity | More bugs | Keep it simple like single drone |
| V11.3 Angular partitioning (90° sectors) | Angles are RELATIVE - as drone moves, "preferred" direction changes | Don't use angles from current position |
| V11.4/V11.5 Euclidean world coords | BROKE SINGLE DRONE - coverage dropped from 100% to 61% | MUST use Manhattan distance in GRID coords |
| **CRITICAL: Changing distance calculation** | Single drone pathfinding depends on Manhattan grid distance | NEVER change: `abs(c[0]-curr[0]) + abs(c[1]-curr[1])` |

### THE CORE PROBLEM (UNSOLVED AS OF V11.5)

**Single drone achieves 100% coverage. Multi-drone achieves ~40%.**

The fundamental issue: coordination approaches keep interfering with the simple "go to nearest unsearched" behavior that makes single drone work. Every "smart" coordination attempt causes oscillation or clustering.

**What might actually work (not yet tried):**
1. Just use global_searched and global_free, NO spatial separation at all
2. Let drones occasionally overlap - gossip will sync and they'll naturally spread
3. The claim system with short timeout (5s) should prevent long-term conflicts
4. If this still fails, the gossip sync itself is broken

### THE CORRECT APPROACH (NOT YET FULLY WORKING)

```python
# Each drone does this:
1. Get global_searched from gossip (what ANY drone has searched)
2. Get global_free from gossip (frontiers ANY drone has discovered)
3. Frontiers = (free_cells + global_free) - global_searched
4. Score each frontier by distance only (+10 if exact cell claimed by other)
5. Pick nearest
6. STICK with target until reached/searched/failed
7. Repeat
```

This is literally just single-drone behavior with shared knowledge. If this doesn't work, the gossip sync is broken.

### HOW TO DEBUG
1. Print `len(global_searched)` and `len(global_free)` in `_find_best_target()`
2. If `global_free` is empty when drones are in range, gossip isn't working
3. If `global_free` has data but drones don't use it, frontier filtering is wrong
4. If drones go to frontiers but get stuck, pathfinding or stuck detection is wrong
