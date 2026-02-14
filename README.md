# Drone IED Search Simulation

An autonomous drone simulation system for searching unknown buildings to locate IEDs (Improvised Explosive Devices). The simulation models real drone hardware — ST 3D ToF LiDAR, OV2640 camera, STM32WL sub-GHz mesh radio, and 900 MHz long-range control link — so that simulation behavior accurately predicts real-world drone performance.

## Mission

Indoor search & rescue and hazard (IED-like) detection using a cooperative swarm. Each drone maps rooms, identifies people/objects with onboard AI, shares small map updates over mesh, and maintains a long-range pilot link.

## Mission Objectives

1. **Search an UNKNOWN building** - The drone has NO prior knowledge of the building layout
2. **Detect IEDs** - IED sensor has 2-meter range; must pass within 2m to detect
3. **Complete coverage** - Systematically cover all accessible areas
4. **Return safely** - Must return to start position before 7-minute time limit
5. **Don't get stuck** - Navigate around obstacles without getting trapped

## Key Constraints

- **Building is completely unknown** - Only LiDAR and camera data reveal the layout
- **LiDAR-only navigation** - Drones use ONLY sensor data, never pre-programmed building knowledge
- **7-minute time limit** - 6 min forced return, 7 min battery death (per drone)
- **2-meter IED detection range** - Requires close proximity scanning
- **Must return home** - All data is lost if drone doesn't return

## Real Hardware Electronics Stack (150 g class)

### System Architecture

| Processor | Role |
|-----------|------|
| **STM32H7** | Flight control, stabilization, navigation, motor control, failsafe |
| **STM32N6** | Camera + LiDAR processing, AI inference, object detection, semantic mapping |
| **STM32WL** | Sub-GHz mesh radio for building/tunnel drone-to-drone communication |
| **900 MHz long-range radio** | Pilot command, telemetry, emergency control link (RFD900x-class) |

**Data flow:** Camera + LiDAR -> STM32N6 -> detections/map updates -> STM32H7 -> mesh radio (STM32WL). STM32H7 also connects to the 900 MHz control radio for operator link and failsafe.

### Sensors

| Sensor | Part | Notes |
|--------|------|-------|
| **LiDAR** | ST 3D ToF 54x42 | 71 deg diagonal FoV, 9m range, depth grid |
| **Camera** | OV2640 | Low-weight JPEG camera for AI object detection |
| **IED Sensor** | (application-specific) | 2m detection range |

### Radios (two independent links)

| Radio | Part | Purpose |
|-------|------|---------|
| **Mesh radio** | STM32WL sub-GHz module | Drone-to-drone mesh inside buildings/tunnels |
| **Control radio** | 900 MHz long-range (RFD900x-class) | Pilot command, telemetry, emergency failsafe |

### Cost Estimates

| Phase | Per-drone electronics |
|-------|----------------------|
| Prototyping (dev boards) | ~$500-900 total |
| Early builds | ~$200-350 per drone |
| Production (1k+ units) | ~$100-180 per drone |

## Hardware-Matched Simulation

The simulation models the real sensors and radios so behavior translates directly to hardware.

### LiDAR — ST 3D ToF 54x42

| Parameter | Value |
|-----------|-------|
| Sensor | ST 3D ToF 54x42 depth grid |
| Horizontal FoV | 59 deg (derived from 71 deg diagonal, 54:42 aspect) |
| Horizontal resolution | 54 rays (one per column) |
| Max range | 9 meters |
| Scan behavior | Forward-facing cone; drone rotates periodically to build 360 deg awareness |

The simulation casts 54 rays across a 59 deg cone centered on the drone's heading. Every 1.5 simulated seconds the drone takes additional scans at rotated orientations to accumulate full-surround map data from the limited field of view.

### Camera — OV2640

The OV2640 JPEG camera feeds the STM32N6 for onboard AI inference. In simulation this is modelled by the vision system (`core/vision.py`) which detects people, equipment, hazards, and weapons within an 8m / 60 deg FoV cone.

### Mesh Radio — STM32WL Sub-GHz (or XBee 900 MHz)

The drone-to-drone mesh link is modelled after the STM32WL sub-GHz radio module. The simulation code uses the protocol name `xbee_900mhz` since early prototyping may use either an XBee 900 MHz module or the STM32WL — both operate in the same sub-GHz band with similar characteristics.

| Parameter | Simulated Value |
|-----------|----------------|
| Bandwidth | 12.5 KB/s (100 kbps) |
| Latency | 20-80 ms per hop |
| Packet loss | 1% (close range) to 15% (max range), distance-dependent |
| Max fragment size | 200 bytes |
| Priority queue | CRITICAL(1) > HIGH(2) > MEDIUM(3) > LOW(4) |
| Topology | Store-and-forward mesh with AODV-style multi-hop routing |

### Control Radio — 900 MHz Long-Range

The pilot/operator link (RFD900x-class) provides command uplink, telemetry downlink, and emergency failsafe. This link is not yet modelled in the simulation — all operator interaction currently happens through the pygame GUI.

## How It Works

### Search Strategy (V11.9)
1. **LiDAR maps walls** - 54 rays across 59 deg FoV, 9m range; rotation scans every 1.5s
2. **Camera detects objects** - OV2640 feeds STM32N6 AI for person/hazard/IED identification
3. **Divide into 2m grid** - IED detector has 2m range, so 2m grid = full coverage
4. **BFS-distance scoring** - All unsearched cells scored via flood-fill through free space (respects walls)
5. **Region richness** - Cells near large unsearched areas get strong pull bonus (up to -22)
6. **Frontier bonus** - Cells adjacent to unknown space get -8 to -17 bonus (discovers new rooms)
7. **Spatial separation** - Multi-drone: cells near other drones penalized (up to +16 within 20m)
8. **Share via mesh** - Map updates broadcast to swarm over STM32WL gossip protocol
9. **Return before time expires** - 6 min forced return, 7 min battery death per drone

### Search Scoring Formula
```
score = bfs_distance (through free cells, respects walls)
      - min(richness*0.7, 22)  region richness (unsearched cells in 3-cell radius)
      - 8   if frontier cell (adjacent to unknown space)
      - 5   doorway bonus (frontier with both free + unknown neighbors)
      + 2   if wall cell
      + 15  if claimed by another drone
      + (20-d)*0.8  if within 20m of another drone
      - 1.5 sweep continuation bonus
      - 1.0 region completion bonus
      - 0.5*N rush mode (N = unsearched neighbors, after 3 min)
```
Lowest score wins.

### Per-Drone Timers
| Time | Action |
|------|--------|
| 0-3 min | SEARCH mode — explore with frontier + richness scoring |
| 3+ min | RUSH mode — prefer clusters of unsearched cells |
| 6 min | Forced return home (no exceptions) |
| 7 min | Battery dead — drone stops, mission over |

### Single Drone Performance
- Target: **80%+ coverage** in typical buildings
- BFS-distance scoring discovers rooms through doorways
- Region richness pulls drone to big unsearched areas
- Adaptive A* stride navigates doors and corners

### Multi-Drone Swarm Mode (1-12 drones)
- STM32WL / XBee 900 MHz mesh networking with gossip protocol
- Distance-based spatial separation pushes drones apart
- Region richness + gossip sync prevents duplicate coverage
- Global coverage computed from all search algorithms directly
- Per-drone independent timers and screenshots

## Installation

```bash
# Clone the repository
git clone https://github.com/jmrothberg/flight-planning.git
cd flight-planning

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run the simulation
python3 simulation_main.py
```

## Controls

| Key | Action |
|-----|--------|
| **SPACE** | Start/Stop Mission |
| **R** | Reset Simulation |
| **N** | New Object Placement (same building) |
| **B** | New Building Layout |
| **D** | Cycle Drone Count (1-12) |
| **+/-** | Adjust Communication Range |
| **P** | Save Screenshot |
| **ESC** | Exit |

## Project Structure

```
flight_planning/
├── simulation_main.py      # Main entry point - orchestrates all systems
├── algorithm_config.py     # Search algorithm selection
├── requirements.txt        # Python dependencies
├── CLAUDE.md              # AI assistant session notes and bug documentation
│
├── core/                  # Portable to real hardware (maps to STM32N6 + STM32H7)
│   ├── drone.py           # Flight control, waypoint navigation, battery (STM32H7)
│   ├── slam.py            # Occupancy grid mapping, localization (STM32N6)
│   ├── navigation.py      # A* pathfinding, exploration planning (STM32H7)
│   ├── sensors.py         # IED detection with actual IED position (2m range)
│   ├── vision.py          # Object detection via OV2640 + AI (STM32N6)
│   ├── communication.py   # STM32WL/XBee mesh protocol, priority queue, fragmentation
│   ├── minimap.py         # Real-time discovered map tracking
│   ├── search_systematic_mapper.py  # Grid-based coverage algorithm (V11.9)
│   ├── drone_manager.py   # Multi-drone orchestration (1-12 drones)
│   ├── mesh_network.py    # Sub-GHz mesh with distance-based packet loss
│   └── gossip_map.py      # Distributed map sharing via gossip protocol
│
├── simulation/            # Simulation-only (not for real hardware)
│   ├── environment.py     # Building layouts, ST 3D ToF LiDAR simulation
│   ├── graphics.py        # Pygame rendering, minimaps, drone maps, UI panels
│   └── physics.py         # Drone flight dynamics, collisions
│
├── d*_mid_*.png          # Per-drone mid-mission screenshots (at 3 min)
└── d*_done_*.png         # Per-drone completion screenshots
```

## Core Modules

### Search Algorithm (`core/search_systematic_mapper.py`)
The brain of the drone navigation (runs on STM32N6):
- **BFS-distance scoring**: Flood-fill through free space ensures targets behind walls are unreachable
- **Region richness**: Counts unsearched cells in 3-cell radius; big unsearched areas get up to -22 bonus
- **Frontier exploration**: Cells adjacent to unknown space get -8 bonus; doorways get -5 extra
- **Adaptive A* stride**: Tries stride 3, 2, 1 with `is_path_clear` check — commits through doors, follows corners
- **Navigation escape**: Detects physical stuck (wall collision) and forces escape move
- **IED detection**: Cardinal neighbors only, wall-checked both sides, no through-wall marking
- **Multi-drone**: gossip-shared global_searched prevents duplication; spatial penalty pushes apart

### LiDAR Sensor Simulation (`simulation/environment.py`)
Models the ST 3D ToF 54x42 LiDAR:
- 54 horizontal rays across 59 deg FoV centered on drone heading
- 9m max range
- Returns structured dict: `{ranges, start_angle, angle_step, hfov, num_rays}`

### Multi-Drone Coordination (`core/drone_manager.py`, `core/gossip_map.py`)
For 1-12 drone swarm missions:
- Each drone maintains its own search algorithm + gossip map
- **Gossip data flow** (every frame):
  1. `sync_local_to_gossip(i)` — each drone's local free/searched/wall cells pushed to its gossip map
  2. `broadcast_map_updates()` — nearby drones (within comm_range) exchange gossip payloads
  3. `update_search_from_gossip(i)` — merged gossip data fed back to search algorithm
- Global coverage computed from search algorithms directly (not gossip — always accurate)
- Spatial separation penalty: `(20-d)*0.8` within 20m, even through walls
- Region richness applied to ALL candidates (not just frontiers) — survives gossip de-frontiering

## Output Files

### Screenshots
- `d{id}_mid_{time}_{cov}pct.png` — Each drone at 3 minutes (half of 6-min return)
- `d{id}_done_{time}_{cov}pct.png` — Each drone at completion or battery death
- Single drone = D0, same naming convention

### Videos
- `sim_{n}d_{cov}pct_{time}.mp4` — Full mission recording, start to last drone finish
- Tiny videos (<20 frames) are automatically discarded

## Algorithm Versions

| Version | Description |
|---------|-------------|
| V11.5h | Conservative return timing |
| V11.6a | Fixed entry oscillation, optimized timing |
| V11.7 | Bonus-based frontier exploration, multi-drone gossip |
| V11.8 | Wall-aware penalties, per-drone timers, door-stuck fixes, nav escape |
| **V11.9** | **Region richness for all candidates, distance-based spatial separation, IED position fix, simplified screenshots** |

## Future Roadmap

- **3D Navigation** — Multiple floors, stairs, vertical connections (grid → voxels)
- **Hallways & Tunnels** — Long narrow spaces with limited LiDAR visibility
- **N-Drone Scaling** — All algorithms designed for 1-12+ drones
- **900 MHz Control Link** — Pilot telemetry, failsafe simulation
- **Real Hardware Integration** — STM32H7 + STM32N6 + STM32WL deployment

## Hardware Integration

The `core/` modules are designed for direct portability to real hardware:

| Simulation Module | Target Hardware | Integration Notes |
|-------------------|-----------------|-------------------|
| `environment.get_lidar_scan()` | ST 3D ToF 54x42 via STM32N6 | Same dict format `{ranges, start_angle, ...}` |
| `vision.py` | OV2640 camera via STM32N6 AI | Replace with STM32N6 inference output |
| `communication.py` / `mesh_network.py` | STM32WL sub-GHz module | Implement `MeshProtocolInterface` for STM32WL SPI/UART |
| `drone.py` / `navigation.py` | STM32H7 flight controller | Replace `physics.py` with real motor/IMU API |
| `sensors.py` | IED sensor hardware | Replace with actual sensor interface |

## License

MIT License - See LICENSE file for details.
