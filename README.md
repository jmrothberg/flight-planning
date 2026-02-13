# Drone IED Search Simulation

An autonomous drone simulation system for searching unknown buildings to locate IEDs (Improvised Explosive Devices). The simulation models real drone hardware — STM 54x42 LiDAR and 900 MHz XBee mesh radio — so that simulation behavior accurately predicts real-world drone performance.

![Simulation Screenshot](r_s_m_v11.6a_02m20s_cov93pct_final_7min_18098.png)

## Mission Objectives

1. **Search an UNKNOWN building** - The drone has NO prior knowledge of the building layout
2. **Detect IEDs** - IED sensor has 3-meter range; must pass within 3m to detect
3. **Complete coverage** - Systematically cover all accessible areas
4. **Return safely** - Must return to start position before 7-minute time limit
5. **Don't get stuck** - Navigate around obstacles without getting trapped

## Key Constraints

- **Building is completely unknown** - Only LiDAR sensor data reveals the layout
- **7-minute time limit** - Battery/mission constraint
- **3-meter IED detection range** - Requires close proximity scanning
- **Must return home** - All data is lost if drone doesn't return

## Hardware-Matched Sensors

### LiDAR — STM 54x42 (71 deg diagonal FoV, 9m range)

| Parameter | Value |
|-----------|-------|
| Sensor | STM 54x42 depth grid |
| Horizontal FoV | 59 deg (derived from 71 deg diagonal, 54:42 aspect) |
| Horizontal resolution | 54 rays (one per column) |
| Max range | 9 meters |
| Scan behavior | Forward-facing cone; drone rotates periodically to build 360 deg awareness |

The simulation casts 54 rays across a 59 deg cone centered on the drone's heading. Every 5 simulated seconds the drone takes additional scans at rotated orientations to accumulate full-surround map data from the limited field of view.

### Radio — 900 MHz XBee Mesh

| Parameter | Value |
|-----------|-------|
| Protocol | XBee 900 MHz mesh (store-and-forward) |
| Bandwidth | 12.5 KB/s (100 kbps) |
| Latency | 20-80 ms per hop |
| Packet loss | 1% (close range) to 15% (max range), distance-dependent |
| Max fragment size | 200 bytes |
| Priority queue | CRITICAL(1) > HIGH(2) > MEDIUM(3) > LOW(4) |

Messages exceeding 200 bytes must be pre-fragmented. The `CommProtocol.fragment_message()` method handles this. Bandwidth is tracked per rolling 1-second window.

### Data Types over Radio

| Data Type | Priority | Typical Size |
|-----------|----------|--------------|
| Mission commands / emergency | CRITICAL | ~50-100 bytes |
| Hazard / IED detections | HIGH | ~100-500 bytes |
| Map updates (incremental) | MEDIUM | ~100-800 bytes |
| LiDAR keyframes (54x42 grid) | LOW | ~1-5 KB compressed |
| Drone pose | LOW | ~50-80 bytes |

## How It Works

### Search Strategy (3m Grid Coverage)
1. **LiDAR maps walls** - 54 rays across 59 deg FoV, 9m range; periodic rotation for 360 deg coverage
2. **Divide into 3m grid** - IED detector has 3m range, so 3m grid = full coverage
3. **Visit each grid cell** - Nearest unsearched cell first, with sweep optimisation
4. **Track coverage** - Searched cells / total discovered free cells
5. **Return before time expires** - Distance-based timing with safety margin

### Single Drone Performance
- Achieves **90-95% coverage** in typical buildings
- Uses boustrophedon-style sweep patterns for efficiency
- Completes rooms before moving to new areas

### Multi-Drone Mode
- **1-12 drones** working together (press D to cycle)
- XBee 900 MHz mesh networking with gossip protocol for map sharing
- Distance-dependent packet loss and per-hop latency
- Coordinated frontier selection to avoid overlap
- Range-limited communication (adjustable with +/- keys)

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
├── README.md              # This file
├── Drone_sim_README.md    # Detailed technical documentation
│
├── core/                  # Portable to real hardware
│   ├── drone.py           # Flight control, waypoint navigation, battery
│   ├── slam.py            # Occupancy grid mapping, localization
│   ├── navigation.py      # A* pathfinding, exploration planning
│   ├── sensors.py         # IED detection (3m range)
│   ├── vision.py          # Object detection (person, hazard, weapon)
│   ├── communication.py   # XBee 900MHz protocol, priority queue, fragmentation
│   ├── minimap.py         # Real-time discovered map tracking
│   ├── search_systematic_mapper.py  # Grid-based coverage algorithm (V11.6a)
│   ├── drone_manager.py   # Multi-drone orchestration (1-12 drones)
│   ├── mesh_network.py    # XBee mesh with distance-based packet loss
│   └── gossip_map.py      # Distributed map sharing via gossip protocol
│
├── simulation/            # Simulation-only (not for real hardware)
│   ├── environment.py     # Building layouts, STM 54x42 LiDAR simulation
│   ├── graphics.py        # Pygame rendering, FoV cone visualisation
│   └── physics.py         # Drone flight dynamics, collisions
│
└── r_s_m_*.png           # Screenshot results showing coverage achieved
```

## Core Modules

### Search Algorithm (`core/search_systematic_mapper.py`)
The brain of the drone navigation:
- **Grid-based coverage**: 3m cells match IED detector range
- **Frontier selection**: Picks nearest unsearched cell
- **Sweep preference**: Continues in same direction to reduce backtracking
- **Region completion**: Finishes rooms before moving on
- **Return timing**: Distance-based formula ensures safe return
- **Limited FoV aware**: Handles both dict (new STM format) and list (legacy) LiDAR data

### LiDAR Sensor Simulation (`simulation/environment.py`)
Models the STM 54x42 LiDAR:
- 54 horizontal rays across 59 deg FoV centred on drone heading
- 9m max range
- Returns structured dict: `{ranges, start_angle, angle_step, hfov, num_rays}`

### SLAM System (`core/slam.py`)
Simultaneous Localization and Mapping:
- Processes limited-FoV LiDAR data to build occupancy grid
- Accepts both new dict format and legacy list format
- Particle filter tuned for 54-ray scans

### Communication (`core/communication.py`)
XBee 900 MHz mesh radio simulation:
- 12.5 KB/s bandwidth with rolling window tracking
- 200-byte fragment size with `fragment_message()` helper
- Priority queue: CRITICAL > HIGH > MEDIUM > LOW
- Compression methods for LiDAR keyframes, poses, detections, and map data

### Mesh Network (`core/mesh_network.py`)
Simulated XBee mesh protocol:
- Distance-dependent packet loss (1-15%)
- Per-hop latency (20-80 ms)
- Message size estimation and fragment validation
- AODV-style multi-hop routing

### Multi-Drone Coordination (`core/drone_manager.py`, `core/gossip_map.py`)
For 1-12 drone missions:
- Each drone maintains its own map
- Gossip protocol shares discoveries when in comm range
- Claimed targets prevent duplicate coverage
- 12 distinct drone colours for visualisation

## Display Panels

### Main View (Top Left)
- Building layout with walls
- Drone position and orientation
- LiDAR rays showing 59 deg sensor cone with FoV boundary lines
- IED detector range (blue circle)
- Detected objects labelled

### Discovered Map (Bottom Left)
- **Green squares**: Searched grid cells (IED-scanned)
- **Yellow dots**: Frontier cells (unexplored areas)
- **Blue labels**: Detected objects
- **Red "IED!"**: Detected explosives
- **Black lines**: Discovered walls

### Drone Status (Right Panel)
- Position, heading, speed
- Battery level and mission time
- Coverage percentage
- Search mode (SEARCH/RUSH/RETURN)
- Objects found

### Multi-Drone Panel (Bottom Right, when D>1)
- Drone count and comm range
- Active mesh links
- Global coverage percentage
- Per-drone status

## Algorithm Versions

| Version | Description | Coverage |
|---------|-------------|----------|
| V11.5h | Conservative return timing | 81% |
| V11.6 | Sweep patterns (had bugs) | 76% |
| **V11.6a** | Fixed entry oscillation, optimised timing | **93%+** |
| **V11.6a+hw** | Real hardware: STM LiDAR (59 deg/9m) + XBee 900MHz | TBD |

## Technical Details

### Return Timing Formula (V11.6a)
```
time_to_return = distance_to_exit / 1.2 + 70 seconds
```
- 1.2 m/s effective speed (accounts for obstacle navigation)
- 70 second safety margin
- Triggers return when `time_remaining < time_to_return`

### Grid Coverage Math
- IED detector range: 3m
- Grid cell size: 3m x 3m
- Visiting centre of each cell guarantees every point is within 3m
- Coverage % = (searched interior cells) / (discovered interior cells) x 100

### Periodic Rotation Scan
With only 59 deg FoV, a single scan covers ~16% of the surroundings. Every 5 simulated seconds the drone takes ~5 additional scans at rotated orientations, effectively building a 360 deg picture from accumulated limited-FoV sweeps.

### XBee Mesh Protocol
- Drones broadcast map updates when in comm range
- Data includes: searched cells, free cells, walls, features
- Distance-dependent packet loss: 1% at close range, up to 15% at max range
- Per-hop latency: 20-80 ms random
- Messages > 200 bytes must be pre-fragmented
- Version vectors prevent duplicate processing
- Multi-hop routing extends effective range

## Hardware Integration

The `core/` modules are designed for portability to real hardware:

1. **LiDAR**: Replace `environment.get_lidar_scan()` with STM 54x42 hardware driver — same dict format
2. **IED Sensor**: Replace `sensors.py` with actual sensor interface
3. **Flight Control**: Replace `physics.py` with real drone API
4. **Radio**: Implement `MeshProtocolInterface` for XBee 900 MHz hardware — protocol parameters already match

## License

MIT License - See LICENSE file for details.

## Contributing

Contributions welcome! Please read the technical documentation in `Drone_sim_README.md` before making changes.

Key guidelines:
- **Don't break single-drone mode** - It works well, test thoroughly
- **Multi-drone coordination is WIP** - Improvements welcome
- **Document version changes** - Update README and docstrings
- **LiDAR format**: All new code should handle the dict format from `get_lidar_scan()`
