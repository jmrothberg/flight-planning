# Drone IED Search Simulation

An autonomous drone simulation system for searching unknown buildings to locate IEDs (Improvised Explosive Devices). The drone must explore, map, and return home safely within a 7-minute mission window.

![Simulation Screenshot](r_s_m_v11.6a_02m20s_cov93pct_final_7min_18098.png)

## Mission Objectives

1. **Search an UNKNOWN building** - The drone has NO prior knowledge of the building layout
2. **Detect IEDs** - IED sensor has 3-meter range; must pass within 3m to detect
3. **Complete coverage** - Systematically cover all accessible areas
4. **Return safely** - Must return to start position before 7-minute time limit
5. **Don't get stuck** - Navigate around obstacles without getting trapped

## Key Constraints

- **Building is completely unknown** - Only LIDAR sensor data reveals the layout
- **7-minute time limit** - Battery/mission constraint
- **3-meter IED detection range** - Requires close proximity scanning
- **Must return home** - All data is lost if drone doesn't return

## How It Works

### Search Strategy (3m Grid Coverage)
1. **LIDAR maps walls** → Identifies rooms and free space (360° scan, 25m range)
2. **Divide into 3m grid** → IED detector has 3m range, so 3m grid = full coverage
3. **Visit each grid cell** → Nearest unsearched cell first, with sweep optimization
4. **Track coverage** → Searched cells / total discovered free cells
5. **Return before time expires** → Distance-based timing with safety margin

### Single Drone Performance
- Achieves **90-95% coverage** in typical buildings
- Uses boustrophedon-style sweep patterns for efficiency
- Completes rooms before moving to new areas

### Multi-Drone Mode
- 2-4 drones working together
- Mesh networking with gossip protocol for map sharing
- Coordinated frontier selection to avoid overlap
- Range-limited communication (adjustable)

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
| **D** | Cycle Drone Count (1→2→3→4→1) |
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
│   ├── communication.py   # Priority message queue, radio TX
│   ├── minimap.py         # Real-time discovered map tracking
│   ├── search_systematic_mapper.py  # Grid-based coverage algorithm (V11.6a)
│   ├── drone_manager.py   # Multi-drone orchestration
│   ├── mesh_network.py    # Range-limited mesh with AODV routing
│   └── gossip_map.py      # Distributed map sharing via gossip protocol
│
├── simulation/            # Simulation-only (not for real hardware)
│   ├── environment.py     # Fake building layouts, LIDAR/camera simulation
│   ├── graphics.py        # Pygame rendering, UI panels, minimap display
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

### SLAM System (`core/slam.py`)
Simultaneous Localization and Mapping:
- Processes LIDAR data to build occupancy grid
- Tracks drone position
- Provides map data for pathfinding

### Navigation (`core/navigation.py`)
Path planning and obstacle avoidance:
- A* pathfinding on occupancy grid
- Collision avoidance with walls
- Fallback navigation when A* fails

### Multi-Drone Coordination (`core/drone_manager.py`, `core/gossip_map.py`)
For 2+ drone missions:
- Each drone maintains its own map
- Gossip protocol shares discoveries when in comm range
- Claimed targets prevent duplicate coverage
- Mesh networking routes messages through nearby drones

## Display Panels

### Main View (Top Left)
- Building layout with walls
- Drone position and orientation
- LIDAR rays (red) showing sensor coverage
- IED detector range (blue circle)
- Detected objects labeled

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
| **V11.6a** | Fixed entry oscillation, optimized timing | **93%+** |

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
- Grid cell size: 3m × 3m
- Visiting center of each cell guarantees every point is within 3m
- Coverage % = (searched interior cells) / (discovered interior cells) × 100

### Multi-Drone Gossip Protocol
- Drones broadcast map updates when in comm range
- Data includes: searched cells, free cells, walls, features
- Version vectors prevent duplicate processing
- Multi-hop routing extends effective range

## Hardware Integration

The `core/` modules are designed for portability to real hardware:

1. **LIDAR**: Replace `environment.get_lidar_scan()` with hardware driver
2. **IED Sensor**: Replace `sensors.py` with actual sensor interface
3. **Flight Control**: Replace `physics.py` with real drone API
4. **Radio**: Implement `MeshProtocolInterface` for hardware mesh

## License

MIT License - See LICENSE file for details.

## Contributing

Contributions welcome! Please read the technical documentation in `Drone_sim_README.md` before making changes.

Key guidelines:
- **Don't break single-drone mode** - It works well, test thoroughly
- **Multi-drone coordination is WIP** - Improvements welcome
- **Document version changes** - Update README and docstrings
