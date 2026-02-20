# Drone IED Search Simulation

Autonomous drone swarm simulation for searching unknown buildings to locate IEDs.
Each drone has 3 processors that handle flight, AI/mapping, and radio independently
— exactly matching the real hardware architecture.

## Quick Start

```bash
python3 simulation_gui.py
```

Auto-starts on launch. Press **D** to add drones (1-12). Press **R** to reset.
Click a drone then **M** for manual control. Click **[MAP]/[DSTR]** buttons to change mission.

## The 3-Processor Architecture

Each physical drone has 3 chips connected via SPI and UART buses.
The code is organized the same way — one directory per processor:

```
                    ┌─────────────────────────────────┐
                    │         Per-Drone Hardware       │
                    │                                  │
  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
  │   STM32H7   │   │   STM32N6   │   │   STM32WL   │
  │   Flight    │◄──│   Neural    │◄──│    Mesh     │
  │  Controller │──►│  Processor  │──►│    Radio    │
  └─────────────┘   └─────────────┘   └─────────────┘
   Motor control     LiDAR + Camera    Gossip protocol
   PID loops         SLAM mapping      Map sharing
   GPS, battery      Search algorithm  Drone-to-drone
   IMU fusion        A* pathfinding    LoRa sub-GHz
```

### What Each Processor Does

| Processor | Chip | Clock | Role |
|-----------|------|-------|------|
| **H7** | STM32H755 Cortex-M7 | 480 MHz | Keeps the drone flying. PID stabilization at 400 Hz, motor control, GPS/IMU sensor fusion, battery monitoring, failsafe |
| **N6** | STM32N6 NPU+M55 | 800 MHz | The "brain." Processes LiDAR scans, builds a map (SLAM), estimates pose from IMU + optical flow, runs the search algorithm to pick targets, computes A* paths, detects IEDs and objects |
| **WL** | STM32WL sub-GHz | 48 MHz | The radio. Shares maps between drones using a gossip protocol over LoRa mesh. Discovers neighbors, broadcasts IED alerts |

### How They Talk to Each Other

Data flows between processors via typed messages defined in `core/processor_bus.py`:

```
H7 ──PositionUpdate──► N6     (x, y, z, orientation, battery) @ 100 Hz via SPI
N6 ──WaypointCommand──► H7    (target x, y, z) @ 10 Hz via SPI
N6 ──LocalMapData────► WL     (searched/free/wall cells for gossip broadcast)
WL ──RemoteMapData───► N6     (other drones' map data received via radio)
H7 ──PositionBeacon──► WL     (drone_id, position) @ 1 Hz via UART
```

## The 4 Programs

| Program | What it is | On real hardware? |
|---------|-----------|-------------------|
| `core/stm32h7/stm32h7_main.py` | Flight controller app (H7 firmware) | Yes — runs on the H7 chip |
| `core/stm32n6/stm32n6_main.py` | Neural processor app (N6 firmware) | Yes — runs on the N6 chip |
| `core/stm32wl/stm32wl_main.py` | Mesh radio app (WL firmware) | Yes — runs on the WL chip |
| `simulation_gui.py` | Simulation: creates the world, wires processors, runs GUI | No — the world is real on hardware |

## Directory Structure

```
flight_planning/
├── simulation_gui.py              # Simulation entry point (the "world" + GUI)
├── algorithm_config.py            # Search algorithm selection
├── requirements.txt
├── CLAUDE.md                      # AI assistant notes & bug documentation
│
├── core/                          # ── Drone brain (maps to real hardware) ──
│   ├── __init__.py                # Architecture documentation
│   ├── processor_bus.py           # Inter-processor message types (SPI/UART)
│   ├── drone_manager.py           # Multi-drone orchestration (sim-level)
│   │
│   ├── stm32h7/                   # ── H7: Flight Controller ──
│   │   ├── drone.py               # Drone physics, PID controller, motors
│   │   └── stm32h7_main.py       # FlightControllerApp (HAL + main loop)
│   │
│   ├── stm32n6/                   # ── N6: Neural Processor ──
│   │   ├── search_systematic_mapper.py  # THE search algorithm (target scoring)
│   │   ├── pose_estimator.py      # Pose estimation (IMU + optical flow + dead reckoning)
│   │   ├── navigation.py          # A* pathfinding (0.3m grid)
│   │   ├── sensors.py             # IED sensor (2m range)
│   │   ├── slam.py                # SLAM occupancy grid mapping
│   │   ├── vision.py              # Camera AI object detection
│   │   ├── minimap.py             # Real-time discovered map
│   │   └── stm32n6_main.py       # NeuralProcessorApp (main loop)
│   │
│   ├── stm32wl/                   # ── WL: Mesh Radio ──
│   │   ├── mesh_network.py        # Sub-GHz mesh with packet loss model
│   │   ├── gossip_map.py          # Distributed map sharing protocol
│   │   ├── communication.py       # Protocol stack, priority queue
│   │   └── stm32wl_main.py       # MeshRadioApp (main loop)
│
├── simulation/                    # ── Simulated world (NOT on any chip) ──
│   ├── environment.py             # Building layouts, LiDAR ray-casting
│   ├── graphics.py                # Pygame rendering, minimaps, UI panels
│   ├── joystick_widget.py         # Manual flight control joystick panel
│   └── physics.py                 # Drone flight dynamics, wall collision
```

## How a Mission Works

1. **Simulation creates a building** with random layout, places IEDs inside
2. **Each drone gets 3 processor apps** (H7, N6, WL) — one instance per chip
3. **Drone flies to the door** — H7 handles flight, approaches laser-targeted entry point
4. **Enters building, does 360° scan** — N6 processes LiDAR, builds initial map
5. **N6 picks search targets** using BFS-distance scoring with frontier/richness bonuses
6. **N6 sends WaypointCommand to H7** — H7 navigates there with PID + collision avoidance
7. **H7 sends PositionUpdate back to N6** — N6 updates map, marks cells as searched
8. **WL gossips map to neighbors** — other drones learn what this drone has mapped
9. **Repeat until 6 min** — forced return home. At 7 min battery dies.

## Key Hardware Specs (Modeled in Simulation)

| Component | Specification |
|-----------|--------------|
| LiDAR | ST 3D ToF: 54 rays, 59° HFoV, 9m range — maps walls |
| Camera | OV2640: 60° FoV, 8m range — detects people, equipment, hazards, weapons |
| IMU | 9-axis (accel + gyro + mag) — gyroscope provides heading at 400 Hz |
| Optical flow | PMW3901 — body-frame velocity from surface tracking |
| IED sensor | 2m proximity detection — triggers when drone passes near an IED |
| Grid resolution | 2m cells (matches IED sensor) |
| Mesh radio | Sub-GHz LoRa, 12.5 KB/s, distance-based loss |
| Max speed | 2.0 m/s |
| Battery | 6 min search + 1 min return = 7 min total |
| Wall collision | 0.2m radius pushback |
| A* grid | 0.3m resolution, 5000 max iterations |

### Three Sensor Modalities

Each drone has three independent sensors, all processed by the N6 neural processor:

1. **LiDAR** (ST 3D ToF) — 54 rays across 59° HFoV, 9m range. Maps walls and free space.
   Rotation scans every 1.5s give full 360° awareness. This is the primary navigation sensor.

2. **Camera** (OV2640) — 60° FoV, 8m range. Detects and classifies objects: people, equipment,
   hazards, weapons, furniture. Confidence decreases with distance. Occlusion-aware (can't see
   through walls). Detections are shared between drones via the gossip protocol.

3. **IED Sensor** — 2m proximity detector. Triggers when the drone passes within 2m of an IED.
   This is why the search grid uses 2m cells — visiting every cell guarantees full IED coverage.

The building is populated with randomized objects (3 people, 1 equipment, 1 hazard, 1 weapon,
1 IED) placed in valid open spaces. Press **N** to re-randomize object positions.

## Controls

| Key / Action | Effect |
|-------------|--------|
| **SPACE** | Start/Stop Mission |
| **R** | Reset Simulation |
| **D** | Cycle Drone Count (1-12) |
| **N** | New Object Placement (same building) |
| **B** | New Building Layout |
| **+/-** | Adjust Communication Range |
| **P** | Save Screenshot |
| **M** | Toggle Manual Control (select drone first) |
| **L** | Toggle Radio Link Mode (Long-Range / Mesh) |
| **E** | Toggle Pose Estimation (TRUTH → EST 100x → EST 1x) |
| **G** | Toggle GPS Correction (when estimation is ON) |
| **Click drone** | Select drone (yellow ring) |
| **Click [MAP]/[DSTR]** | Toggle drone mission (Map or Destroy) |
| **Click POSE button** | Same as E — cycle pose estimation mode |
| **ESC** | Exit |

## Search Algorithm (V11.9)

The search algorithm runs on the **N6 neural processor** (`core/stm32n6/search_systematic_mapper.py`).
It picks the next target cell by scoring every unsearched cell:

```
score = bfs_distance (through free cells, respects walls)
      - min(richness*0.7, 22)  region richness (unsearched in 3-cell radius)
      - 8   if frontier (adjacent to unknown space)
      - 5   doorway bonus (frontier with both free + unknown neighbors)
      + 2   if wall cell
      + 15  if claimed by another drone
      + (20-d)*0.8  if within 20m of another drone
      - 1.5 sweep continuation bonus
      - 1.0 region completion bonus
      - 0.5*N rush mode (N = unsearched neighbors, after 3 min)
```

**Lowest score wins.** The drone always goes to the best-scoring cell it can reach.

### Per-Drone Timers

| Time | Action |
|------|--------|
| 0-3 min | SEARCH mode — explore with frontier + richness scoring |
| 3+ min | RUSH mode — prefer clusters of unsearched cells |
| 6 min | Forced return home (no exceptions) |
| 7 min | Battery dead — drone stops |

## Multi-Drone Swarm (1-12 Drones)

- Each drone runs its own H7/N6/WL processor stack independently
- **Gossip protocol** (3-phase, every frame):
  1. N6 → WL: push local map data (searched/free/wall cells)
  2. WL ↔ WL: exchange gossip payloads with nearby drones (within comm range)
  3. WL → N6: feed merged data back into search algorithm
- **Spatial separation**: cells near other drones get penalty score (up to +16 within 20m)
- **Global coverage**: computed from all search algorithms directly (not gossip)
- Per-drone independent timers and screenshots

## Base Station (Ground Controller)

The **Discovered Map** is a realistic operator display, not a God's-eye debug view.
It shows ONLY what the base station has received via radio transmissions.

The base station is a physical ground controller sitting at the launch point
(where drone 0 starts). It has a radio (same WL mesh radio as the drones) but
no LiDAR, no camera, no sensors. It is the operator's only window into the building.

**What it knows = only what was radioed to it:**
- Map cells (walls, free, searched) arrive via the gossip protocol — either directly
  from a drone in comm range, or hopped through intermediate drones
- Wall outlines on the Discovered Map are LiDAR-quality segments, but filtered to only
  show walls in areas the base station has received data about — no "magic" walls
- Drone positions only appear on the Discovered Map if a position update reached
  the base station (direct or relayed)
- If a drone flies deep into the building and is out of range of all other drones,
  the base station has **no idea** where it is or what it found — until another drone
  relays that data
- IED detections, object sightings — same rule: only shown if gossip delivered them
- The **Objects Found** panel in the UI also reads from the base station — only objects
  that were radioed back appear in the count

**Radio visualization** shows signal flow direction (drone → base station), consistent
with how all other radio links are drawn. The base station appears as a white triangle
with a comm range circle on the physical map.

**Scale**: The base station's Discovered Map is rendered at the same scale as the
physical building map, so the operator can compare them directly.

## Manual Drone Control

Click any drone on the map to select it (yellow ring appears), then press **M** to
enter manual mode. A dual-stick joystick panel appears in the UI:

- **Left stick** — Forward/back and strafe
- **Right stick** — Yaw rotation
- IED sensor stays active during manual flight (cells are still marked as searched)
- At 6 minutes, manual mode auto-releases for forced return home
- Press **M** again to release and resume autonomous search

### Radio Link Modes

Manual control has two radio modes, toggled with **L** or clicking the radio button
in the Controls panel:

| Mode | Button | Processor | Range |
|------|--------|-----------|-------|
| **Long-Range** | `Long-Range (H7)` green | STM32H7 direct 900 MHz | Unlimited |
| **Mesh Radio** | `Mesh Radio (WL)` red | STM32WL mesh network | Mesh-limited |

- **Long-Range (default)** — Joystick commands go directly to the drone's STM32H7 flight
  controller over a dedicated 900 MHz link. Works at any distance. This is the standard
  military drone control link.

- **Mesh Radio** — Joystick commands travel through the STM32WL mesh network, hopping from
  the base station through intermediate drones to reach the target. Only works if there
  is a radio path (direct or multi-hop) from the base station to the drone.
  When the link drops, the drone hovers in place until the link is re-established.
  The joystick panel shows **MESH: LINKED** (green) or **MESH: NO LINK** (red).

## Per-Drone Missions (Map / Destroy)

Each drone has a mission shown in the Per-Drone Status panel as a clickable button:

| Mission | Button | Behavior |
|---------|--------|----------|
| **Map** | `[MAP]` (green) | Default — autonomous search algorithm explores the building |
| **Destroy** | `[DSTR]` (red) | Fly to a known IED position and destroy it |

**Destroy workflow:**
1. Click `[MAP]` to toggle a drone to `[DSTR]` mode
2. If the drone (or any drone via gossip) has already detected an IED, it immediately
   targets the closest un-destroyed IED and flies there using A* navigation
3. If no IED is known yet, the drone continues searching. Once an IED is found
   (by this drone or shared via gossip from another), it auto-targets it
4. When the drone arrives within 0.5m of the IED, it detonates — the IED is removed
   from the simulation and marked with a red X on the map
5. The drone automatically reverts to MAP mode after destruction

The Objects Found panel shows both `ied: N` (total detected) and `IED destroyed: N`
(total eliminated, in red). Object counts come from the base station's radio data —
only detections that were radioed back appear in the panel.

## Wall Collision Reporting

The per-drone status shows `W:N` — a count of real wall impacts. Only collisions where
the drone was moving >0.5 m/s count (not normal proximity clearance). In autonomous mode
with LiDAR navigation, this should be 0. Manual flight into walls will increment the counter.

## Pose Estimation (Realistic Localization)

Real drones can't read their true position — they must estimate it from noisy sensors.
The pose estimation pipeline (`core/stm32n6/pose_estimator.py`) simulates this:

### Sensor Pipeline

| Sensor | What it measures | Noise model |
|--------|-----------------|-------------|
| **IMU gyroscope** | Heading (yaw) | White noise + slow bias drift (random walk) |
| **Optical flow** | Body-frame velocity | Scale drift + velocity noise + 2% dropouts |
| **GPS** (optional) | Absolute position | 2.0m std noise, 1 Hz updates |

Each frame: noisy IMU yaw + noisy optical flow velocity → rotate to world frame → integrate
to estimated position. This is **dead reckoning** — it accumulates drift over time.

### Three Modes

| Mode | Button | Accuracy | Use case |
|------|--------|----------|----------|
| **TRUTH** | `POSE: TRUTH` (green) | Perfect | Default. Debug, baseline comparison |
| **EST 100x** | `POSE: EST 100x acc` (yellow) | 100x better than spec | Validate pipeline works. Map slightly noisier but usable |
| **EST 1x** | `POSE: EST 1x acc` (red) | Real-world spec | Realistic. Drift visible, may get stuck from accumulated error |

Toggle with **E** key or click the POSE button. GPS correction toggled with **G** key.

### Design Principle: Truth for Sensors, Estimate for Autonomy

- **Sensors use truth position** — LiDAR, camera, IED sensor physically ARE at the true
  position. You can't change where a sensor is by estimating wrong.
- **Autonomy uses estimated position** — SLAM mapping, search algorithm, A* navigation,
  minimap all use the drone's ESTIMATED position, because that's all a real drone knows.
- This means with estimation ON, the map has slight noise (walls shift by estimation error),
  and the search algorithm may make suboptimal decisions — exactly like a real system.

## Output Files

| File Pattern | Description |
|-------------|-------------|
| `d{id}_mid_{time}_{cov}pct.png` | Each drone at 3 min (mid-mission) |
| `d{id}_done_{time}_{cov}pct.png` | Each drone at completion |
| `sim_{n}d_{cov}pct_{time}.mp4` | Full mission video recording |

## Installation

```bash
git clone https://github.com/jmrothberg/flight-planning.git
cd flight-planning
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 simulation_gui.py
```

## Cost Estimates

| Phase | Per-drone electronics |
|-------|----------------------|
| Prototyping (dev boards) | ~$500-900 |
| Early builds | ~$200-350 |
| Production (1k+ units) | ~$100-180 |

## Future Roadmap

- **Scan Matching Against Pre-Built Maps** — Correct dead reckoning drift using keyframe-based
  submap matching (not self-built map, which causes feedback loops)
- **Magnetometer Fusion** — Indoor compass for absolute heading reference (noisy but bounded)
- **3D Navigation** — Multiple floors, stairs, vertical connections (grid becomes voxels)
- **Hallways & Tunnels** — Long narrow spaces with limited LiDAR visibility
- **900 MHz Control Link** — Pilot telemetry, failsafe simulation
- **Real Hardware Integration** — Deploy to STM32H7 + STM32N6 + STM32WL

## License

MIT License - See LICENSE file for details.
