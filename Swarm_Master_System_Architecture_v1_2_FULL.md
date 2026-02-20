# Search & Rescue Swarm Drone System
## Master System Architecture Specification v1.2 (Complete & Aligned)

This document consolidates the Electronics Stack, Perception Architecture, Pose Estimation Pipeline, and Mesh Communications Architecture into a single aligned system specification. It preserves all previously defined cost sections, processor separation, and mission philosophy.

**v1.2 Changes:** Added Pose Estimation Pipeline (Section 3), updated Electronics Stack with IMU and optical flow sensors, updated Data Flow with estimation pipeline.

---

## 1. Mission Overview

Deploy 1-12 cooperative 150 g class drones for indoor search & rescue and hazard (IED-like) detection. Each drone maps rooms, identifies people and objects, shares small structured map updates over mesh, and maintains a long-range pilot link.

---

## 2. Electronics Stack (150 g Class)

### System Architecture Overview

| Processor | Role |
|-----------|------|
| **STM32H7** | Flight control, stabilization, navigation, motor control, failsafe, IMU processing, optical flow sensor interface |
| **STM32N6** | Camera + LiDAR processing, AI inference, object detection, semantic mapping, **pose estimation pipeline** |
| **STM32WL** | Sub-GHz mesh radio for building/tunnel communication |
| **900 MHz long-range radio** | Pilot command, telemetry, emergency control link |
| **Camera** | OV2640 low-weight JPEG camera (~1 MP) |
| **LiDAR** | ST 3D ToF 54x42 depth sensor (horizontal: 54 rays, 59 deg HFoV, 9m range) |
| **IMU** | 6-axis accelerometer + gyroscope (yaw rate for dead reckoning) |
| **Optical Flow Sensor** | Downward-facing flow sensor (body-frame velocity estimation) |

### Data Flow

```
Camera + LiDAR --> STM32N6 --> detections/map updates --> STM32H7 --> mesh radio (STM32WL)

IMU + Optical Flow --> STM32H7 --> pose sensor data --> STM32N6 (pose estimator)

STM32H7 <--> 900 MHz control radio for operator link and failsafe
```

**Pose Estimation Data Flow:**
1. STM32H7 reads IMU (gyroscope yaw rate) and optical flow sensor (body-frame velocity)
2. STM32H7 sends `IMUUpdate` and `OpticalFlowUpdate` to STM32N6 via SPI bus
3. STM32N6 runs pose estimation pipeline: dead reckoning from IMU yaw + optical flow velocity
4. STM32N6 uses estimated pose for SLAM mapping, search decisions, and A* navigation
5. Estimated pose transmitted over mesh as part of drone pose updates

### Design Principle

Flight controller (STM32H7) remains deterministic and isolated. All AI, perception, and pose estimation tasks execute on STM32N6. Sensors (IMU, optical flow) are read by H7 and forwarded as structured data packets.

---

## 3. Pose Estimation Pipeline (NEW in v1.2)

### Overview

Real drones cannot read their true position — they must estimate it from noisy sensors. The Pose Estimation Pipeline runs on STM32N6 and produces an estimated (x, y, yaw) that the entire autonomy stack uses instead of truth position.

### Sensor Inputs

| Sensor | What It Provides | Noise Characteristics |
|--------|------------------|----------------------|
| **IMU (gyroscope)** | Yaw rate / heading | White noise (std=0.005 rad/step) + slow bias drift (0.0002 rad/s random walk) |
| **Optical flow** | Body-frame velocity (forward, right) | Velocity noise (std=0.05 m/s) + scale drift (0.001/s) + 2% dropout chance |
| **GPS** (outdoor/optional) | Absolute position | Position noise (std=2.0m), 1 Hz update rate |
| **LiDAR** (future) | Scan matching correction | Correlative matching against occupancy grid |

### Estimation Pipeline

```
1. Read noisy IMU yaw    --> estimated heading (with drift)
2. Read noisy optical flow --> body-frame velocity (with noise + dropouts)
3. Dead reckoning         --> rotate body velocity by IMU yaw, integrate to (est_x, est_y)
4. GPS correction         --> if enabled, pull estimate toward noisy GPS (weighted fusion)
5. Scan matching          --> future: correct against pre-built map (NOT self-built SLAM map)
```

### What Uses Estimated Pose vs Truth Pose

| System | Uses | Reason |
|--------|------|--------|
| SLAM mapping | **Estimated** | Drone records walls where it *thinks* it is |
| Search algorithm | **Estimated** | Target selection based on believed position |
| A* navigation | **Estimated** | Path planning from believed position |
| Minimap rendering | **Estimated** | Shows drone's internal model |
| LiDAR sensor | **Truth** | Sensor physically IS at true position |
| Camera sensor | **Truth** | Sensor physically IS at true position |
| IED detector | **Truth** | Proximity sensor uses real distance |
| Physics/collision | **Truth** | Physical simulation uses real position |

### Operating Modes (Simulation)

| Mode | Key | Description |
|------|-----|-------------|
| Truth pose (default) | E | Autonomy reads true position directly (debugging baseline) |
| 100x accuracy | E | Noise reduced to 1/100th of spec (optical flow 100x better than real hardware) |
| 1x accuracy | E | Full real-world noise levels (maximum realism) |
| GPS correction | G | Toggle GPS fusion (only active when estimation is ON) |

### Design Decisions

1. **Dead reckoning is primary**: IMU + optical flow provide continuous position updates. GPS and scan matching are corrections applied on top.

2. **Scan matching disabled against self-built maps**: Matching against a SLAM map that was built using the estimated pose creates a positive feedback loop — shifted estimates corrupt the map, which amplifies further shifts. Scan matching should only be used against pre-built or externally-provided maps.

3. **Actual velocity, not commanded**: The optical flow sensor measures actual drone velocity (position delta / dt), not the commanded velocity from the flight controller. A drone stuck at a wall reads zero velocity, not the 2 m/s it's trying to fly.

4. **Per-drone independent estimators**: In multi-drone mode, each drone has its own PoseEstimator with independent noise state. Drift is uncorrelated between drones.

### Future: Wall-Based Correction

Current dead reckoning drifts over time without external correction. Planned approaches:
- **Line feature matching**: Extract wall lines from LiDAR, match against known wall segments in SLAM map. Provides correction without the feedback loop of correlative scan matching.
- **Keyframe-based correction**: Store high-confidence scan snapshots, match new scans against snapshots (not the live SLAM map).
- **Magnetometer integration**: Indoor magnetic field mapping for heading correction.

---

## 4. Perception Architecture

### 4.1 Primary Mode - Real-Time RGB Detection

**Sensor:** OV2640 (~1 MP JPEG camera)

**Model:** YOLOv8n INT8 (TFLite)

**Input to model:** 256x256 RGB

**Target inference:** 20-30 FPS

**Pipeline on STM32N6:**
1. Capture 1 MP frame
2. Downscale/crop to 256x256
3. Normalize and quantize to INT8
4. Run inference on Neural-ART NPU
5. Apply confidence threshold (>= 0.5)
6. Select highest-confidence detection
7. Output format: `label confidence%`

### 4.2 Fallback Mode - LiDAR-Only Detection

**Sensor:** ST 3D ToF 54x42 depth grid (~2,268 pixels)
`Depth[y][x]` -> distance (mm)

Activated when lighting insufficient or RGB confidence unstable.

**Model:** Tiny INT8 TFLite CNN
**Input:** 54x42x1

**Initial geometric classes:**
- person_like_blob
- doorway_opening
- wall_plane
- obstacle_blob
- empty_open

Mode switching handled locally on STM32N6 and transparent to STM32H7.

---

## 5. Training Strategy (Teacher-Student)

There are no pretrained semantic models for 54x42 depth grids.

**Training Process:**
1. Collect synchronized RGB + LiDAR frames
2. Run YOLOv8n on RGB
3. Accept detections with confidence >= 0.8
4. Align camera detections with LiDAR grid
5. Train tiny CNN to predict labels from depth only

Recommended initial dataset: ~10,000 synchronized frames.

---

## 6. Mesh Communications Architecture

Phase 1 uses only 900 MHz XBee-class mesh radio (STM32WL).

### Realistic Constraints

| Parameter | Value |
|-----------|-------|
| Effective throughput | ~100 kbps |
| Per-hop latency | 20-80 ms |
| Packet loss | 1-15% |
| Max payload | ~200 bytes |

Store-and-forward model. Drones relay data hop-by-hop. Structured data only (no video streaming).

### Priority Model

| Priority | Data Type |
|----------|-----------|
| **Highest** | Mission commands, hazard detections |
| **Medium** | Map updates, **pose estimation corrections** |
| **Lower** | LiDAR keyframes, diagnostics |

### Data Types Transmitted

1. **Drone pose updates** (50-80 bytes) - includes estimated position and heading
2. **Map deltas** (100-800 bytes) - SLAM grid updates at estimated positions
3. **Object detections** (100-500 bytes) - type, confidence, position estimate
4. **Compressed LiDAR keyframes** (1-5 KB, fragmented)

**Redundancy Principle:** Multiple drones intentionally maintain overlapping map data so mission information survives loss of individual units.

**Pose Estimation Note:** Drone positions transmitted over mesh are *estimated* positions (from the pose estimation pipeline), not truth positions. The base station's Discovered Map reflects where drones *believe* they are. Position accuracy depends on each drone's estimation quality.

---

## 7. Initial Development Hardware (Low Quantity Estimates)

| Component | Cost |
|-----------|------|
| STM32H7 dev board | $30-60 |
| STM32N6 Discovery kit | $120-180 |
| STM32WL Nucleo board | $40-60 |
| RFD900x-class control radio pair | $180-260 |
| OV2640 camera module | $6-12 |
| ST 3D ToF LiDAR | $25-60 |
| IMU module (6-axis) | $5-15 |
| Optical flow sensor | $10-25 |

**Starter budget estimate:** ~$550-950 total for early prototyping (updated with IMU and optical flow).

---

## 8. Production Direction (Approximate Volume Pricing)

| Component | Cost |
|-----------|------|
| STM32H7 MCU | $8-18 |
| STM32N6 MCU | $12-30 |
| STM32WL radio module | $5-15 |
| 900 MHz control radio OEM | $50-130 |
| OV2640 camera | $2.5-7 |
| 3D ToF LiDAR | $12-35 |
| IMU (integrated) | $1-3 |
| Optical flow sensor | $3-8 |

**Electronics BOM estimate:**
- Early builds: ~$210-370 per drone
- 1k+ units: ~$105-190 per drone
