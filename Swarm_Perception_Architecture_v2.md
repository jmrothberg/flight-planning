# Search & Rescue Swarm Drone
## Perception Architecture v2 (Synchronized Camera + LiDAR + Pose Estimation)

This document defines the perception architecture for the 150 g class Search & Rescue swarm drone system. It is aligned with:

- The Electronics Stack (STM32H7 / STM32N6 / STM32WL architecture)
- The Mesh Communications & Mission Concept specification

This document is structured to be clear to both human engineers and large language models.

**v2 Changes:** Added Pose Estimation Pipeline section (Section 7), updated sensor list with IMU and optical flow, clarified that all perception outputs use estimated pose.

---

## 1. Architectural Alignment with Electronics Stack

### Processor Roles

**STM32H7:**
- Flight control
- Stabilization and motor control
- Navigation and failsafe logic
- Interface to 900 MHz control radio
- **IMU reading** (gyroscope yaw rate, accelerometer)
- **Optical flow sensor reading** (body-frame velocity)

**STM32N6:**
- Camera processing
- 54x42 ToF LiDAR processing
- AI inference (Neural-ART NPU)
- Object detection
- Semantic mapping generation
- **Pose estimation pipeline** (dead reckoning + corrections)

**STM32WL:**
- 900 MHz sub-GHz mesh communications
- Store-and-forward relay behavior

### Data Flow

```
OV2640 Camera + 54x42 ST ToF --> STM32N6 --> detections / map deltas --> STM32H7 --> STM32WL mesh

IMU + Optical Flow --> STM32H7 --> sensor data packets --> STM32N6 (pose estimator)

STM32H7 <--> 900 MHz long-range radio (pilot control + failsafe)
```

### Sensor Suite

| Sensor | Interface | Data | Used By |
|--------|-----------|------|---------|
| OV2640 Camera | SPI/DCMI | 1 MP JPEG frames | N6 (object detection) |
| ST 3D ToF LiDAR | I2C/SPI | 54x42 depth grid + 54-ray horizontal scan | N6 (SLAM, scan matching) |
| IMU (6-axis) | SPI | Gyroscope yaw rate, accelerometer | H7 -> N6 (pose estimation) |
| Optical Flow | SPI/UART | Body-frame velocity (vx, vy), quality | H7 -> N6 (pose estimation) |
| GPS (optional) | UART | Latitude/longitude, 1 Hz | H7 -> N6 (pose correction) |

---

## 2. Real-Time RGB Perception (Primary Mode)

**Sensor:** OV2640 (~1 MP)

**Model:**
- YOLOv8n
- INT8 quantized
- TFLite format
- Input: 256x256 RGB

**Pipeline on STM32N6:**
1. Capture 1 MP frame
2. Downscale or crop to 256x256
3. Quantize to INT8
4. NPU inference
5. Confidence threshold >= 0.5
6. Select highest-confidence detection
7. Output: `label confidence%`

Target inference: 20-30 FPS.
Single top detection for bandwidth efficiency.

---

## 3. Low-Light Fallback: LiDAR-Only Perception

**Sensor:** ST 3D ToF 54x42 depth grid (~2,268 pixels)
`Depth[y][x]` -> distance (mm)

If RGB confidence falls or lighting insufficient, switch to LiDAR mode.

**Depth Model:**
- Tiny CNN
- INT8 TFLite
- Input: 54x42x1

**Initial classes:**
- person_like_blob
- doorway_opening
- wall_plane
- obstacle_blob
- empty_open

Mode switching handled locally on STM32N6 and transparent to STM32H7.

---

## 4. Teacher-Student Training Strategy

No pretrained models exist for 54x42 LiDAR grids.

**Training approach:**
1. Collect synchronized RGB + LiDAR frames
2. Run YOLOv8n on RGB
3. Filter labels (confidence >= 0.8)
4. Align with LiDAR grid
5. Train tiny CNN on depth only

Recommended dataset: ~10,000 synchronized frames.

---

## 5. Mesh Integration

Perception outputs are structured and compact.

**Detection packet includes:**
- Object type
- Confidence
- Position estimate (based on drone's **estimated** pose)

**Data sizes align with mesh constraints:**
- Object detection: ~100-500 bytes
- Map delta: 100-800 bytes
- LiDAR keyframe (compressed): 1-5 KB

No video streaming over 900 MHz mesh.

**All positions in mesh-transmitted data are estimated positions** from the pose estimation pipeline (Section 7). The mesh layer does not apply position corrections.

---

## 6. Operational Impact

Enables:
- Indoor mapping
- Human detection
- Corridor recognition
- Hazard detection
- Operation in darkness or smoke

Supports distributed store-and-forward mesh behavior.

---

## 7. Pose Estimation Pipeline (NEW in v2)

### Overview

A real drone cannot read its true position from the simulation engine. It must estimate its pose from noisy sensors. The Pose Estimation Pipeline runs alongside the perception stack on STM32N6 and produces an estimated (x, y, yaw) that all autonomy systems consume.

### Why Pose Estimation Matters for Perception

All perception outputs include position information:
- SLAM places wall cells relative to the drone's position
- Object detections are localized relative to the drone's position
- Search coverage marks cells as "searched" at the drone's position

If the drone's believed position drifts from reality, all perception outputs shift accordingly. The entire SLAM map, object positions, and coverage tracking operate in the drone's **estimated** coordinate frame.

### Sensor Pipeline

```
Step 1: Noisy IMU Yaw
   - Read gyroscope yaw rate from STM32H7
   - Add white noise (std=0.005 rad/step) + bias drift (0.0002 rad/s random walk)
   - Produces estimated heading

Step 2: Noisy Optical Flow Velocity
   - Read body-frame velocity from optical flow sensor via STM32H7
   - Add velocity noise (std=0.05 m/s) + scale drift (0.001/s)
   - Handle dropouts (2% chance, 0.1s duration)
   - Produces body-frame velocity estimate

Step 3: Dead Reckoning
   - Rotate body-frame velocity by estimated yaw
   - Integrate over dt to update (est_x, est_y)
   - This is the core position update

Step 4: GPS Correction (optional, outdoor/near-entrance)
   - 1 Hz position fix with noise (std=2.0m)
   - Weighted fusion pulls estimate toward GPS (weight=0.15)
   - Bounds long-term drift

Step 5: Scan Matching Correction (future)
   - Correlative scan matching against pre-built occupancy grid
   - MUST NOT match against self-built SLAM map (feedback loop)
   - Search 11x11x5 candidate offsets, score by LiDAR endpoint hits
```

### What Uses Estimated vs Truth Pose

| System | Uses | Reason |
|--------|------|--------|
| SLAM mapping | **Estimated** | Maps where drone *thinks* it is |
| Object detection positioning | **Estimated** | Localizes objects relative to believed position |
| Search algorithm | **Estimated** | Tracks coverage at believed position |
| A* navigation | **Estimated** | Plans path from believed position |
| LiDAR sensor (raw data) | **Truth** | Physical sensor IS at true position |
| Camera sensor (raw data) | **Truth** | Physical sensor IS at true position |
| IED proximity detector | **Truth** | Physical proximity uses real distance |
| Physics / collision | **Truth** | Simulation physics uses real position |

### Relationship to Perception Modes

| Perception Mode | Pose Estimation Impact |
|----------------|----------------------|
| **RGB detection** | Object positions calculated using estimated drone pose |
| **LiDAR-only detection** | Same — object positions use estimated pose |
| **SLAM mapping** | Wall cells placed at estimated position. Estimation drift = map drift |
| **LiDAR scan matching** | Future: LiDAR scans compared against map for pose correction |

### Critical Design Constraint: No Self-Referential Correction

Scan matching (comparing current LiDAR scan against the SLAM occupancy grid to correct position) creates a **positive feedback loop** when the SLAM map was built using the estimated pose:

1. Estimated position drifts slightly
2. SLAM records walls at the shifted position
3. Scan matching finds a new "best offset" against the corrupted map
4. Estimate shifts further in the same direction
5. SLAM records walls at the new shifted position
6. Repeat - error grows 0.3m per frame

**Solution:** Scan matching correction must only be applied against:
- Pre-built maps (from a previous mission)
- Frozen map snapshots (keyframes from earlier in the mission)
- External reference data (GPS, magnetic field map)

Dead reckoning with IMU + optical flow provides stable short-term accuracy. GPS (when available) bounds long-term drift. Future work: line feature matching for wall-based correction without the feedback loop.

### Accuracy Characteristics

| Condition | Expected Drift | Notes |
|-----------|---------------|-------|
| Short mission (<2 min) | <0.5m | Dead reckoning sufficient |
| Medium mission (2-6 min) | 0.5-3m | GPS correction helps near entrance |
| Long mission without GPS | 3-10m+ | Unbounded drift, needs scan matching |
| With GPS (1 Hz, outdoor) | <2m bounded | GPS noise limits accuracy |
| With scan matching (future) | <0.5m bounded | Best accuracy, requires pre-built map |

---

## 8. Future Perception Enhancements

### Scan Matching Against Pre-Built Maps
Use correlative or ICP-based scan matching against a frozen occupancy grid from a previous pass or externally provided floor plan.

### Line Feature Matching
Extract wall lines from LiDAR scans, match against known wall segments. Provides pose correction without the self-referential feedback loop.

### Magnetometer Integration
Indoor magnetic field signatures can provide heading correction independent of gyroscope drift.

### Multi-Drone Relative Localization
When two drones detect each other (LiDAR or visual), their relative position provides a constraint for correcting both drones' estimated poses simultaneously.
