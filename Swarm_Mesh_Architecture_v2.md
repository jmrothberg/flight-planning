# Search & Rescue Swarm Drone
## Mesh Communications Architecture v2 (Aligned System Specification)

This document defines the communications architecture for the 150 g class Search & Rescue swarm drone system. It is aligned with:

- Electronics Stack (STM32H7 / STM32N6 / STM32WL separation)
- Swarm Perception Architecture (Camera + LiDAR processing on STM32N6)
- **Pose Estimation Pipeline (IMU + optical flow dead reckoning on STM32N6)**

The purpose of this document is to provide a clear specification for both human engineers and large language models implementing simulation or firmware.

**v2 Changes:** Added pose estimation data types to mesh transmissions, updated drone pose packet format, added base station estimated position handling.

---

## 1. Mission Concept

**System:** 1-12 cooperative indoor swarm drones

**Mission example:**
- Enter unknown building
- Map interior collaboratively
- Detect people and hazards (e.g., IED-like objects)
- Share map and detections across swarm
- Relay mission-critical data back to base

The swarm must function even when individual drones lose direct base contact.

---

## 2. Processor & Radio Responsibilities

**STM32H7:**
- Flight control
- Navigation and failsafe
- Interface to 900 MHz control radio
- **IMU and optical flow sensor reading** (forwarded to N6 for pose estimation)

**STM32N6:**
- Camera + LiDAR perception
- Object detection
- Semantic map updates
- Structured detection packet generation
- **Pose estimation pipeline** (IMU + optical flow dead reckoning)

**STM32WL:**
- 900 MHz sub-GHz mesh networking
- Store-and-forward packet relay

**900 MHz long-range radio:**
- Pilot command
- Telemetry
- Emergency override

---

## 3. Radio Architecture (Phase 1)

First development phase uses only:
- 900 MHz XBee-class mesh radio

**Purpose:**
- Building penetration
- Multi-hop backbone
- Low-bandwidth reliable relay

**Future phase (not included yet):**
- 2.4 GHz short-range burst radio for high-speed local exchange

---

## 4. Mesh Philosophy

Distributed store-and-forward mesh.

Each drone:
- Acts as sensor platform
- Acts as relay node
- Stores undelivered data locally
- Forwards packets when routes become available

No central dependency required for mapping continuity.

---

## 5. Realistic Radio Constraints (Simulator Model)

| Parameter | Value |
|-----------|-------|
| Effective usable throughput | ~100 kbps |
| Per-hop latency | 20-80 ms |
| Packet loss | 1-15% |
| Max payload size | ~200 bytes |

System must:
- Fragment larger data
- Prioritize critical messages
- Avoid flooding

---

## 6. Data Types Transmitted

Structured data only (no video streaming).

| Data Type | Size | Contents |
|-----------|------|----------|
| **Drone pose updates** | 50-80 bytes | **Estimated** position (x, y), estimated heading (yaw), estimation confidence, timestamp |
| **Map deltas** | 100-800 bytes | Grid cell updates (free/wall/searched) at **estimated** positions |
| **Object detections** | 100-500 bytes | Object type, confidence, position estimate |
| **Compressed LiDAR keyframes** | 1-5 KB (fragmented) | Raw scan data for potential re-processing |

**Detection packet includes:**
- Object type
- Confidence
- Position estimate

### Pose Estimation Impact on Mesh Data

All position data transmitted over mesh reflects the drone's **estimated** position, not its true position. This means:

1. **Drone pose updates** contain the estimated (x, y, yaw) from the pose estimation pipeline. The base station and other drones receive where each drone *believes* it is.

2. **Map deltas** are generated at the estimated position. SLAM walls and free cells are recorded where the drone thinks it is standing. If estimation drifts, the shared map will have corresponding spatial error.

3. **Object detections** use the estimated position as the reference point for calculating object locations. Detection positions are relative to the drone's believed position.

4. **Implication for base station:** The Discovered Map at the base station shows the building as the swarm *perceives* it, including any estimation drift. This is realistic — an operator in the field would see the same estimated-position map.

---

## 7. Priority Model

| Priority | Data Type |
|----------|-----------|
| **Highest** | Mission commands, hazard detections |
| **Medium** | Map updates, drone pose updates |
| **Lower** | LiDAR keyframes, noncritical diagnostics |

Mesh layer must implement a priority queue.

---

## 8. Failure Model

**Assume:**
- Link drops
- Packet loss
- Temporary drone isolation

**Behavior:**
- Continue mapping locally (using local pose estimation)
- Store data
- Relay once reconnected

Progress must continue under partial connectivity.

**Pose estimation during isolation:** When a drone is out of mesh range, its pose estimation continues independently using onboard IMU and optical flow. Dead reckoning drift accumulates without external correction (GPS, scan matching). Upon reconnection, the drone's accumulated map data is synced, but with whatever positional drift has occurred.

---

## 9. Integration with Perception Architecture

Perception (STM32N6) generates compact detection and map packets.
STM32H7 handles flight stability and passes packets to STM32WL.
Mesh propagates detections hop-by-hop.

Hazard detections are immediately elevated in priority.

All drones maintain partial global map redundancy.

### Pose Estimation Integration

The pose estimation pipeline on STM32N6 determines where the drone believes it is. This estimated position is used as the origin for all perception outputs:
- SLAM map cells are placed at the estimated position
- Object detections are localized relative to estimated position
- Search coverage is tracked at the estimated position

The mesh layer transmits these perception outputs as-is. No position correction occurs at the mesh layer — correction happens at the source (STM32N6 pose estimator).

---

## 10. Base Station & Operator View

The base station receives all mesh data and presents the operator's view of the mission.

**Key principle:** The Discovered Map shows only what was physically radioed to the base station. It reflects:
- Drone positions as **estimated** by each drone's pose pipeline
- SLAM walls placed at **estimated** positions
- Object detections at **estimated** positions

The operator sees the building as the swarm perceives it. Any estimation drift is visible in the operator's map. This is realistic — a real ground station would receive the same estimated data.

**GPS correction** (when available, e.g., near building entrance) helps bound the drift, keeping the operator's map close to ground truth.
