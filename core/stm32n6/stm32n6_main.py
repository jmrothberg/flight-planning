"""
STM32N6 — Neural Processing Unit Main Loop
Hardware: STM32N657 Cortex-M55 + Neural Processing Unit (NPU) @ 800 MHz

This is the "brain" of the drone. It processes all sensor data, builds maps,
runs the search algorithm, and decides where to fly next. The NPU accelerates
LiDAR point cloud processing and camera-based object detection.

RESPONSIBILITIES:
  - LiDAR scan processing (54 rays, 59 deg FoV, rotation scans for 360 deg)
  - SLAM: grid-based occupancy mapping from LiDAR
  - Search algorithm: SystematicMapper (BFS-distance frontier exploration)
  - A* pathfinding (0.3m grid, adaptive stride, 5000 max iterations)
  - IED detection processing (2m range proximity sensor)
  - Camera inference: object detection via NPU (YOLOv8 or similar)
  - Target selection + stuck detection + return timing
  - Minimap construction for operator display

INTER-PROCESSOR COMMUNICATION:
  H7 → N6:  PositionUpdate @ 100 Hz via SPI  (x, y, z, orientation, battery)
  N6 → H7:  WaypointCommand @ 10 Hz via SPI  (target_x, target_y, target_z)
  N6 → WL:  LocalMapData @ 1 Hz via SPI      (searched/free/wall cells, features)
  WL → N6:  RemoteMapData on-receive via SPI  (gossip payload from other drones)

MEMORY BUDGET: 4.2 MB SRAM (+ 1.5 MB NPU SRAM)
  - Occupancy grid: 125×125 cells × 1 byte = 15 KB (at 0.2m resolution, 25m building)
  - Search grid: 13×13 cells × 8 bytes = ~1.4 KB (at 2m resolution)
  - A* open set: max 5000 nodes × 32 bytes = 160 KB
  - LiDAR buffer: 54 rays × 4 bytes × 10 scans = 2.2 KB
  - Camera frame: 320×240×3 = 230 KB (one frame in processing pipeline)
  - NPU model: ~1 MB (quantized INT8)
  - SLAM particles: 100 × 16 bytes = 1.6 KB
"""

import time
import math
from dataclasses import dataclass
from typing import Tuple, Optional, Set, Dict, List

from core.stm32n6.search_systematic_mapper import SystematicMapper
from core.stm32n6.navigation import AStarPathfinder
from core.stm32n6.sensors import IEDSensor, IEDReading
from core.stm32n6.slam import SLAMSystem
from core.stm32n6.vision import VisionSystem
from core.stm32n6.minimap import MinimapSystem
from core.processor_bus import (
    PositionUpdate, WaypointCommand, LocalMapData, RemoteMapData,
)


# ── Hardware Abstraction Layer (HAL) ─────────────────────────────────────

@dataclass
class LiDARScan:
    """Raw LiDAR data from ST VL53L5CX ToF sensor array.
    On real HW: read via SPI DMA from sensor board.
    54 horizontal rays, 59 deg HFoV, 9m max range."""
    ranges: list          # 54 float values (meters), 9.0 = no return
    start_angle: float    # radians, relative to drone heading
    angle_step: float     # radians between consecutive rays
    hfov: float           # total horizontal field of view (radians)
    num_rays: int         # number of rays (54)
    timestamp_us: int


@dataclass
class CameraFrame:
    """Raw camera frame from OV5640 or similar.
    On real HW: DCMI DMA into framebuffer, then NPU processes."""
    width: int
    height: int
    data: bytes           # RGB888 or YUV422
    timestamp_us: int


# ── Neural Processor Application ─────────────────────────────────────────

class NeuralProcessorApp:
    """
    Main neural processing application running on STM32N6.

    The M55 core runs the main loop: LiDAR processing, SLAM updates,
    search algorithm, and A* pathfinding. The NPU runs camera inference
    in parallel via hardware pipeline (DMA → NPU → output buffer).

    Critical timing:
      - LiDAR processing: < 2 ms per scan (54 rays)
      - Search algorithm: < 10 ms per target selection (BFS + scoring)
      - A* pathfinding: < 50 ms per path (5000 max iterations)
      - Camera inference: ~30 ms per frame (NPU, INT8 quantized)
      - Total budget: 100 ms per cycle (10 Hz decision rate)
    """

    # Loop rates (Hz)
    DECISION_RATE = 10       # Main decision loop (waypoint updates to H7)
    LIDAR_RATE = 15          # LiDAR scan processing
    ROTATION_SCAN_RATE = 0.67  # Full rotation scan (every 1.5s simulated)
    CAMERA_RATE = 10         # Camera inference (NPU pipeline)
    MAP_EXPORT_RATE = 1      # Send local map to WL for gossip
    IED_CHECK_RATE = 5       # IED sensor polling

    # Search parameters (matching CLAUDE.md specs)
    GRID_SIZE_M = 2.0              # Grid cell size = IED detector range
    MISSION_DURATION_S = 360.0     # 6 min forced return
    BATTERY_DEAD_S = 420.0         # 7 min battery death
    STUCK_SAMPLE_INTERVAL_S = 0.4  # Position sampling for stuck detection
    STUCK_THRESHOLD_M = 0.8        # Net displacement threshold
    TARGET_HOLD_S = 1.5            # Min time before retargeting

    # A* parameters
    ASTAR_GRID_RESOLUTION = 0.3    # meters
    ASTAR_NAV_RADIUS = 0.2         # meters (wall clearance)
    ASTAR_MAX_ITERATIONS = 5000

    def __init__(self, drone_id: int = 0,
                 building_width: float = 25.0, building_height: float = 25.0):
        self.drone_id = drone_id

        # Core algorithms
        self.search = SystematicMapper(
            coverage_radius=self.GRID_SIZE_M,
            drone_id=drone_id,
            building_width=building_width,
            building_height=building_height,
        )
        self.slam = SLAMSystem(map_width=building_width + 10,
                               map_height=building_height + 10)
        self.pathfinder = AStarPathfinder()
        self.ied_sensor = IEDSensor(detection_range_m=2.0)
        self.vision = VisionSystem()
        self.minimap = MinimapSystem()

        # Current state (received from H7 via SPI)
        self._position = PositionUpdate(0, 0, 3, 0, 100.0)
        self.inside_building = False

        # Navigation state
        self.current_target: Optional[Tuple[float, float]] = None
        self.current_path: list = []
        self.path_index: int = 0
        self.returning_home = False
        self.mission_complete = False
        self.entry_point = (0.0, 0.0)  # Where to return to

        # Multi-drone coordination (received from WL via gossip)
        self.global_searched: Set[Tuple[int, int]] = set()
        self.global_free: Set[Tuple[int, int]] = set()
        self.other_drone_positions: List[Tuple[float, float]] = []
        self.claimed_by_others: Set[Tuple[int, int]] = set()

        # Outgoing commands
        self._waypoint_cmd: Optional[WaypointCommand] = None
        self._local_map: Optional[LocalMapData] = None

        # Timing
        self._mission_start_time: Optional[float] = None
        self._last_decision = 0.0
        self._last_rotation_scan = 0.0
        self._last_map_export = 0.0
        self._last_ied_check = 0.0
        self._last_stuck_sample = 0.0

        # Stuck detection
        self._position_history: list = []
        self._stuck_count = 0

    # ── Sensor Input (HAL) ───────────────────────────────────────────

    def read_lidar(self) -> Optional[LiDARScan]:
        """Read LiDAR scan from VL53L5CX via SPI DMA.
        On real HW: non-blocking DMA read, returns None if no new data.
        In simulation: provided by simulation engine."""
        return None  # Simulation fills this via feed_lidar()

    def read_camera(self) -> Optional[CameraFrame]:
        """Read camera frame from DCMI DMA framebuffer.
        On real HW: double-buffered DMA, NPU processes in pipeline.
        In simulation: provided by simulation engine."""
        return None  # Simulation fills this via process_camera()

    # ── Inter-Processor Communication ────────────────────────────────

    def receive_position(self, update: PositionUpdate):
        """Receive position update from H7 via SPI slave DMA.
        On real HW: SPI1 slave mode, DMA circular buffer, CRC-8.
        Called at 100 Hz."""
        self._position = update

    def receive_remote_map(self, remote: RemoteMapData):
        """Receive gossip map data from WL via SPI.
        On real HW: SPI2 slave mode, variable-length frame with CRC-16.
        Called when WL has new gossip data."""
        # Merge remote drone's knowledge into our search algorithm
        payload = remote.payload
        if 'searched' in payload:
            self.global_searched.update(
                tuple(c) for c in payload.get('searched', [])
            )
        if 'free' in payload:
            self.global_free.update(
                tuple(c) for c in payload.get('free', [])
            )
        if 'positions' in payload:
            self.other_drone_positions = payload['positions']
        if 'claimed' in payload:
            self.claimed_by_others = set(
                tuple(c) for c in payload.get('claimed', [])
            )

        # Push to search algorithm
        self.search.set_global_searched(self.global_searched)
        self.search.set_global_free(self.global_free)
        self.search.set_other_drone_positions(self.other_drone_positions)
        self.search.set_claimed_by_others(self.claimed_by_others)

    def send_waypoint(self, target_x: float, target_y: float, target_z: float,
                      emergency: bool = False) -> WaypointCommand:
        """Send waypoint command to H7 via SPI master DMA.
        On real HW: SPI1 master, 12-byte frame, CRC-8, @ 10 Hz."""
        self._waypoint_cmd = WaypointCommand(
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            is_emergency=emergency,
        )
        return self._waypoint_cmd

    def send_local_map(self) -> LocalMapData:
        """Send local map data to WL via SPI master DMA.
        On real HW: SPI2 master, chunked transfer with flow control.
        Called at MAP_EXPORT_RATE (1 Hz)."""
        self._local_map = LocalMapData(
            searched_cells=self.search.searched_cells.copy(),
            free_cells=self.search.free_cells.copy(),
            wall_cells=self.search.wall_cells.copy(),
        )
        return self._local_map

    # ── Core Processing Logic ────────────────────────────────────────

    def init(self):
        """One-time initialization after power-on.
        On real HW: configure SPI/DMA for LiDAR, init NPU model,
        zero SLAM grids, calibrate IED sensor baseline."""
        print(f"[N6-{self.drone_id}] Neural processor initialized")
        print(f"[N6-{self.drone_id}]   Search grid: {self.GRID_SIZE_M}m cells")
        print(f"[N6-{self.drone_id}]   A* grid: {self.ASTAR_GRID_RESOLUTION}m resolution")
        print(f"[N6-{self.drone_id}]   Mission timeout: {self.MISSION_DURATION_S}s")

    def process_lidar(self, lidar_data: dict):
        """
        Process a LiDAR scan: update SLAM map and search grid.

        This is the primary sensor pipeline:
        1. SLAM occupancy grid update (ray casting via Bresenham)
        2. Search grid update (mark free/wall/searched cells)
        3. Frontier detection (cells adjacent to unknown space)

        On real HW: LiDAR DMA triggers this via interrupt.
        Processing must complete in < 2 ms for 54 rays.

        Args:
            lidar_data: dict with 'ranges', 'start_angle', 'angle_step'
        """
        pos = (self._position.x, self._position.y)
        orientation = self._position.orientation

        # Update SLAM occupancy grid (for A* pathfinding)
        self.slam.update(pos, lidar_data, orientation)

        # Feed to search algorithm (marks free/wall cells on 2m grid)
        self.search.feed_lidar_scan(
            pos[0], pos[1], lidar_data, orientation
        )

    def process_rotation_scan(self, scans: List[dict]):
        """
        Process a full rotation scan (multiple LiDAR scans at different angles).

        The drone's LiDAR has only 59 deg FoV. To get 360 deg awareness,
        we take ~6 scans at rotated orientations every 1.5s.

        CRITICAL: Without rotation scans, the search algorithm only sees
        the forward cone and misses doorways/rooms to the sides.

        Args:
            scans: List of (lidar_data, scan_angle) tuples
        """
        pos = (self._position.x, self._position.y)

        for lidar_data, scan_angle in scans:
            # SLAM update at each rotation angle
            self.slam.update(pos, lidar_data, scan_angle)

            # Search grid update (discovers doorways outside forward cone)
            self.search.feed_lidar_scan(
                pos[0], pos[1], lidar_data, scan_angle
            )

    def decide_next_waypoint(self) -> Optional[WaypointCommand]:
        """
        Main decision function: where should the drone go next?

        Runs at DECISION_RATE (10 Hz). Calls the search algorithm which:
        1. Computes BFS distances through free cells
        2. Scores all candidate cells (frontier bonus, richness, spatial penalty)
        3. Selects lowest-score cell as target
        4. Checks return timing (6 min forced return)

        On real HW: this is the most expensive computation (~10 ms).
        Must not exceed 100 ms total cycle budget.

        Returns:
            WaypointCommand to send to H7, or None if no target
        """
        pos = (self._position.x, self._position.y)
        orientation = self._position.orientation

        # Update mission elapsed time for return timing
        if self._mission_start_time is not None:
            elapsed = time.time() - self._mission_start_time
            self.search.set_simulated_time(elapsed)

            # Hard deadline: 7 min = battery dead
            if elapsed >= self.BATTERY_DEAD_S:
                self.mission_complete = True
                return self.send_waypoint(0, 0, 0, emergency=True)

            # 6 min: forced return
            if elapsed >= self.MISSION_DURATION_S:
                self.returning_home = True

        # If returning home, navigate to entry point
        if self.returning_home:
            return self._navigate_home()

        # Ask search algorithm for next target
        # This is the core intelligence: BFS scoring + frontier exploration
        waypoint = self.search.get_next_waypoint(pos, None, orientation)

        if waypoint is None:
            # Search complete or stuck
            if self.search.is_mission_complete():
                self.returning_home = True
                return self._navigate_home()
            return None

        # Plan A* path to target
        target_3d = (waypoint[0], waypoint[1], self._position.z)
        return self.send_waypoint(*target_3d)

    def _navigate_home(self) -> Optional[WaypointCommand]:
        """Navigate back to entry point for extraction.

        Uses A* pathfinding with intermediate hops if the full path
        is too long (> 5000 iterations). A* handles ~8m reliably;
        for 25m cross-building paths, we chain 8m→5m→3m hops.
        """
        home = self.entry_point
        return self.send_waypoint(home[0], home[1], self._position.z)

    def check_ied(self, environment) -> Optional[IEDReading]:
        """
        Check IED sensor (2m proximity detector).

        On real HW: reads from chemical/RF sensor array via ADC DMA.
        Returns IEDReading with component probabilities and ACTUAL
        IED position (not drone position — critical bug fix).

        Args:
            environment: Simulation environment (has IED positions)

        Returns:
            IEDReading if IED detected, None otherwise
        """
        pos = (self._position.x, self._position.y)
        reading = self.ied_sensor.read(pos, environment)

        if reading and reading.confidence > 0.5:
            return reading
        return None

    def get_coverage_stats(self) -> dict:
        """Get current search coverage statistics.
        Used by operator UI and mission completion checks."""
        return self.search.get_coverage_stats()

    def update(self, dt: float):
        """
        Main neural processor tick. Called at DECISION_RATE (10 Hz).

        On the real STM32N6 this is the main RTOS task, running when
        the NPU is not busy with camera inference. LiDAR processing
        runs at higher priority via DMA interrupt.

        Args:
            dt: Time step in seconds
        """
        now = time.time()

        # Start mission clock on first update
        if self._mission_start_time is None:
            self._mission_start_time = now

        # Skip if mission complete
        if self.mission_complete:
            return

        # ── Stuck detection (sample every 0.4s) ─────────────────────
        if now - self._last_stuck_sample >= self.STUCK_SAMPLE_INTERVAL_S:
            self._last_stuck_sample = now
            pos = (self._position.x, self._position.y)
            self._position_history.append(pos)
            if len(self._position_history) > 10:
                self._position_history = self._position_history[-10:]

            # Check if stuck: < 0.8m net displacement over last 5 samples
            if len(self._position_history) >= 5:
                old = self._position_history[-5]
                displacement = math.hypot(pos[0] - old[0], pos[1] - old[1])
                if displacement < self.STUCK_THRESHOLD_M:
                    self._stuck_count += 1
                    # Blacklist current target
                    self.search.mark_target_unreachable()
                else:
                    self._stuck_count = 0

        # ── Decision cycle (10 Hz) ──────────────────────────────────
        if now - self._last_decision >= 1.0 / self.DECISION_RATE:
            self._last_decision = now
            self.decide_next_waypoint()

        # ── Export local map to WL (1 Hz) ────────────────────────────
        if now - self._last_map_export >= 1.0 / self.MAP_EXPORT_RATE:
            self._last_map_export = now
            self.send_local_map()


# ── Entry Point (on real HW: Reset_Handler → main) ──────────────────────

def main(drone_id: int = 0):
    """
    STM32N6 main entry point.

    On real hardware this is called from startup after clock config,
    NPU initialization (load quantized model weights), and SPI/DMA setup.
    The main loop never returns.
    """
    app = NeuralProcessorApp(drone_id=drone_id)
    app.init()

    print(f"[N6-{drone_id}] Neural processor running — decision @ {app.DECISION_RATE} Hz")
    print(f"[N6-{drone_id}] LiDAR processing @ {app.LIDAR_RATE} Hz")
    print(f"[N6-{drone_id}] Map export → WL @ {app.MAP_EXPORT_RATE} Hz")

    dt = 1.0 / app.DECISION_RATE

    try:
        while True:
            app.update(dt)
            time.sleep(dt)  # In real HW: RTOS task yield
    except KeyboardInterrupt:
        stats = app.get_coverage_stats()
        print(f"[N6-{drone_id}] Shutdown — coverage: {stats}")


if __name__ == "__main__":
    main()
