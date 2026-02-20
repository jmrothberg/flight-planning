"""
Pose Estimator — Realistic 2D Position/Heading Estimation Pipeline

Simulates what a real drone must do: estimate its pose from noisy sensors
rather than reading truth position directly.

Pipeline:
  1. Noisy IMU yaw (white noise + bias drift)
  2. Noisy optical flow velocity (scale drift + dropouts)
  3. Dead reckoning: integrate body-frame velocity rotated by IMU yaw
  4. Optional GPS correction (noisy, 1 Hz)
  5. Correlative scan matching against SLAM occupancy grid

The PoseEstimator produces (est_x, est_y, est_yaw) which the autonomy stack
uses instead of truth pose when pose estimation is enabled (E key toggle).
"""

import math
import random
import numpy as np
from typing import Tuple, Optional


# ── Tunable noise parameters ──────────────────────────────────────────────

# IMU yaw noise
IMU_YAW_WHITE_NOISE_STD = 0.005     # rad per step
IMU_BIAS_DRIFT_RATE = 0.0002        # rad/s random walk

# Optical flow noise
OPTICAL_FLOW_VEL_NOISE_STD = 0.05   # m/s white noise
OPTICAL_FLOW_SCALE_DRIFT = 0.001    # scale drift per second
OPTICAL_FLOW_DROPOUT_CHANCE = 0.02  # 2% chance per step
OPTICAL_FLOW_DROPOUT_DURATION = 0.1 # seconds

# GPS noise
GPS_POSITION_NOISE_STD = 2.0        # meters
GPS_UPDATE_INTERVAL = 1.0           # seconds (1 Hz)
GPS_FUSION_WEIGHT = 0.15            # how much GPS pulls the estimate

# Scan matching
SCAN_MATCH_GAIN = 0.8               # correction scaling (0=ignore, 1=full)
SCAN_MATCH_MIN_CELLS = 30           # minimum SLAM cells before matching
SCAN_MATCH_SEARCH_RANGE = 5         # grid cells in each direction (11x11)
SCAN_MATCH_YAW_STEPS = 5            # yaw offsets to try
SCAN_MATCH_YAW_RANGE = 0.1          # rad range (+/-)
SCAN_MATCH_MIN_RAYS = 10            # minimum valid LiDAR rays for matching
SCAN_MATCH_OCCUPIED_THRESH = 70     # occupancy value to count as wall


class PoseEstimator:
    """Estimates drone pose from noisy simulated sensors.

    Usage:
        estimator = PoseEstimator(drone_id=0)
        estimator.initialize(start_x, start_y, start_yaw)

        # Each frame:
        estimator.update(dt, truth_pos, truth_ori, truth_vel, lidar_data, grid_map)
        est_pos = estimator.get_position()   # (est_x, est_y)
        est_yaw = estimator.get_yaw()        # radians
    """

    def __init__(self, drone_id: int = 0):
        self.drone_id = drone_id

        # Noise scaling: 1.0 = real-world, 0.1 = 1/10th noise for testing
        self._noise_scale: float = 1.0

        # Estimated state
        self.est_x: float = 0.0
        self.est_y: float = 0.0
        self.est_yaw: float = 0.0

        # IMU bias state (random walk)
        self._imu_bias: float = 0.0

        # Optical flow scale drift
        self._flow_scale: float = 1.0
        self._flow_dropout_timer: float = 0.0

        # GPS timing
        self._gps_timer: float = 0.0
        self._gps_enabled: bool = False

        # Frame counter for logging
        self._frame_count: int = 0

    def initialize(self, x: float, y: float, yaw: float):
        """Set initial pose (called at mission start)."""
        self.est_x = x
        self.est_y = y
        self.est_yaw = yaw
        self._imu_bias = 0.0
        self._flow_scale = 1.0
        self._flow_dropout_timer = 0.0
        self._gps_timer = 0.0
        self._frame_count = 0

    def reset(self):
        """Full reset to zero state."""
        self.initialize(0.0, 0.0, 0.0)

    def set_gps_enabled(self, enabled: bool):
        """Toggle GPS correction."""
        self._gps_enabled = enabled

    def set_noise_scale(self, scale: float):
        """Set noise scaling factor. 1.0 = real-world, 0.1 = 1/10th noise."""
        self._noise_scale = max(0.0, scale)

    def get_position(self) -> Tuple[float, float]:
        """Return current estimated position."""
        return (self.est_x, self.est_y)

    def get_yaw(self) -> float:
        """Return current estimated yaw."""
        return self.est_yaw

    def get_error(self, truth_pos: Tuple[float, float], truth_yaw: float) -> Tuple[float, float]:
        """Return (position_error_m, yaw_error_rad) vs truth."""
        pos_err = math.hypot(self.est_x - truth_pos[0], self.est_y - truth_pos[1])
        yaw_err = abs(_wrap_angle(self.est_yaw - truth_yaw))
        return (pos_err, yaw_err)

    def update(self, dt: float, truth_pos, truth_ori: float,
               truth_vel, lidar_data: Optional[dict],
               grid_map=None):
        """Run one estimation step.

        Args:
            dt: Time step in seconds
            truth_pos: True position (x, y, z) or (x, y) — used to generate
                       noisy sensor readings (NOT used directly for estimation)
            truth_ori: True orientation in radians — generates noisy IMU yaw
            truth_vel: True velocity (vx, vy, vz) or (vx, vy) — generates noisy flow
            lidar_data: LiDAR scan dict from environment (for scan matching)
            grid_map: SLAM GridMap object (for scan matching)
        """
        if dt <= 0:
            return

        self._frame_count += 1

        # Extract 2D truth values for sensor simulation
        tx = float(truth_pos[0])
        ty = float(truth_pos[1])
        tvx = float(truth_vel[0]) if len(truth_vel) > 0 else 0.0
        tvy = float(truth_vel[1]) if len(truth_vel) > 1 else 0.0

        # ── 1. Noisy IMU yaw ──────────────────────────────────────────
        noisy_yaw = self._generate_noisy_imu_yaw(truth_ori, dt)

        # ── 2. Noisy optical flow (body-frame velocity) ───────────────
        body_vx, body_vy = self._generate_noisy_optical_flow(
            tvx, tvy, truth_ori, dt)

        # ── 3. Dead reckoning: rotate body velocity by IMU yaw, integrate
        world_vx = body_vx * math.cos(noisy_yaw) - body_vy * math.sin(noisy_yaw)
        world_vy = body_vx * math.sin(noisy_yaw) + body_vy * math.cos(noisy_yaw)

        self.est_x += world_vx * dt
        self.est_y += world_vy * dt
        self.est_yaw = noisy_yaw

        # ── 4. GPS correction (if enabled) ────────────────────────────
        if self._gps_enabled:
            self._gps_timer += dt
            if self._gps_timer >= GPS_UPDATE_INTERVAL:
                self._gps_timer -= GPS_UPDATE_INTERVAL
                self._apply_gps_correction(tx, ty)

        # ── 5. Scan matching correction ───────────────────────────────
        # NOTE: Scan matching against a self-built map creates a positive
        # feedback loop (shifted estimate → map updated at wrong pos →
        # scan match finds another wrong offset → repeat). Only enable
        # if grid_map comes from an external/pre-built source.
        if grid_map is not None and lidar_data is not None:
            self._apply_scan_matching(lidar_data, grid_map)

    # ── Internal: Noisy IMU yaw generation ─────────────────────────────

    def _generate_noisy_imu_yaw(self, truth_ori: float, dt: float) -> float:
        """Generate noisy yaw from truth orientation + bias drift + white noise."""
        ns = self._noise_scale

        # Bias random walk (scaled)
        self._imu_bias += random.gauss(0, IMU_BIAS_DRIFT_RATE * dt * ns)

        # White noise (scaled)
        noise = random.gauss(0, IMU_YAW_WHITE_NOISE_STD * ns)

        # Combine: start from current estimate, apply truth delta + noise
        truth_delta = _wrap_angle(truth_ori - self.est_yaw)
        noisy_yaw = self.est_yaw + truth_delta + noise + self._imu_bias
        return _wrap_angle(noisy_yaw)

    # ── Internal: Noisy optical flow generation ────────────────────────

    def _generate_noisy_optical_flow(self, truth_vx: float, truth_vy: float,
                                      truth_ori: float, dt: float
                                      ) -> Tuple[float, float]:
        """Generate noisy body-frame velocity from truth world velocity.

        Returns (body_vx_forward, body_vy_right) with noise applied.
        """
        ns = self._noise_scale

        # Rotate world velocity to body frame
        cos_ori = math.cos(truth_ori)
        sin_ori = math.sin(truth_ori)
        body_fwd = truth_vx * cos_ori + truth_vy * sin_ori
        body_right = -truth_vx * sin_ori + truth_vy * cos_ori

        # Scale drift (slow multiplicative error, scaled)
        self._flow_scale += random.gauss(0, OPTICAL_FLOW_SCALE_DRIFT * dt * ns)
        self._flow_scale = max(0.9, min(1.1, self._flow_scale))  # clamp

        # Dropout handling (dropout chance scaled)
        if self._flow_dropout_timer > 0:
            self._flow_dropout_timer -= dt
            return (0.0, 0.0)  # No flow data during dropout

        if random.random() < OPTICAL_FLOW_DROPOUT_CHANCE * ns:
            self._flow_dropout_timer = OPTICAL_FLOW_DROPOUT_DURATION
            return (0.0, 0.0)

        # Apply scale drift + white noise (velocity noise scaled)
        noisy_fwd = body_fwd * self._flow_scale + random.gauss(0, OPTICAL_FLOW_VEL_NOISE_STD * ns)
        noisy_right = body_right * self._flow_scale + random.gauss(0, OPTICAL_FLOW_VEL_NOISE_STD * ns)

        return (noisy_fwd, noisy_right)

    # ── Internal: GPS correction ───────────────────────────────────────

    def _apply_gps_correction(self, truth_x: float, truth_y: float):
        """Pull estimate toward noisy GPS reading."""
        gps_x = truth_x + random.gauss(0, GPS_POSITION_NOISE_STD)
        gps_y = truth_y + random.gauss(0, GPS_POSITION_NOISE_STD)

        # Weighted fusion: pull toward GPS
        self.est_x += GPS_FUSION_WEIGHT * (gps_x - self.est_x)
        self.est_y += GPS_FUSION_WEIGHT * (gps_y - self.est_y)

    # ── Internal: Correlative scan matching ────────────────────────────

    def _apply_scan_matching(self, lidar_data: dict, grid_map):
        """Correlative scan matching against SLAM occupancy grid.

        Searches a 7x7x5 grid of (dx, dy, dyaw) offsets. For each candidate,
        recomputes LiDAR endpoints and counts how many land on occupied cells.
        Best match provides a correction to the estimated pose.
        """
        if not isinstance(lidar_data, dict):
            return

        ranges = lidar_data.get("ranges", [])
        start_angle = lidar_data.get("start_angle", 0)
        angle_step = lidar_data.get("angle_step", 0)
        max_range = lidar_data.get("max_range", 9.0)

        if not hasattr(grid_map, 'grid') or not hasattr(grid_map, 'world_to_grid'):
            return

        # Check minimum map data
        explored = getattr(grid_map, 'explored_cells', set())
        if len(explored) < SCAN_MATCH_MIN_CELLS:
            return

        # Collect valid rays (those that hit something, not max range)
        valid_rays = []
        for i, r in enumerate(ranges):
            if 0.1 < r < max_range * 0.95:
                ray_angle = start_angle + i * angle_step
                valid_rays.append((r, ray_angle))

        if len(valid_rays) < SCAN_MATCH_MIN_RAYS:
            return

        # Subsample rays for performance (use up to 30)
        if len(valid_rays) > 30:
            step = len(valid_rays) // 30
            valid_rays = valid_rays[::step][:30]

        resolution = grid_map.resolution
        grid = grid_map.grid
        grid_h, grid_w = grid.shape

        # Generate candidate offsets
        best_score = -1
        best_dx = 0.0
        best_dy = 0.0
        best_dyaw = 0.0

        search_range = SCAN_MATCH_SEARCH_RANGE
        yaw_steps = SCAN_MATCH_YAW_STEPS
        yaw_range = SCAN_MATCH_YAW_RANGE

        for dx_cells in range(-search_range, search_range + 1):
            dx = dx_cells * resolution
            for dy_cells in range(-search_range, search_range + 1):
                dy = dy_cells * resolution
                for yi in range(yaw_steps):
                    dyaw = -yaw_range + (2 * yaw_range * yi / max(1, yaw_steps - 1))

                    # Score: count endpoints landing on occupied cells
                    score = 0
                    candidate_x = self.est_x + dx
                    candidate_y = self.est_y + dy
                    candidate_yaw = self.est_yaw + dyaw

                    for r, body_angle in valid_rays:
                        # Body-relative angle → world angle using candidate yaw
                        world_angle = candidate_yaw + (body_angle - self.est_yaw)
                        ex = candidate_x + r * math.cos(world_angle)
                        ey = candidate_y + r * math.sin(world_angle)

                        gx, gy = grid_map.world_to_grid(ex, ey)
                        if 0 <= gx < grid_w and 0 <= gy < grid_h:
                            if grid[gy, gx] >= SCAN_MATCH_OCCUPIED_THRESH:
                                score += 1

                    if score > best_score:
                        best_score = score
                        best_dx = dx
                        best_dy = dy
                        best_dyaw = dyaw

        # Apply correction scaled by gain (only if meaningful match)
        min_match_ratio = 0.3
        if best_score > len(valid_rays) * min_match_ratio:
            self.est_x += best_dx * SCAN_MATCH_GAIN
            self.est_y += best_dy * SCAN_MATCH_GAIN
            self.est_yaw = _wrap_angle(self.est_yaw + best_dyaw * SCAN_MATCH_GAIN)


def _wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
