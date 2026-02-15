"""
STM32H7 — Flight Controller Main Loop
Hardware: STM32H755 dual-core Cortex-M7 (480 MHz) / M4 (240 MHz)

This is the real-time flight controller. It runs the fastest loop (~400 Hz)
and is responsible for keeping the drone in the air and moving safely.

RESPONSIBILITIES:
  - IMU sensor fusion (accelerometer + gyroscope → orientation estimate)
  - PID stabilization loops (roll, pitch, yaw, altitude)
  - Motor ESC PWM output (4 motors)
  - GPS/barometer position estimate
  - Battery voltage monitoring + low-battery failsafe
  - RC receiver failsafe (return-to-launch if signal lost)
  - Wall collision avoidance (from LiDAR proximity via N6)
  - Waypoint following (receive target from N6, compute motor commands)

INTER-PROCESSOR COMMUNICATION:
  H7 → N6:  PositionUpdate @ 100 Hz via SPI  (x, y, z, orientation, battery)
  N6 → H7:  WaypointCommand @ 10 Hz via SPI  (target_x, target_y, target_z)
  H7 → WL:  PositionBeacon @ 1 Hz via UART   (drone_id, x, y)

MEMORY BUDGET: 1 MB RAM, 2 MB Flash
  - No dynamic allocation after init
  - Fixed-size buffers for sensor history
  - PID state: 4 controllers × 3 floats = 48 bytes
  - Position/velocity state: 24 floats = 96 bytes
"""

import time
import math
from dataclasses import dataclass
from typing import Tuple, Optional

from core.stm32h7.drone import Drone, FlightController, PIDController
from core.processor_bus import PositionUpdate, WaypointCommand, PositionBeacon


# ── Hardware Abstraction Layer (HAL) ─────────────────────────────────────
# On real hardware, these read from actual peripherals via DMA.
# In simulation, they're filled by the simulation engine.

@dataclass
class IMUReading:
    """Raw IMU data from MPU6050/ICM-42688."""
    accel_x: float  # m/s² (body frame)
    accel_y: float
    accel_z: float
    gyro_x: float   # rad/s (body frame)
    gyro_y: float
    gyro_z: float
    timestamp_us: int


@dataclass
class BarometerReading:
    """Altitude from BMP390 barometer."""
    altitude_m: float
    pressure_pa: float
    temperature_c: float


@dataclass
class GPSReading:
    """Position from u-blox NEO-M9N GPS (outdoor only)."""
    latitude: float
    longitude: float
    altitude_m: float
    hdop: float       # horizontal dilution of precision
    fix_type: int     # 0=none, 2=2D, 3=3D
    num_satellites: int


@dataclass
class BatteryReading:
    """Battery state from ADC voltage divider + current sensor."""
    voltage_v: float      # cell voltage (e.g., 3.7V nominal)
    current_a: float      # instantaneous current draw
    remaining_pct: float  # estimated remaining capacity
    cell_count: int       # number of cells in series


# ── Flight Controller Application ────────────────────────────────────────

class FlightControllerApp:
    """
    Main flight controller application running on STM32H7.

    This is the innermost control loop. It must NEVER block or allocate
    memory dynamically. All sensor reads and motor writes happen via DMA.
    The PID loop runs at 400 Hz on the M7 core. Telemetry and position
    updates to other processors run at lower rates on the M4 core.
    """

    # Loop rates (Hz)
    PID_RATE = 400          # Inner stabilization loop (M7 core)
    POSITION_RATE = 100     # Position update to N6 via SPI
    BEACON_RATE = 1         # Position beacon to WL via UART
    BATTERY_CHECK_RATE = 10 # Battery monitoring

    # Safety thresholds
    BATTERY_CRITICAL_PCT = 5.0     # Emergency land immediately
    BATTERY_LOW_PCT = 20.0         # Return to home
    MAX_TILT_DEG = 45.0            # Max allowed tilt before emergency level
    MAX_ALTITUDE_M = 10.0
    MIN_ALTITUDE_M = 0.3
    WALL_PROXIMITY_M = 0.2         # Collision avoidance pushback radius

    def __init__(self, drone_id: int = 0):
        self.drone_id = drone_id

        # Flight controller hardware
        self.drone = Drone()
        self.flight_controller = FlightController()

        # Current state (updated from sensors)
        self.position = (0.0, 0.0, 0.0)     # x, y, z in meters
        self.velocity = (0.0, 0.0, 0.0)     # vx, vy, vz in m/s
        self.orientation = 0.0                # yaw in radians
        self.battery_pct = 100.0

        # Incoming commands from N6 (via SPI)
        self._waypoint_cmd: Optional[WaypointCommand] = None

        # Outgoing data to other processors
        self._position_update = PositionUpdate(0, 0, 0, 0, 100.0)
        self._position_beacon = PositionBeacon(drone_id, 0, 0)

        # Timing (in simulation, these are simulated ticks)
        self._last_position_send = 0.0
        self._last_beacon_send = 0.0
        self._last_battery_check = 0.0

        # Safety state
        self.armed = False
        self.failsafe_active = False
        self.return_to_launch = False
        self.emergency_land = False

    # ── Sensor Reads (HAL) ───────────────────────────────────────────

    def read_imu(self) -> IMUReading:
        """Read IMU via SPI/I2C DMA. On real HW: non-blocking DMA read.
        In simulation: derived from drone.velocity and drone.orientation."""
        return IMUReading(
            accel_x=0.0, accel_y=0.0, accel_z=-9.81,
            gyro_x=0.0, gyro_y=0.0, gyro_z=self.drone.angular_velocity,
            timestamp_us=int(time.time() * 1e6),
        )

    def read_barometer(self) -> BarometerReading:
        """Read barometer via I2C. ~25 Hz update rate."""
        return BarometerReading(
            altitude_m=float(self.drone.position[2]),
            pressure_pa=101325.0,
            temperature_c=25.0,
        )

    def read_gps(self) -> GPSReading:
        """Read GPS via UART. ~10 Hz update rate. Indoor: no fix."""
        return GPSReading(
            latitude=0.0, longitude=0.0,
            altitude_m=float(self.drone.position[2]),
            hdop=99.9, fix_type=0, num_satellites=0,
        )

    def read_battery(self) -> BatteryReading:
        """Read battery ADC. On real HW: voltage divider + INA219 current sensor."""
        return BatteryReading(
            voltage_v=3.7 * (self.drone.battery_level / 100.0),
            current_a=5.0,
            remaining_pct=self.drone.battery_level,
            cell_count=4,
        )

    # ── Motor Output (HAL) ───────────────────────────────────────────

    def write_motors(self, m1: float, m2: float, m3: float, m4: float):
        """Write PWM to 4 ESCs via timer DMA. Values 0.0-1.0.
        On real HW: TIM1 CH1-4 with DShot600 protocol."""
        # In simulation, motor commands are implicit via drone.velocity
        pass

    # ── Inter-Processor Communication ────────────────────────────────

    def receive_waypoint(self) -> Optional[WaypointCommand]:
        """Read waypoint command from N6 via SPI slave DMA buffer.
        Non-blocking: returns None if no new command available.
        On real HW: SPI2 in slave mode, DMA circular buffer, CRC-8."""
        cmd = self._waypoint_cmd
        self._waypoint_cmd = None  # Consume
        return cmd

    def send_position_update(self) -> PositionUpdate:
        """Send position update to N6 via SPI master DMA.
        On real HW: SPI1 master, 16-byte frame, CRC-8, @ 100 Hz."""
        self._position_update = PositionUpdate(
            x=self.position[0],
            y=self.position[1],
            z=self.position[2],
            orientation=self.orientation,
            battery_pct=self.battery_pct,
            velocity_x=self.velocity[0],
            velocity_y=self.velocity[1],
            velocity_z=self.velocity[2],
        )
        return self._position_update

    def send_position_beacon(self) -> PositionBeacon:
        """Send position beacon to WL via UART DMA.
        On real HW: UART4, 8-byte frame, CRC-8, @ 1 Hz."""
        self._position_beacon = PositionBeacon(
            drone_id=self.drone_id,
            x=self.position[0],
            y=self.position[1],
        )
        return self._position_beacon

    # ── Core Flight Logic ────────────────────────────────────────────

    def init(self):
        """One-time initialization after power-on.
        On real HW: configure clocks, DMA channels, SPI/UART/I2C peripherals,
        calibrate IMU gyro bias, wait for barometer settle."""
        self.armed = False
        self.failsafe_active = False
        print(f"[H7-{self.drone_id}] Flight controller initialized")

    def update(self, dt: float):
        """
        Main flight controller tick. Called at PID_RATE (400 Hz on real HW).

        On the real STM32H7 this runs in the TIM6 interrupt handler on the
        M7 core. The M4 core handles telemetry and SPI communication with
        N6/WL at lower rates.

        Args:
            dt: Time step in seconds (1/400 on real HW, variable in sim)
        """
        now = time.time()

        # ── 1. Read sensors ──────────────────────────────────────────
        imu = self.read_imu()
        baro = self.read_barometer()
        battery = self.read_battery()

        # ── 2. State estimation (sensor fusion) ──────────────────────
        # On real HW: complementary filter or Madgwick AHRS on IMU
        # In simulation: use drone state directly
        self.position = (
            float(self.drone.position[0]),
            float(self.drone.position[1]),
            float(self.drone.position[2]),
        )
        self.velocity = (
            float(self.drone.velocity[0]),
            float(self.drone.velocity[1]),
            float(self.drone.velocity[2]),
        )
        self.orientation = float(self.drone.orientation)
        self.battery_pct = battery.remaining_pct

        # ── 3. Safety checks ────────────────────────────────────────
        if self.battery_pct < self.BATTERY_CRITICAL_PCT:
            self.emergency_land = True
        elif self.battery_pct < self.BATTERY_LOW_PCT:
            self.return_to_launch = True

        if self.emergency_land:
            self.drone.emergency_stop()
            self.drone.mission_complete = True
            return

        # ── 4. Process waypoint command from N6 ─────────────────────
        cmd = self.receive_waypoint()
        if cmd is not None:
            if cmd.is_emergency:
                self.drone.emergency_stop()
            else:
                target = (cmd.target_x, cmd.target_y, cmd.target_z)
                self.drone.navigate_to(target)

        # ── 5. PID control + motor mixing ────────────────────────────
        if self.drone.current_waypoint is not None:
            desired_yaw = math.atan2(
                self.drone.current_waypoint[1] - self.drone.position[1],
                self.drone.current_waypoint[0] - self.drone.position[0],
            )
            motors = self.flight_controller.compute_motor_commands(
                self.drone, self.drone.current_waypoint, desired_yaw
            )
            self.write_motors(*motors)

        # ── 6. Update drone physics ─────────────────────────────────
        # On real HW: this is implicit (motors spin, physics is real)
        # In simulation: drone.update() applies velocity and battery drain
        self.drone.update(dt)

        # ── 7. Send position to N6 (100 Hz) ─────────────────────────
        if now - self._last_position_send >= 1.0 / self.POSITION_RATE:
            self.send_position_update()
            self._last_position_send = now

        # ── 8. Send beacon to WL (1 Hz) ─────────────────────────────
        if now - self._last_beacon_send >= 1.0 / self.BEACON_RATE:
            self.send_position_beacon()
            self._last_beacon_send = now

    def arm(self):
        """Arm motors. On real HW: enable ESC output, play arming tone."""
        self.armed = True
        print(f"[H7-{self.drone_id}] Motors armed")

    def disarm(self):
        """Disarm motors. On real HW: disable ESC output."""
        self.armed = False
        self.drone.emergency_stop()
        print(f"[H7-{self.drone_id}] Motors disarmed")


# ── Entry Point (on real HW: Reset_Handler → main) ──────────────────────

def main(drone_id: int = 0):
    """
    STM32H7 main entry point.

    On real hardware this is called from the startup assembly after
    SystemClock_Config(), HAL_Init(), and peripheral initialization.
    The main loop never returns.
    """
    app = FlightControllerApp(drone_id=drone_id)
    app.init()
    app.arm()

    print(f"[H7-{drone_id}] Flight controller running — PID @ {app.PID_RATE} Hz")
    print(f"[H7-{drone_id}] Position → N6 @ {app.POSITION_RATE} Hz via SPI")
    print(f"[H7-{drone_id}] Beacon → WL @ {app.BEACON_RATE} Hz via UART")

    # On real HW: this is while(1) in bare-metal or RTOS task loop
    # In simulation: driven by simulation_main.py frame loop
    dt = 1.0 / app.PID_RATE

    try:
        while True:
            app.update(dt)
            time.sleep(dt)  # In real HW: timer interrupt, not sleep
    except KeyboardInterrupt:
        app.disarm()
        print(f"[H7-{drone_id}] Shutdown")


if __name__ == "__main__":
    main()
