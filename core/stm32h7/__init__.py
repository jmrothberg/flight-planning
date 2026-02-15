"""
STM32H7 — Flight Controller Processor

Hardware: STM32H755 dual-core Cortex-M7/M4 @ 480 MHz
Role: IMU sensor fusion, motor control (ESC PWM), GPS/barometer,
      PID stabilization loops, battery monitoring, RC failsafe.

Inter-processor buses:
  H7 -> N6: PositionUpdate (x, y, z, orientation, battery) via SPI @ 100 Hz
  N6 -> H7: WaypointCommand (target x, y, z) via SPI @ 10 Hz
  H7 -> WL: PositionBeacon (drone_id, x, y) via UART @ 1 Hz
"""

from core.stm32h7.drone import Drone, FlightController, PIDController

__all__ = ["Drone", "FlightController", "PIDController"]
