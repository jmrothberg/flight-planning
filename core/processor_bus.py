"""
Inter-Processor Communication Bus Definitions

Defines typed dataclasses for every data channel between the 3 processors.
In simulation these are pass-through (direct Python attribute access).
On real hardware each becomes a serialized SPI/UART message with CRC.

Bus topology:
  STM32H7 (flight controller) <--SPI--> STM32N6 (neural processor)
  STM32N6 (neural processor)  <--SPI--> STM32WL (mesh radio)
  STM32H7 (flight controller) <--UART-> STM32WL (mesh radio)
"""

from dataclasses import dataclass, field
from typing import Dict, Set, Tuple, List, Optional


# ── H7 → N6: Position + state from flight controller ────────────────────

@dataclass
class PositionUpdate:
    """H7 -> N6 @ 100 Hz via SPI.
    Flight controller sends current position, orientation, and battery state
    so the neural processor can run SLAM and search without IMU access."""
    x: float
    y: float
    z: float
    orientation: float      # radians
    battery_pct: float      # 0.0 - 100.0
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    velocity_z: float = 0.0


# ── N6 → H7: Navigation commands from search algorithm ──────────────────

@dataclass
class WaypointCommand:
    """N6 -> H7 @ 10 Hz via SPI.
    Neural processor sends next waypoint for the flight controller to navigate to.
    Flight controller handles PID loops and motor control to reach the waypoint."""
    target_x: float
    target_y: float
    target_z: float
    speed: float = 2.0      # m/s requested speed
    is_emergency: bool = False


# ── N6 → WL: Local map data for gossip broadcast ────────────────────────

@dataclass
class LocalMapData:
    """N6 -> WL via SPI (on-demand, ~1 Hz).
    Neural processor sends its discovered map data so the radio can
    broadcast it to other drones via gossip protocol."""
    searched_cells: Set[Tuple[int, int]] = field(default_factory=set)
    free_cells: Set[Tuple[int, int]] = field(default_factory=set)
    wall_cells: Set[Tuple[int, int]] = field(default_factory=set)
    features: List[Dict] = field(default_factory=list)


# ── WL → N6: Remote map data received from other drones ─────────────────

@dataclass
class RemoteMapData:
    """WL -> N6 via SPI (on receive).
    Radio processor forwards gossip payloads from other drones so the
    neural processor can merge them into its search algorithm."""
    source_drone_id: int
    payload: Dict = field(default_factory=dict)


# ── H7/N6 → WL: Position beacon for mesh network ───────────────────────

@dataclass
class PositionBeacon:
    """H7 -> WL @ 1 Hz via UART.
    Broadcast own position so other drones can compute spatial penalties
    and avoid exploring the same areas."""
    drone_id: int
    x: float
    y: float


# ── Simulation Bus (pass-through in sim, SPI/UART on hardware) ──────────

class SimulationBus:
    """In-simulation inter-processor bus. Holds latest value per channel.
    On real hardware, each channel maps to a specific SPI/UART peripheral
    with DMA transfer and CRC validation."""

    def __init__(self):
        self.position_update: Optional[PositionUpdate] = None
        self.waypoint_command: Optional[WaypointCommand] = None
        self.local_map_data: Optional[LocalMapData] = None
        self.remote_map_data: Optional[RemoteMapData] = None
        self.position_beacon: Optional[PositionBeacon] = None
