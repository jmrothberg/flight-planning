"""
STM32WL — Mesh Radio Processor Main Loop
Hardware: STM32WL55 Cortex-M4 + integrated sub-GHz radio @ 48 MHz

This is the communication processor. It manages the mesh radio network,
broadcasts position beacons, and runs the gossip protocol for sharing
map data between drones. It has the slowest clock but the lowest power
draw, and is the only processor with direct RF hardware access.

RESPONSIBILITIES:
  - Sub-GHz radio management (LoRa or FSK modulation, 900 MHz ISM band)
  - HELLO beacon broadcast (1 Hz, announces drone presence + position)
  - Neighbor discovery (track which drones are within comm range)
  - Gossip protocol: 3-phase map sharing
      Phase 1: Receive local map from N6 (searched/free/wall cells)
      Phase 2: Exchange gossip payloads with neighbors (bidirectional merge)
      Phase 3: Send merged global knowledge back to N6
  - Message fragmentation (200-byte max radio frame)
  - AODV routing for multi-hop relay (when drones are > 1 hop apart)
  - IED alert broadcast (high-priority, all-drones notification)
  - Position beacon relay (for spatial separation penalty computation)

INTER-PROCESSOR COMMUNICATION:
  N6 → WL:  LocalMapData @ 1 Hz via SPI  (searched/free/wall cells, features)
  WL → N6:  RemoteMapData on-receive      (gossip payload from other drones)
  H7 → WL:  PositionBeacon @ 1 Hz via UART (drone_id, x, y)

RADIO PARAMETERS:
  - Frequency: 915 MHz (US ISM) or 868 MHz (EU)
  - Modulation: LoRa SF7/BW125 (5.5 kbps) for beacons, FSK 50 kbps for data
  - TX power: +14 dBm (25 mW)
  - Range: ~100m indoor (with walls), ~1 km outdoor line-of-sight
  - Duty cycle: 10% max (regulatory compliance)

MEMORY BUDGET: 256 KB RAM, 256 KB Flash
  - Gossip map state: ~20 KB (cell sets, compressed)
  - Message queue: 16 messages × 200 bytes = 3.2 KB
  - Routing table: 12 entries × 16 bytes = 192 bytes
  - Neighbor table: 12 entries × 24 bytes = 288 bytes
  - Radio TX/RX buffers: 2 × 256 bytes = 512 bytes
  - TOTAL: < 30 KB active (fits easily in 256 KB)
"""

import time
import math
from dataclasses import dataclass, field
from typing import Tuple, Optional, Set, Dict, List

from core.stm32wl.mesh_network import MeshNode, SimulatedMeshProtocol, MeshMessage
from core.stm32wl.gossip_map import GossipMap, MapFeature
from core.stm32wl.communication import CommSystem, CommProtocol
from core.processor_bus import (
    LocalMapData, RemoteMapData, PositionBeacon,
)


# ── Hardware Abstraction Layer (HAL) ─────────────────────────────────────

@dataclass
class RadioConfig:
    """Sub-GHz radio configuration for STM32WL integrated transceiver."""
    frequency_mhz: float = 915.0     # ISM band center frequency
    tx_power_dbm: int = 14           # Transmit power (+14 dBm = 25 mW)
    bandwidth_khz: float = 125.0     # LoRa bandwidth
    spreading_factor: int = 7        # LoRa SF (7-12, lower = faster)
    coding_rate: int = 5             # LoRa CR (5=4/5, 8=4/8)
    preamble_length: int = 8         # LoRa preamble symbols
    sync_word: int = 0x34            # Network sync word (private network)
    max_payload_bytes: int = 200     # Max radio frame payload


@dataclass
class RadioStats:
    """Runtime radio statistics for diagnostics."""
    tx_packets: int = 0
    rx_packets: int = 0
    tx_bytes: int = 0
    rx_bytes: int = 0
    crc_errors: int = 0
    tx_duty_cycle_pct: float = 0.0   # Current duty cycle usage
    rssi_last_dbm: float = -120.0    # Last received signal strength
    snr_last_db: float = 0.0         # Last signal-to-noise ratio


# ── Mesh Radio Application ───────────────────────────────────────────────

class MeshRadioApp:
    """
    Main mesh radio application running on STM32WL.

    This processor has the simplest logic but the most critical timing
    constraints. Radio TX/RX windows must be precisely scheduled to avoid
    collisions. The gossip protocol runs between radio slots.

    The STM32WL's integrated sub-GHz radio (no external transceiver)
    handles modulation, CRC, and packet framing in hardware. The M4 core
    manages protocol logic, gossip merging, and inter-processor SPI.

    Timing budget per gossip cycle (100 ms):
      - Radio TX: ~20 ms (200 bytes @ LoRa SF7)
      - Radio RX: ~30 ms (listen window)
      - Gossip merge: ~5 ms (set union operations)
      - SPI to N6: ~2 ms (local map transfer)
      - Idle/sleep: ~43 ms (power saving)
    """

    # Loop rates (Hz)
    MAIN_RATE = 10           # Main gossip/radio loop
    BEACON_RATE = 1          # HELLO beacon broadcast
    GOSSIP_RATE = 10         # Gossip sync attempts (with neighbors in range)
    NEIGHBOR_EXPIRE_S = 5.0  # Expire neighbors not heard from in 5s

    # Radio constraints
    MAX_FRAGMENT_SIZE = 200  # bytes per radio frame
    DUTY_CYCLE_MAX = 0.10   # 10% regulatory limit
    MAX_HOPS = 4            # Max relay hops for multi-hop routing

    def __init__(self, drone_id: int = 0, comm_range: float = 10.0):
        self.drone_id = drone_id
        self.comm_range = comm_range

        # Radio config
        self.radio_config = RadioConfig()
        self.radio_stats = RadioStats()

        # Mesh networking (handles neighbor discovery + routing)
        # In simulation: uses shared memory bus
        # On real HW: uses radio TX/RX via HAL_SUBGHZ
        self._protocol: Optional[SimulatedMeshProtocol] = None
        self._mesh_node: Optional[MeshNode] = None

        # Gossip map (distributed map sharing)
        self.gossip_map = GossipMap(drone_id=drone_id)

        # Communication system (fragmentation, priority queue)
        self.comm = CommSystem()

        # Current position (received from H7 via UART)
        self._own_position: Tuple[float, float] = (0.0, 0.0)
        self._own_beacon = PositionBeacon(drone_id, 0, 0)

        # Local map data (received from N6 via SPI)
        self._local_map: Optional[LocalMapData] = None
        self._local_map_dirty = False  # True when new data from N6

        # Outgoing data to N6
        self._remote_map_queue: List[RemoteMapData] = []

        # Neighbor tracking
        self._neighbor_positions: Dict[int, Tuple[float, float]] = {}
        self._neighbor_last_seen: Dict[int, float] = {}

        # Timing
        self._last_beacon = 0.0
        self._last_gossip = 0.0

    # ── Radio HAL ────────────────────────────────────────────────────

    def radio_init(self):
        """Initialize sub-GHz radio hardware.
        On real HW: HAL_SUBGHZ_Init(), configure modulation, set frequency,
        calibrate image rejection, enter RX mode."""
        print(f"[WL-{self.drone_id}] Radio init: {self.radio_config.frequency_mhz} MHz, "
              f"+{self.radio_config.tx_power_dbm} dBm, "
              f"SF{self.radio_config.spreading_factor}")

    def radio_tx(self, data: bytes, dest_id: int = -1) -> bool:
        """Transmit data via radio.
        On real HW: HAL_SUBGHZ_Transmit() with DMA, blocks until TX complete.
        Returns False if duty cycle exceeded or channel busy (CSMA/CA).

        Args:
            data: Raw payload bytes (max MAX_FRAGMENT_SIZE)
            dest_id: Destination drone ID (-1 = broadcast)
        """
        if len(data) > self.MAX_FRAGMENT_SIZE:
            return False

        # Check duty cycle
        if self.radio_stats.tx_duty_cycle_pct >= self.DUTY_CYCLE_MAX * 100:
            return False

        self.radio_stats.tx_packets += 1
        self.radio_stats.tx_bytes += len(data)
        return True

    def radio_rx(self) -> Optional[bytes]:
        """Check for received radio data.
        On real HW: check DMA buffer, HAL_SUBGHZ_GetPayload() if IRQ fired.
        Non-blocking: returns None if no packet available."""
        self.radio_stats.rx_packets += 1  # Count check attempts
        return None  # Simulation uses message bus instead

    # ── Inter-Processor Communication ────────────────────────────────

    def receive_position_beacon(self, beacon: PositionBeacon):
        """Receive position from H7 via UART DMA.
        On real HW: LPUART1, 8-byte frame, DMA circular buffer.
        Called at 1 Hz."""
        self._own_position = (beacon.x, beacon.y)
        self._own_beacon = beacon

    def receive_local_map(self, local_map: LocalMapData):
        """Receive local map data from N6 via SPI slave DMA.
        On real HW: SPI1 slave mode, chunked transfer with flow control.
        Called at 1 Hz (N6 pushes when it has fresh data).

        This is Phase 1 of gossip: local search grid → gossip map."""
        self._local_map = local_map
        self._local_map_dirty = True

        # Phase 1: Sync local discoveries into gossip map
        self.gossip_map.add_local_searched(local_map.searched_cells)
        self.gossip_map.add_local_free(local_map.free_cells)
        self.gossip_map.add_local_walls(local_map.wall_cells)

    def get_pending_remote_maps(self) -> List[RemoteMapData]:
        """Get queued remote map updates to send to N6.
        On real HW: SPI1 master, triggered when gossip merge produces new data.
        N6 polls this or WL triggers SPI interrupt."""
        pending = self._remote_map_queue[:]
        self._remote_map_queue.clear()
        return pending

    def _queue_remote_for_n6(self, source_drone_id: int, payload: dict):
        """Queue a gossip merge result to send to N6."""
        self._remote_map_queue.append(
            RemoteMapData(source_drone_id=source_drone_id, payload=payload)
        )

    # ── Mesh Protocol ────────────────────────────────────────────────

    def init_mesh(self, protocol: SimulatedMeshProtocol):
        """Initialize mesh networking with a protocol instance.
        On real HW: mesh protocol runs directly on the radio.
        In simulation: uses SimulatedMeshProtocol with shared memory."""
        self._protocol = protocol
        self._mesh_node = MeshNode(drone_id=self.drone_id, protocol=protocol)

    # ── Core Radio Logic ─────────────────────────────────────────────

    def init(self):
        """One-time initialization after power-on.
        On real HW: configure SUBGHZ peripheral, calibrate PLL,
        set up DMA for SPI slave (N6) and LPUART (H7)."""
        self.radio_init()
        print(f"[WL-{self.drone_id}] Mesh radio initialized")
        print(f"[WL-{self.drone_id}]   Comm range: {self.comm_range}m")
        print(f"[WL-{self.drone_id}]   Max fragment: {self.MAX_FRAGMENT_SIZE} bytes")
        print(f"[WL-{self.drone_id}]   Max hops: {self.MAX_HOPS}")

    def update(self, dt: float):
        """
        Main mesh radio tick. Called at MAIN_RATE (10 Hz).

        On the real STM32WL this runs as the main loop, alternating between
        radio TX/RX windows and protocol processing. Power management uses
        STOP2 mode between radio events to minimize current draw.

        The gossip protocol runs in 3 phases each cycle:
          Phase 1: Ingest local map from N6 (already done in receive_local_map)
          Phase 2: Exchange gossip with neighbors in radio range
          Phase 3: Queue merged knowledge for N6

        Args:
            dt: Time step in seconds
        """
        now = time.time()

        # ── 1. Mesh update: send HELLO beacons, process incoming ─────
        if self._mesh_node is not None:
            messages = self._mesh_node.update(self._own_position)

            # Process any received mesh messages
            for msg in messages:
                if msg.msg_type == "MAP_UPDATE":
                    # Remote gossip data received via mesh
                    self.gossip_map.merge_remote_update(msg.payload, force=True)
                    self._queue_remote_for_n6(msg.source_id, msg.payload)
                elif msg.msg_type == "IED_ALERT":
                    # High-priority: relay IED alert to all neighbors + N6
                    self._queue_remote_for_n6(msg.source_id, msg.payload)
                elif msg.msg_type == "POSITION":
                    # Track neighbor position for spatial penalty
                    src = msg.source_id
                    if 'x' in msg.payload and 'y' in msg.payload:
                        self._neighbor_positions[src] = (
                            msg.payload['x'], msg.payload['y']
                        )
                        self._neighbor_last_seen[src] = now

        # ── 2. Expire stale neighbors ───────────────────────────────
        expired = [nid for nid, t in self._neighbor_last_seen.items()
                   if now - t > self.NEIGHBOR_EXPIRE_S]
        for nid in expired:
            self._neighbor_positions.pop(nid, None)
            self._neighbor_last_seen.pop(nid, None)

        # ── 3. Gossip Phase 2: exchange with neighbors ───────────────
        if now - self._last_gossip >= 1.0 / self.GOSSIP_RATE:
            self._last_gossip = now
            self._do_gossip_exchange()

        # ── 4. Gossip Phase 3: queue merged knowledge for N6 ────────
        # After gossip exchange, N6 needs updated global knowledge
        # (positions, global_searched, global_free, claimed targets)
        self._export_gossip_to_n6()

    def _do_gossip_exchange(self):
        """
        Phase 2: Bidirectional gossip sync with all neighbors in range.

        For each pair (self, neighbor) within comm_range:
          - Get our gossip payload
          - Send to neighbor (via mesh broadcast)
          - Receive neighbor's payload (already merged via mesh messages)

        On real HW: this translates to radio TX of our gossip payload,
        then listen for neighbor's response in the next RX window.
        The radio handles CSMA/CA collision avoidance.
        """
        if self._mesh_node is None:
            return

        # Broadcast our gossip payload to all neighbors
        neighbors = self._mesh_node.get_neighbors()
        if neighbors:
            payload = self.gossip_map.get_sync_payload()
            self._mesh_node.broadcast("MAP_UPDATE", payload)

            # Track radio stats
            self.radio_stats.tx_packets += 1

    def _export_gossip_to_n6(self):
        """
        Phase 3: Push merged global knowledge back to N6.

        After gossip exchange, our gossip map has merged data from
        all reachable drones. Package this as RemoteMapData for N6's
        search algorithm to use in target scoring.
        """
        # Build payload with everything N6 needs for search scoring
        payload = {
            'searched': list(self.gossip_map.get_global_searched()),
            'free': list(self.gossip_map.get_global_free()),
            'positions': list(self._neighbor_positions.values()),
        }
        # Only queue if we have actual multi-drone data
        if self._neighbor_positions:
            self._queue_remote_for_n6(self.drone_id, payload)

    def broadcast_ied_alert(self, ied_position: Tuple[float, float],
                            confidence: float):
        """
        Broadcast high-priority IED detection alert to all drones.

        On real HW: uses highest radio priority, bypasses normal queue,
        transmitted immediately with max TX power and retransmission.

        Args:
            ied_position: World coordinates of detected IED
            confidence: Detection confidence (0.0 - 1.0)
        """
        if self._mesh_node is None:
            return

        # Add to local gossip
        self.gossip_map.add_local_feature(
            "ied", ied_position, confidence
        )

        # Broadcast alert
        payload = {
            'type': 'ied_alert',
            'position': ied_position,
            'confidence': confidence,
            'drone_id': self.drone_id,
        }
        self._mesh_node.broadcast("IED_ALERT", payload)

        self.radio_stats.tx_packets += 1
        print(f"[WL-{self.drone_id}] IED ALERT broadcast: "
              f"pos=({ied_position[0]:.1f}, {ied_position[1]:.1f}), "
              f"conf={confidence:.0%}")

    def get_neighbor_count(self) -> int:
        """Get number of active neighbors."""
        return len(self._neighbor_positions)

    def get_radio_stats(self) -> RadioStats:
        """Get radio statistics for diagnostics."""
        return self.radio_stats

    def set_comm_range(self, new_range: float):
        """Update communication range (for testing/deployment tuning).
        On real HW: adjusts TX power and RX sensitivity threshold."""
        self.comm_range = max(2.0, min(100.0, new_range))
        if self._protocol is not None:
            self._protocol.update_comm_range(self.comm_range)
        print(f"[WL-{self.drone_id}] Comm range → {self.comm_range:.0f}m")


# ── Entry Point (on real HW: Reset_Handler → main) ──────────────────────

def main(drone_id: int = 0, comm_range: float = 10.0):
    """
    STM32WL main entry point.

    On real hardware this is called from startup after SUBGHZ peripheral
    init, PLL calibration, and SPI/UART DMA setup. The main loop
    alternates between radio TX/RX windows and protocol processing.
    Between events, the MCU enters STOP2 mode for power saving.
    """
    app = MeshRadioApp(drone_id=drone_id, comm_range=comm_range)
    app.init()

    print(f"[WL-{drone_id}] Mesh radio running — gossip @ {app.GOSSIP_RATE} Hz")
    print(f"[WL-{drone_id}] Beacon @ {app.BEACON_RATE} Hz")
    print(f"[WL-{drone_id}] Comm range: {comm_range}m")

    dt = 1.0 / app.MAIN_RATE

    try:
        while True:
            app.update(dt)
            time.sleep(dt)  # In real HW: STOP2 between radio events
    except KeyboardInterrupt:
        stats = app.get_radio_stats()
        print(f"[WL-{drone_id}] Shutdown — TX: {stats.tx_packets} pkts, "
              f"RX: {stats.rx_packets} pkts")


if __name__ == "__main__":
    main()
