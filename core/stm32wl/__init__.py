"""
STM32WL — Mesh Radio Processor

Hardware: STM32WL55 Cortex-M4 + sub-GHz radio @ 48 MHz
Role: Mesh networking (AODV routing), gossip protocol for distributed
      map sharing, inter-drone position beacons, message fragmentation.

Inter-processor buses:
  N6 -> WL: LocalMapData (searched_cells, free_cells, wall_cells, features)
  WL -> N6: RemoteMapData (gossip payload from other drones)
  H7 -> WL: PositionBeacon (drone_id, x, y) via UART @ 1 Hz
"""

from core.stm32wl.mesh_network import MeshNode, SimulatedMeshProtocol, MeshMessage, MeshProtocolInterface
from core.stm32wl.gossip_map import GossipMap, MapFeature
from core.stm32wl.communication import (
    CommSystem, CommProtocol, DataCompressor, MessageFormatter,
    Message, MessagePriority,
)

__all__ = [
    "MeshNode", "SimulatedMeshProtocol", "MeshMessage", "MeshProtocolInterface",
    "GossipMap", "MapFeature",
    "CommSystem", "CommProtocol", "DataCompressor", "MessageFormatter",
    "Message", "MessagePriority",
]
