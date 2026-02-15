"""
STM32N6 — Neural Processing Unit

Hardware: STM32N657 Cortex-M55 + NPU @ 800 MHz
Role: LiDAR point cloud processing, camera inference (object/IED detection),
      SLAM mapping, A* pathfinding, search algorithm (SystematicMapper).

Inter-processor buses:
  H7 -> N6: PositionUpdate (x, y, z, orientation, battery) via SPI @ 100 Hz
  N6 -> H7: WaypointCommand (target x, y, z) via SPI @ 10 Hz
  N6 -> WL: LocalMapData (searched_cells, free_cells, wall_cells, features)
  WL -> N6: RemoteMapData (gossip payload from other drones)
"""

from core.stm32n6.search_systematic_mapper import SystematicMapper
from core.stm32n6.navigation import AStarPathfinder, RRTExplorer, PathPlanner, Node
from core.stm32n6.sensors import IEDSensor, IEDReading
from core.stm32n6.slam import SLAMSystem, GridMap, ParticleFilter, Particle
from core.stm32n6.vision import (
    VisionSystem, SimpleObjectDetector, AdvancedObjectDetector,
    ImageProcessor, Detection, ObjectType,
)
from core.stm32n6.minimap import MinimapSystem, DiscoveredFeature, FeatureType

__all__ = [
    "SystematicMapper",
    "AStarPathfinder", "RRTExplorer", "PathPlanner", "Node",
    "IEDSensor", "IEDReading",
    "SLAMSystem", "GridMap", "ParticleFilter", "Particle",
    "VisionSystem", "SimpleObjectDetector", "AdvancedObjectDetector",
    "ImageProcessor", "Detection", "ObjectType",
    "MinimapSystem", "DiscoveredFeature", "FeatureType",
]
