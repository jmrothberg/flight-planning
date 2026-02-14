"""
Gossip Protocol for Distributed Map Sharing
Drones exchange map data when in communication range.

Based on gossip protocols used in multi-robot SLAM systems:
- Each drone maintains local map + merges remote updates
- Version vectors prevent redundant sync
- Data propagates across network as drones come in/out of range
"""

from typing import Dict, Set, Tuple, List, Any, Optional
from dataclasses import dataclass, field
import time


@dataclass
class MapFeature:
    """A discovered map feature with attribution."""
    feature_type: str  # "wall", "door", "object", "ied"
    position: Tuple[float, float]
    confidence: float
    drone_id: int  # Which drone discovered this
    timestamp: float
    extra_data: dict = field(default_factory=dict)


class GossipMap:
    """
    Distributed map sharing via gossip protocol.
    Each drone maintains its own view of the shared map.
    """
    
    def __init__(self, drone_id: int):
        """
        Initialize gossip map for a drone.
        
        Args:
            drone_id: This drone's unique ID
        """
        self.drone_id = drone_id
        
        # Searched cells per drone (for color-coded visualization)
        # Key: drone_id, Value: set of (grid_x, grid_y) tuples
        self.searched_by_drone: Dict[int, Set[Tuple[int, int]]] = {}
        
        # Global union of all searched cells (for search coordination)
        self.global_searched: Set[Tuple[int, int]] = set()
        
        # Free cells known by each drone
        self.free_by_drone: Dict[int, Set[Tuple[int, int]]] = {}
        self.global_free: Set[Tuple[int, int]] = set()
        
        # Wall cells known by each drone
        self.walls_by_drone: Dict[int, Set[Tuple[int, int]]] = {}
        self.global_walls: Set[Tuple[int, int]] = set()
        
        # Discovered features (IEDs, objects, doors)
        self.features: List[MapFeature] = []
        
        # Version vector for efficient sync
        # Key: drone_id, Value: version number (increments on each local update)
        self.versions: Dict[int, int] = {}
        
        # Track what versions we've received from each drone
        self.received_versions: Dict[int, int] = {}
        
        # Timestamp of last sync with each drone
        self.last_sync: Dict[int, float] = {}
    
    def add_local_searched(self, cells: Set[Tuple[int, int]]):
        """
        Add cells searched by THIS drone.
        
        Args:
            cells: Set of (grid_x, grid_y) cells that were searched
        """
        if self.drone_id not in self.searched_by_drone:
            self.searched_by_drone[self.drone_id] = set()
        
        new_cells = cells - self.searched_by_drone[self.drone_id]
        if new_cells:
            self.searched_by_drone[self.drone_id].update(new_cells)
            self.global_searched.update(new_cells)
            self._increment_version()
    
    def add_local_free(self, cells: Set[Tuple[int, int]]):
        """Add free space cells discovered by THIS drone."""
        if self.drone_id not in self.free_by_drone:
            self.free_by_drone[self.drone_id] = set()
        
        new_cells = cells - self.free_by_drone[self.drone_id]
        if new_cells:
            self.free_by_drone[self.drone_id].update(new_cells)
            self.global_free.update(new_cells)
            self._increment_version()
    
    def add_local_walls(self, cells: Set[Tuple[int, int]]):
        """Add wall cells discovered by THIS drone."""
        if self.drone_id not in self.walls_by_drone:
            self.walls_by_drone[self.drone_id] = set()
        
        new_cells = cells - self.walls_by_drone[self.drone_id]
        if new_cells:
            self.walls_by_drone[self.drone_id].update(new_cells)
            self.global_walls.update(new_cells)
            self._increment_version()
    
    def add_local_feature(self, feature_type: str, position: Tuple[float, float],
                          confidence: float, extra_data: dict = None):
        """Add a discovered feature (IED, object, door)."""
        feature = MapFeature(
            feature_type=feature_type,
            position=position,
            confidence=confidence,
            drone_id=self.drone_id,
            timestamp=time.time(),
            extra_data=extra_data or {}
        )
        
        # Check for duplicate (same position and type)
        for existing in self.features:
            if (existing.feature_type == feature_type and
                abs(existing.position[0] - position[0]) < 1.0 and
                abs(existing.position[1] - position[1]) < 1.0):
                # Update confidence if higher
                if confidence > existing.confidence:
                    existing.confidence = confidence
                return
        
        self.features.append(feature)
        self._increment_version()
    
    def _increment_version(self):
        """Increment our version number."""
        self.versions[self.drone_id] = self.versions.get(self.drone_id, 0) + 1
    
    def get_sync_payload(self) -> dict:
        """
        Get data to share with other drones.
        Includes ALL known cells from ALL drones (full knowledge sharing).
        
        Returns:
            Dictionary with map data for transmission
        """
        # Share ALL searched cells organized by drone (full gossip)
        all_searched = {}
        for drone_id, cells in self.searched_by_drone.items():
            all_searched[drone_id] = list(cells)
        
        all_free = {}
        for drone_id, cells in self.free_by_drone.items():
            all_free[drone_id] = list(cells)
        
        all_walls = {}
        for drone_id, cells in self.walls_by_drone.items():
            all_walls[drone_id] = list(cells)
        
        return {
            "sender_id": self.drone_id,
            "searched_by_drone": all_searched,
            "free_by_drone": all_free,
            "walls_by_drone": all_walls,
            "features": [
                {
                    "type": f.feature_type,
                    "position": f.position,
                    "confidence": f.confidence,
                    "drone_id": f.drone_id,
                    "timestamp": f.timestamp,
                    "extra": f.extra_data
                }
                for f in self.features
            ]
        }
    
    def merge_remote_update(self, payload: dict, force: bool = False) -> bool:
        """
        Merge map data received from another drone.
        Now handles full knowledge sharing (all cells from all drones).
        
        Args:
            payload: Dictionary from get_sync_payload() of remote drone
            force: If True, always merge (for direct sync)
            
        Returns:
            True if new data was merged
        """
        sender_id = payload.get("sender_id")
        if sender_id is None or sender_id == self.drone_id:
            return False
        
        total_new_searched = 0
        total_new_walls = 0
        
        # Merge searched cells from ALL drones in payload
        searched_by_drone = payload.get("searched_by_drone", {})
        for drone_id_str, cells in searched_by_drone.items():
            drone_id = int(drone_id_str) if isinstance(drone_id_str, str) else drone_id_str
            cell_set = set(tuple(c) for c in cells)
            
            if drone_id not in self.searched_by_drone:
                self.searched_by_drone[drone_id] = set()
            
            new_cells = cell_set - self.searched_by_drone[drone_id]
            if new_cells:
                self.searched_by_drone[drone_id].update(new_cells)
                self.global_searched.update(new_cells)
                total_new_searched += len(new_cells)
        
        # Merge free cells from ALL drones
        free_by_drone = payload.get("free_by_drone", {})
        for drone_id_str, cells in free_by_drone.items():
            drone_id = int(drone_id_str) if isinstance(drone_id_str, str) else drone_id_str
            cell_set = set(tuple(c) for c in cells)
            
            if drone_id not in self.free_by_drone:
                self.free_by_drone[drone_id] = set()
            
            new_cells = cell_set - self.free_by_drone[drone_id]
            if new_cells:
                self.free_by_drone[drone_id].update(new_cells)
                self.global_free.update(new_cells)
        
        # Merge wall cells from ALL drones
        walls_by_drone = payload.get("walls_by_drone", {})
        for drone_id_str, cells in walls_by_drone.items():
            drone_id = int(drone_id_str) if isinstance(drone_id_str, str) else drone_id_str
            cell_set = set(tuple(c) for c in cells)
            
            if drone_id not in self.walls_by_drone:
                self.walls_by_drone[drone_id] = set()
            
            new_cells = cell_set - self.walls_by_drone[drone_id]
            if new_cells:
                self.walls_by_drone[drone_id].update(new_cells)
                self.global_walls.update(new_cells)
                total_new_walls += len(new_cells)
        
        # Merge features
        for f_data in payload.get("features", []):
            self._merge_feature(f_data)
        
        self.last_sync[sender_id] = time.time()
        
        # Only print significant merges (>5 new cells) to reduce spam
        if total_new_searched > 5 or total_new_walls > 5:
            print(f"[GOSSIP] D{self.drone_id} merged from D{sender_id}: +{total_new_searched} searched, +{total_new_walls} walls")
        
        return total_new_searched > 0 or total_new_walls > 0
    
    def _merge_feature(self, f_data: dict):
        """Merge a single feature from remote drone."""
        feature = MapFeature(
            feature_type=f_data["type"],
            position=tuple(f_data["position"]),
            confidence=f_data["confidence"],
            drone_id=f_data["drone_id"],
            timestamp=f_data["timestamp"],
            extra_data=f_data.get("extra", {})
        )
        
        # Check for duplicate
        for existing in self.features:
            if (existing.feature_type == feature.feature_type and
                abs(existing.position[0] - feature.position[0]) < 1.0 and
                abs(existing.position[1] - feature.position[1]) < 1.0):
                if feature.confidence > existing.confidence:
                    existing.confidence = feature.confidence
                return
        
        self.features.append(feature)
    
    def get_searched_by_drone(self) -> Dict[int, Set[Tuple[int, int]]]:
        """Get searched cells organized by drone (for color-coded rendering)."""
        return self.searched_by_drone
    
    def get_global_searched(self) -> Set[Tuple[int, int]]:
        """Get all searched cells from all drones."""
        return self.global_searched
    
    def get_global_free(self) -> Set[Tuple[int, int]]:
        """Get all known free cells from all drones."""
        return self.global_free
    
    def get_global_walls(self) -> Set[Tuple[int, int]]:
        """Get all known wall cells from all drones."""
        return self.global_walls
    
    def get_uncovered_cells(self, interior_only: bool = True) -> Set[Tuple[int, int]]:
        """
        Get cells that are free but not yet searched by any drone.
        
        Args:
            interior_only: If True, only return cells with y >= 0 (inside building)
            
        Returns:
            Set of (grid_x, grid_y) cells that need to be searched
        """
        uncovered = self.global_free - self.global_searched - self.global_walls
        if interior_only:
            uncovered = {c for c in uncovered if c[1] >= 0}
        return uncovered
    
    def get_coverage_stats(self) -> dict:
        """Get coverage statistics."""
        interior_free = len([c for c in self.global_free if c[1] >= 0])
        interior_searched = len([c for c in self.global_searched if c[1] >= 0])
        
        coverage_pct = 0
        if interior_free > 0:
            coverage_pct = int(100 * interior_searched / interior_free)
        
        return {
            "coverage_pct": min(coverage_pct, 100),
            "total_free": interior_free,
            "total_searched": interior_searched,
            "uncovered": interior_free - interior_searched,
            "by_drone": {
                drone_id: len([c for c in cells if c[1] >= 0])
                for drone_id, cells in self.searched_by_drone.items()
            }
        }
    
    def get_features_by_type(self, feature_type: str) -> List[MapFeature]:
        """Get all features of a specific type."""
        return [f for f in self.features if f.feature_type == feature_type]
    
    def get_ieds(self) -> List[MapFeature]:
        """Get all discovered IEDs."""
        return self.get_features_by_type("ied")
    
    def reset(self):
        """Reset map to empty state."""
        self.searched_by_drone.clear()
        self.global_searched.clear()
        self.free_by_drone.clear()
        self.global_free.clear()
        self.walls_by_drone.clear()
        self.global_walls.clear()
        self.features.clear()
        self.versions.clear()
        self.received_versions.clear()
        self.last_sync.clear()
