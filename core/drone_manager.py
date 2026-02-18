"""
Drone Manager for Multi-Drone Operations
Manages multiple drones with mesh networking and shared map.
"""

from typing import List, Dict, Tuple, Set, Optional
import time
import math

from core.stm32h7.drone import Drone
from core.stm32wl.mesh_network import MeshNode, SimulatedMeshProtocol, MeshMessage
from core.stm32wl.gossip_map import GossipMap
from core.stm32n6.search_systematic_mapper import SystematicMapper


BASE_STATION_ID = -1  # Special ID for the ground base station


class DroneManager:
    """
    Manages multiple drones with mesh networking and coordinated search.
    """

    # Drone colors for visualization (RGB tuples) — supports up to 12 drones
    COLORS = [
        (255, 80, 80),    # Red - Drone 0
        (80, 80, 255),    # Blue - Drone 1
        (80, 255, 80),    # Green - Drone 2
        (255, 200, 80),   # Orange - Drone 3
        (200, 80, 255),   # Purple - Drone 4
        (80, 220, 220),   # Cyan - Drone 5
        (255, 130, 180),  # Pink - Drone 6
        (180, 180, 80),   # Olive - Drone 7
        (255, 160, 80),   # Tangerine - Drone 8
        (80, 180, 130),   # Teal - Drone 9
        (200, 120, 80),   # Brown - Drone 10
        (160, 80, 160),   # Plum - Drone 11
    ]
    
    def __init__(self, count: int = 1, comm_range: float = 10.0,
                 building_width: float = 25.0, building_height: float = 25.0):
        """
        Initialize drone manager.

        Args:
            count: Number of drones (1-12)
            comm_range: Communication range in meters
        """
        self.count = max(1, min(12, count))
        self.comm_range = comm_range
        self._building_width = building_width
        self._building_height = building_height
        
        # Per-drone instances
        self.drones: List[Drone] = []
        self.mesh_nodes: List[MeshNode] = []
        self.gossip_maps: List[GossipMap] = []
        self.search_algorithms: List[SystematicMapper] = []
        
        # Shared resources for simulation
        self.message_bus: List[MeshMessage] = []
        self.positions: Dict[int, Tuple[float, float]] = {}  # Live position reference
        
        # Entry tracking per drone
        self.inside_building: List[bool] = []
        self.entry_sequences: List[List[Tuple[float, float]]] = []
        self.entry_indices: List[int] = []
        self.returning_home: List[bool] = []
        self.mission_start_times: List[Optional[float]] = []
        
        # Mesh communication protocols (one per drone, share positions dict)
        self.protocols: List[SimulatedMeshProtocol] = []
        
        # Per-drone breadcrumbs (trail of positions)
        self.breadcrumbs: List[List[Tuple[float, float]]] = []
        self.last_breadcrumb_time: float = 0.0
        self.breadcrumb_interval: float = 0.2  # seconds between breadcrumbs
        
        # Gossip sync timing
        self.last_gossip_time = 0.0
        self.gossip_interval = 0.1  # seconds between gossip syncs (faster for better coordination)
        
        # Target claiming to prevent oscillation
        # Key: (grid_x, grid_y), Value: (drone_id, claim_time)
        self.claimed_targets: Dict[Tuple[int, int], Tuple[int, float]] = {}
        self.claim_timeout = 5.0  # seconds before claim expires

        # Sync visualization tracking
        # List of (timestamp, visual_sender_id, visual_receiver_id)
        self.recent_syncs: List[Tuple[float, int, int]] = []
        self._sync_turn: Dict[Tuple[int, int], bool] = {}  # pair -> direction toggle

        # Base station — ground controller at launch point
        # Has its own GossipMap + MeshProtocol + MeshNode
        # Receives map data ONLY via radio (direct or multi-hop relay)
        self.base_station_pos: Optional[Tuple[float, float]] = None
        self.base_station_gossip: Optional[GossipMap] = None
        self.base_station_protocol: Optional[SimulatedMeshProtocol] = None
        self.base_station_mesh: Optional[MeshNode] = None
    
    def initialize(self, entry_point: Tuple[float, float] = (25.0, -3.0)):
        """
        Create and initialize all drones.
        
        Args:
            entry_point: Base entry point (x, y) - drones will be staggered
        """
        self.drones.clear()
        self.mesh_nodes.clear()
        self.gossip_maps.clear()
        self.search_algorithms.clear()
        self.protocols.clear()
        self.message_bus.clear()
        self.positions.clear()
        self.inside_building.clear()
        self.entry_sequences.clear()
        self.entry_indices.clear()
        self.returning_home.clear()
        self.mission_start_times.clear()
        self.breadcrumbs.clear()
        
        for i in range(self.count):
            # Stagger entry positions to avoid collision
            offset_x = (i - (self.count - 1) / 2) * 3.0  # 3m spacing
            start_pos = (entry_point[0] + offset_x, entry_point[1], 3.0)
            
            # Create drone
            drone = Drone(position=start_pos)
            self.drones.append(drone)
            
            # Initialize position tracking
            self.positions[i] = (start_pos[0], start_pos[1])
            
            # Create mesh protocol (all share positions dict and message bus)
            protocol = SimulatedMeshProtocol(
                drone_id=i,
                comm_range=self.comm_range,
                positions_ref=self.positions,
                message_bus=self.message_bus
            )
            self.protocols.append(protocol)
            
            # Create mesh node
            mesh = MeshNode(drone_id=i, protocol=protocol)
            self.mesh_nodes.append(mesh)
            
            # Create gossip map
            gossip = GossipMap(drone_id=i)
            self.gossip_maps.append(gossip)
            
            # Create search algorithm (pass drone_id for identification)
            search = SystematicMapper(coverage_radius=2.0, drone_id=i,
                                      building_width=self._building_width,
                                      building_height=self._building_height)
            self.search_algorithms.append(search)
            
            # Entry sequence - staggered door entry
            # Keep entry simple - let search algorithm handle spreading
            door_x = entry_point[0] + offset_x * 0.5  # Wider offset at door for spacing
            
            # Simple entry: outside -> inside -> a few meters in
            # Search algorithm will naturally spread drones to different areas
            entry_seq = [
                (door_x, -0.5),        # Just outside door
                (door_x, 1.0),         # Just inside door
                (door_x, 4.0),         # Move further in (search starts here)
            ]
            self.entry_sequences.append(entry_seq)
            self.entry_indices.append(0)
            self.inside_building.append(False)
            self.returning_home.append(False)
            self.mission_start_times.append(None)
            self.breadcrumbs.append([])  # Empty trail for each drone
        
        # ── Base Station (ground controller at launch point) ──────────
        self.base_station_pos = (entry_point[0], entry_point[1])
        self.positions[BASE_STATION_ID] = self.base_station_pos
        self.base_station_protocol = SimulatedMeshProtocol(
            drone_id=BASE_STATION_ID,
            comm_range=self.comm_range,
            positions_ref=self.positions,
            message_bus=self.message_bus,
        )
        self.base_station_mesh = MeshNode(drone_id=BASE_STATION_ID, protocol=self.base_station_protocol)
        self.base_station_gossip = GossipMap(drone_id=BASE_STATION_ID)

        print(f"DroneManager: Initialized {self.count} drones with {self.comm_range}m comm range")
        print(f"  Base station at ({self.base_station_pos[0]:.1f}, {self.base_station_pos[1]:.1f})")
    
    def update_breadcrumbs(self):
        """Add current positions to each drone's breadcrumb trail."""
        now = time.time()
        if now - self.last_breadcrumb_time < self.breadcrumb_interval:
            return
        self.last_breadcrumb_time = now
        
        for i, drone in enumerate(self.drones):
            pos = (float(drone.position[0]), float(drone.position[1]))
            self.breadcrumbs[i].append(pos)
            # Limit trail length
            if len(self.breadcrumbs[i]) > 500:
                self.breadcrumbs[i] = self.breadcrumbs[i][-500:]
    
    def update_positions(self):
        """Update position reference dict from drone states."""
        for i, drone in enumerate(self.drones):
            self.positions[i] = (float(drone.position[0]), float(drone.position[1]))
    
    def update_mesh(self) -> Dict[int, List[MeshMessage]]:
        """
        Update mesh networking - HELLO beacons and message processing.

        Returns:
            Dict mapping drone_id to list of messages received
        """
        self.update_positions()

        messages_per_drone: Dict[int, List[MeshMessage]] = {}

        for i, mesh in enumerate(self.mesh_nodes):
            pos = self.positions[i]
            messages = mesh.update(pos)
            messages_per_drone[i] = messages

        # Base station mesh update (receives HELLO beacons, etc.)
        if self.base_station_mesh and self.base_station_pos:
            self.base_station_mesh.update(self.base_station_pos)

        return messages_per_drone
    
    def process_gossip(self, messages_per_drone: Dict[int, List[MeshMessage]]):
        """
        Process gossip map updates from received messages.
        
        Args:
            messages_per_drone: Messages received by each drone
        """
        for drone_id, messages in messages_per_drone.items():
            for msg in messages:
                if msg.msg_type == "MAP_UPDATE":
                    # Merge remote map data
                    self.gossip_maps[drone_id].merge_remote_update(msg.payload)
                elif msg.msg_type == "IED_ALERT":
                    # Could trigger alerts on all drones
                    pass
    
    def broadcast_map_updates(self):
        """Have each drone broadcast its map data to neighbors.

        FIXED: Also does DIRECT gossip sync between drones in range,
        bypassing potentially broken mesh protocol.
        """
        now = time.time()
        if now - self.last_gossip_time < self.gossip_interval:
            return
        self.last_gossip_time = now

        # Clean old sync events (keep last 3 seconds for visualization)
        self.recent_syncs = [(t, s, r) for t, s, r in self.recent_syncs if now - t < 3.0]

        # Update each drone's own position in its gossip map (for propagation)
        for i in range(self.count):
            pos_i = self.positions.get(i)
            if pos_i:
                self.gossip_maps[i].update_position(i, pos_i)

        # DIRECT gossip sync: check distance between all drone pairs
        # If within comm_range, directly merge gossip data
        for i in range(self.count):
            for j in range(i + 1, self.count):
                # Calculate distance between drones i and j
                pos_i = self.positions.get(i)
                pos_j = self.positions.get(j)
                if pos_i and pos_j:
                    dist = math.hypot(pos_j[0] - pos_i[0], pos_j[1] - pos_i[1])
                    if dist <= self.comm_range:
                        # Drones are in range - sync gossip DIRECTLY with force=True
                        # to bypass version check and always share current data
                        payload_i = self.gossip_maps[i].get_sync_payload()
                        payload_j = self.gossip_maps[j].get_sync_payload()
                        # i receives from j, j receives from i (force=True for direct sync)
                        self.gossip_maps[i].merge_remote_update(payload_j, force=True)
                        self.gossip_maps[j].merge_remote_update(payload_i, force=True)

                        # Record sync event with alternating visual sender
                        pair_key = (i, j)
                        turn = self._sync_turn.get(pair_key, True)
                        self._sync_turn[pair_key] = not turn
                        visual_sender = i if turn else j
                        visual_receiver = j if turn else i
                        self.recent_syncs.append((now, visual_sender, visual_receiver))

        # Base station gossip: check distance from each drone to base station
        if self.base_station_gossip and self.base_station_pos:
            for i in range(self.count):
                pos_i = self.positions.get(i)
                if pos_i:
                    dist = math.hypot(pos_i[0] - self.base_station_pos[0],
                                      pos_i[1] - self.base_station_pos[1])
                    if dist <= self.comm_range:
                        # Drone in range of base station — bidirectional sync
                        payload_drone = self.gossip_maps[i].get_sync_payload()
                        payload_base = self.base_station_gossip.get_sync_payload()
                        self.base_station_gossip.merge_remote_update(payload_drone, force=True)
                        self.gossip_maps[i].merge_remote_update(payload_base, force=True)
                        # Record sync: drone → base station (for radio visualization)
                        self.recent_syncs.append((now, i, BASE_STATION_ID))

        # Also try mesh broadcast (for multi-hop when it works)
        for i, mesh in enumerate(self.mesh_nodes):
            if mesh.get_neighbors():
                payload = self.gossip_maps[i].get_sync_payload()
                mesh.broadcast("MAP_UPDATE", payload)

        # Clean old messages from bus to prevent unbounded growth
        self.message_bus[:] = [m for m in self.message_bus if now - m.timestamp < 5.0]
    
    def update_search_from_gossip(self, drone_id: int):
        """
        Update search algorithm with global map knowledge.
        
        Args:
            drone_id: Which drone's search algorithm to update
        """
        gossip = self.gossip_maps[drone_id]
        search = self.search_algorithms[drone_id]
        
        # Update search algorithm's knowledge of globally searched cells
        # This prevents drones from searching areas already covered by others
        search.set_global_searched(gossip.get_global_searched())
        
        # Share global free cells (frontiers from other drones)
        # This motivates drones to explore areas discovered by others
        search.set_global_free(gossip.get_global_free())
        
        # V11.4: Pass OTHER drones' current positions for spatial separation
        # This is the key to preventing clustering - drones prefer frontiers
        # that are far from where other drones currently are
        other_positions = []
        for other_id, pos in self.positions.items():
            if other_id != drone_id:
                other_positions.append(pos)
        search.set_other_drone_positions(other_positions)
    
    def sync_local_to_gossip(self, drone_id: int):
        """
        Sync local search algorithm discoveries to gossip map.
        
        Args:
            drone_id: Which drone to sync
        """
        search = self.search_algorithms[drone_id]
        gossip = self.gossip_maps[drone_id]
        
        # Add locally searched cells to gossip map
        gossip.add_local_searched(search.searched_cells.copy())
        gossip.add_local_free(search.free_cells.copy())
        gossip.add_local_walls(search.wall_cells.copy())
    
    def claim_target(self, drone_id: int, target_cell: Tuple[int, int]) -> bool:
        """
        Claim a target cell for a drone. Prevents other drones from targeting it.
        
        Args:
            drone_id: Drone claiming the target
            target_cell: Grid cell being claimed
            
        Returns:
            True if claim successful, False if already claimed by another drone
        """
        now = time.time()
        
        # Clean up expired claims
        expired = [cell for cell, (did, t) in self.claimed_targets.items() 
                   if now - t > self.claim_timeout]
        for cell in expired:
            del self.claimed_targets[cell]
        
        # Check if already claimed by another drone
        if target_cell in self.claimed_targets:
            claiming_drone, claim_time = self.claimed_targets[target_cell]
            if claiming_drone != drone_id and now - claim_time < self.claim_timeout:
                return False  # Already claimed by another
        
        # Claim it
        self.claimed_targets[target_cell] = (drone_id, now)
        return True
    
    def release_target(self, drone_id: int, target_cell: Tuple[int, int]):
        """Release a claimed target."""
        if target_cell in self.claimed_targets:
            claiming_drone, _ = self.claimed_targets[target_cell]
            if claiming_drone == drone_id:
                del self.claimed_targets[target_cell]
    
    def get_claimed_by_others(self, drone_id: int) -> Set[Tuple[int, int]]:
        """Get set of cells claimed by OTHER drones (not this one)."""
        now = time.time()
        claimed = set()
        for cell, (did, t) in self.claimed_targets.items():
            if did != drone_id and now - t < self.claim_timeout:
                claimed.add(cell)
        return claimed
    
    def get_drone(self, drone_id: int) -> Drone:
        """Get drone instance by ID."""
        return self.drones[drone_id]
    
    def get_search(self, drone_id: int) -> SystematicMapper:
        """Get search algorithm for drone."""
        return self.search_algorithms[drone_id]
    
    def get_gossip_map(self, drone_id: int) -> GossipMap:
        """Get gossip map for drone."""
        return self.gossip_maps[drone_id]
    
    def get_mesh_node(self, drone_id: int) -> MeshNode:
        """Get mesh node for drone."""
        return self.mesh_nodes[drone_id]
    
    def get_color(self, drone_id: int) -> Tuple[int, int, int]:
        """Get color for drone visualization."""
        return self.COLORS[drone_id % len(self.COLORS)]
    
    def get_mesh_links(self) -> List[Tuple[int, int]]:
        """
        Get active mesh connections for visualization.

        Returns:
            List of (drone_id_1, drone_id_2) tuples for connected drones
        """
        links = []
        for i, mesh in enumerate(self.mesh_nodes):
            for neighbor_id in mesh.get_neighbors():
                if i < neighbor_id:  # Avoid duplicates
                    links.append((i, neighbor_id))
        return links

    def get_gossip_links(self) -> List[Tuple[int, int, float]]:
        """
        Get drone pairs within direct gossip range (distance-based check).
        More reliable than mesh protocol HELLO beacons.
        Includes base station links (BASE_STATION_ID, drone_id, distance).

        Returns:
            List of (id_1, id_2, distance) tuples
        """
        links = []
        for i in range(self.count):
            for j in range(i + 1, self.count):
                pos_i = self.positions.get(i)
                pos_j = self.positions.get(j)
                if pos_i and pos_j:
                    dist = math.hypot(pos_j[0] - pos_i[0], pos_j[1] - pos_i[1])
                    if dist <= self.comm_range:
                        links.append((i, j, dist))
        # Base station links
        if self.base_station_pos:
            for i in range(self.count):
                pos_i = self.positions.get(i)
                if pos_i:
                    dist = math.hypot(pos_i[0] - self.base_station_pos[0],
                                      pos_i[1] - self.base_station_pos[1])
                    if dist <= self.comm_range:
                        links.append((BASE_STATION_ID, i, dist))
        return links

    def get_recent_syncs(self) -> List[Tuple[float, int, int]]:
        """Get recent sync events for visualization (timestamp, sender_id, receiver_id)."""
        return self.recent_syncs
    
    def get_all_searched_by_drone(self) -> Dict[int, Set[Tuple[int, int]]]:
        """
        Get combined searched cells from all drones' gossip maps.
        Uses drone 0's gossip map as the reference (most complete view).
        
        Returns:
            Dict mapping drone_id to set of searched cells
        """
        if not self.gossip_maps:
            return {}
        
        # Use first drone's gossip map as it has the merged view
        # In a real system, each drone only knows what it's received
        # For visualization, we use drone 0's view
        return self.gossip_maps[0].get_searched_by_drone()
    
    def get_global_coverage_stats(self) -> dict:
        """Get combined coverage statistics from all search algorithms directly."""
        if not self.search_algorithms:
            return {"coverage_pct": 0}

        # Use search algorithms directly (always accurate, no gossip dependency)
        all_searched = set()
        all_free = set()

        for search in self.search_algorithms:
            all_searched.update(search.searched_cells)
            all_free.update(search.free_cells)

        interior_free = set(c for c in all_free if c[1] >= 0)
        interior_searched = len(interior_free & all_searched)

        coverage_pct = 0
        if len(interior_free) > 0:
            coverage_pct = int(100 * interior_searched / len(interior_free))

        return {
            "coverage_pct": min(coverage_pct, 100),
            "total_free": len(interior_free),
            "total_searched": interior_searched,
            "uncovered": len(interior_free) - interior_searched
        }
    
    def set_comm_range(self, new_range: float):
        """
        Update communication range for all drones.

        Args:
            new_range: New communication range in meters
        """
        self.comm_range = max(2.0, min(50.0, new_range))  # Clamp to reasonable range
        for protocol in self.protocols:
            protocol.update_comm_range(self.comm_range)
        if self.base_station_protocol:
            self.base_station_protocol.update_comm_range(self.comm_range)
        print(f"Comm range updated to {self.comm_range:.0f}m")
    
    def set_drone_count(self, count: int, entry_point: Tuple[float, float] = (25.0, -3.0)):
        """
        Change number of drones (requires re-initialization).
        
        Args:
            count: New drone count (1-4)
            entry_point: Entry point for drones
        """
        self.count = max(1, min(12, count))
        self.initialize(entry_point)
    
    def reset(self, entry_point: Tuple[float, float] = (25.0, -3.0)):
        """Reset all drones and maps for new mission."""
        self.initialize(entry_point)
    
    def is_all_missions_complete(self) -> bool:
        """Check if all drones have completed their missions."""
        return all(drone.mission_complete for drone in self.drones)
    
    def get_combined_debug_info(self) -> dict:
        """Get combined debug info for all drones."""
        stats = self.get_global_coverage_stats()
        
        # Count IEDs found across all gossip maps
        ieds_found = set()
        for gossip in self.gossip_maps:
            for ied in gossip.get_ieds():
                ieds_found.add(ied.position)
        
        return {
            "drone_count": self.count,
            "comm_range": self.comm_range,
            "mesh_links": len(self.get_mesh_links()),
            "coverage_pct": stats["coverage_pct"],
            "uncovered": stats["uncovered"],
            "ieds_found": len(ieds_found)
        }
