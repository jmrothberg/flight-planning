"""
Mesh Network for Multi-Drone Communication
Range-limited communication with AODV-style multi-hop routing.

HARDWARE INTEGRATION:
- Implement MeshProtocolInterface for your radio hardware (LoRa, WiFi, etc.)
- Replace SimulatedMeshProtocol instantiation in DroneManager
- All routing/gossip logic remains unchanged
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Set, Optional, Tuple
import time
import math
import uuid


@dataclass
class MeshMessage:
    """Message for mesh network communication."""
    msg_id: str
    msg_type: str  # HELLO, MAP_UPDATE, IED_ALERT, POSITION
    source_id: int  # Original author of the message
    sender_id: int  # Who actually transmitted this (for forwarded messages)
    dest_id: int  # -1 for broadcast
    hop_count: int
    ttl: int  # time to live (max hops)
    payload: dict
    timestamp: float
    
    @staticmethod
    def create(msg_type: str, source_id: int, dest_id: int, payload: dict, ttl: int = 3) -> 'MeshMessage':
        """Factory method to create a new message."""
        return MeshMessage(
            msg_id=f"{source_id}-{uuid.uuid4().hex[:8]}",
            msg_type=msg_type,
            source_id=source_id,
            sender_id=source_id,  # Initially, sender = source
            dest_id=dest_id,
            hop_count=0,
            ttl=ttl,
            payload=payload,
            timestamp=time.time()
        )


class MeshProtocolInterface(ABC):
    """
    Abstract interface for mesh communication hardware.
    
    HARDWARE INTEGRATION:
    Implement this interface for your radio hardware:
    - LoRa radios: Use serial/SPI to send/receive packets
    - WiFi mesh: Use UDP broadcast on local network
    - Custom RF: Implement for your specific hardware
    """
    
    @abstractmethod
    def send(self, message: MeshMessage) -> bool:
        """
        Send message to the network.
        Returns True if transmission started successfully.
        """
        pass
    
    @abstractmethod
    def receive(self) -> List[MeshMessage]:
        """
        Get messages received since last call.
        Returns list of received messages.
        """
        pass
    
    @abstractmethod
    def get_signal_strength(self, target_id: int) -> float:
        """
        Get signal strength to target drone.
        Returns 0.0 if out of range, 1.0 if perfect signal.
        """
        pass
    
    @abstractmethod
    def is_in_range(self, target_id: int) -> bool:
        """Check if target drone is within communication range."""
        pass


class SimulatedMeshProtocol(MeshProtocolInterface):
    """
    Simulated mesh protocol using shared memory.
    Range-limited based on drone positions.
    """
    
    def __init__(self, drone_id: int, comm_range: float,
                 positions_ref: Dict[int, Tuple[float, float]],
                 message_bus: List[MeshMessage]):
        """
        Initialize simulated mesh protocol.
        
        Args:
            drone_id: This drone's ID
            comm_range: Communication range in meters
            positions_ref: Reference to dict of all drone positions (updated externally)
            message_bus: Shared message bus (list) for all drones
        """
        self.drone_id = drone_id
        self.comm_range = comm_range
        self.positions = positions_ref  # Live reference, updated by DroneManager
        self.message_bus = message_bus
        self.processed_ids: Set[str] = set()
        self.inbox: List[MeshMessage] = []
    
    def _get_distance(self, other_id: int) -> float:
        """Calculate distance to another drone."""
        if self.drone_id not in self.positions or other_id not in self.positions:
            return float('inf')
        my_pos = self.positions[self.drone_id]
        other_pos = self.positions[other_id]
        return math.hypot(other_pos[0] - my_pos[0], other_pos[1] - my_pos[1])
    
    def get_signal_strength(self, target_id: int) -> float:
        """Signal strength based on distance (linear falloff)."""
        dist = self._get_distance(target_id)
        if dist > self.comm_range:
            return 0.0
        return max(0.0, 1.0 - (dist / self.comm_range))
    
    def is_in_range(self, target_id: int) -> bool:
        """Check if target is within comm range."""
        return self._get_distance(target_id) <= self.comm_range
    
    def send(self, message: MeshMessage) -> bool:
        """Add message to shared bus (simulates broadcast)."""
        self.message_bus.append(message)
        return True
    
    def receive(self) -> List[MeshMessage]:
        """Get messages from bus that are in range and not yet processed."""
        received = []
        
        for msg in self.message_bus:
            # Skip already processed
            if msg.msg_id in self.processed_ids:
                continue
            # Skip own messages (either as source or sender)
            if msg.source_id == self.drone_id or msg.sender_id == self.drone_id:
                self.processed_ids.add(msg.msg_id)
                continue
            # Check if SENDER (forwarder) is in range - NOT the original source!
            # This enables multi-hop: D0 sends, D1 forwards, D2 receives from D1
            if not self.is_in_range(msg.sender_id):
                continue
            
            # Message is receivable
            received.append(msg)
            self.processed_ids.add(msg.msg_id)
        
        return received
    
    def update_comm_range(self, new_range: float):
        """Update communication range (called when user adjusts with +/- keys)."""
        self.comm_range = new_range


class MeshNode:
    """
    Per-drone mesh networking node.
    Handles neighbor discovery, routing, and message forwarding.
    """
    
    def __init__(self, drone_id: int, protocol: MeshProtocolInterface):
        """
        Initialize mesh node.
        
        Args:
            drone_id: This drone's unique ID
            protocol: Communication protocol implementation
        """
        self.drone_id = drone_id
        self.protocol = protocol
        
        # Neighbor tracking
        self.neighbors: Dict[int, float] = {}  # drone_id -> last_seen_timestamp
        self.neighbor_positions: Dict[int, Tuple[float, float]] = {}
        self.neighbor_timeout = 3.0  # seconds before neighbor is considered lost
        
        # AODV-style routing table
        self.routing_table: Dict[int, int] = {}  # dest_id -> next_hop_id
        
        # Timing
        self.hello_interval = 0.5  # seconds between HELLO beacons
        self.last_hello_time = 0.0
        
        # Message deduplication
        self.seen_message_ids: Set[str] = set()
        self.max_seen_ids = 1000  # Prevent unbounded growth
    
    def update(self, my_position: Tuple[float, float]) -> List[MeshMessage]:
        """
        Update mesh node - send HELLOs, process incoming messages.
        Call this every frame.
        
        Args:
            my_position: Current (x, y) position of this drone
            
        Returns:
            List of messages intended for this drone (to be processed by higher layers)
        """
        now = time.time()
        
        # Expire old neighbors
        self._expire_neighbors(now)
        
        # Send periodic HELLO beacon
        if now - self.last_hello_time >= self.hello_interval:
            self._send_hello(my_position)
            self.last_hello_time = now
        
        # Process incoming messages
        messages_for_me = self._process_incoming()
        
        return messages_for_me
    
    def _send_hello(self, position: Tuple[float, float]):
        """Broadcast HELLO beacon with position."""
        msg = MeshMessage.create(
            msg_type="HELLO",
            source_id=self.drone_id,
            dest_id=-1,  # Broadcast
            payload={"position": list(position)},
            ttl=1  # HELLOs don't hop
        )
        self.protocol.send(msg)
    
    def _expire_neighbors(self, now: float):
        """Remove neighbors we haven't heard from recently."""
        expired = [nid for nid, last_seen in self.neighbors.items()
                   if now - last_seen > self.neighbor_timeout]
        for nid in expired:
            del self.neighbors[nid]
            if nid in self.neighbor_positions:
                del self.neighbor_positions[nid]
            # Remove routes through expired neighbor
            self.routing_table = {dest: hop for dest, hop in self.routing_table.items()
                                  if hop != nid}
    
    def _process_incoming(self) -> List[MeshMessage]:
        """Process received messages, forward if needed, return messages for this drone."""
        received = self.protocol.receive()
        messages_for_me = []
        
        for msg in received:
            # Deduplicate
            if msg.msg_id in self.seen_message_ids:
                continue
            self.seen_message_ids.add(msg.msg_id)
            
            # Limit seen IDs set size
            if len(self.seen_message_ids) > self.max_seen_ids:
                # Remove oldest (approximate by clearing half)
                self.seen_message_ids = set(list(self.seen_message_ids)[self.max_seen_ids // 2:])
            
            # Process HELLO - update neighbor table
            if msg.msg_type == "HELLO":
                self._handle_hello(msg)
                continue
            
            # Is this message for me (or broadcast)?
            if msg.dest_id == -1 or msg.dest_id == self.drone_id:
                messages_for_me.append(msg)
            
            # Forward if TTL allows and not for me specifically
            if msg.dest_id != self.drone_id and msg.ttl > 1 and msg.hop_count < msg.ttl:
                self._forward_message(msg)
        
        return messages_for_me
    
    def _handle_hello(self, msg: MeshMessage):
        """Process HELLO beacon - update neighbor and routing tables."""
        sender_id = msg.source_id
        self.neighbors[sender_id] = time.time()
        
        if "position" in msg.payload:
            pos = msg.payload["position"]
            self.neighbor_positions[sender_id] = (pos[0], pos[1])
        
        # Direct neighbor = direct route
        self.routing_table[sender_id] = sender_id
    
    def _forward_message(self, msg: MeshMessage):
        """Forward message to next hop."""
        # Create forwarded copy with incremented hop count
        # Set sender_id to THIS drone (we are forwarding)
        forwarded = MeshMessage(
            msg_id=msg.msg_id,
            msg_type=msg.msg_type,
            source_id=msg.source_id,  # Original author stays the same
            sender_id=self.drone_id,  # WE are now the sender (forwarder)
            dest_id=msg.dest_id,
            hop_count=msg.hop_count + 1,
            ttl=msg.ttl,
            payload=msg.payload,
            timestamp=msg.timestamp
        )
        self.protocol.send(forwarded)
    
    def send_to(self, dest_id: int, msg_type: str, payload: dict) -> bool:
        """
        Send message to specific drone (routes through mesh if needed).
        
        Args:
            dest_id: Target drone ID
            msg_type: Message type string
            payload: Message data
            
        Returns:
            True if message was sent, False if no route available
        """
        msg = MeshMessage.create(
            msg_type=msg_type,
            source_id=self.drone_id,
            dest_id=dest_id,
            payload=payload,
            ttl=5
        )
        return self.protocol.send(msg)
    
    def broadcast(self, msg_type: str, payload: dict):
        """Broadcast message to all reachable drones."""
        msg = MeshMessage.create(
            msg_type=msg_type,
            source_id=self.drone_id,
            dest_id=-1,  # Broadcast
            payload=payload,
            ttl=3  # Limit broadcast hops
        )
        self.protocol.send(msg)
    
    def get_neighbors(self) -> Set[int]:
        """Get set of current neighbor drone IDs."""
        return set(self.neighbors.keys())
    
    def get_reachable_drones(self) -> Set[int]:
        """Get set of all drones we can reach (directly or via routing)."""
        return set(self.routing_table.keys())
    
    def get_neighbor_positions(self) -> Dict[int, Tuple[float, float]]:
        """Get positions of all neighbors (from their HELLO beacons)."""
        return self.neighbor_positions.copy()
