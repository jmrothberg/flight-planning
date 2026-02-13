"""
Communication System for Drone Data Transmission
Handles priority-based transmission of important findings
Designed for low-bandwidth, high-latency scenarios typical in drone operations
"""

import json
import time
import threading
from typing import List, Dict, Any, Optional, Callable
from dataclasses import dataclass, asdict
from enum import Enum
from queue import PriorityQueue
import uuid

class MessagePriority(Enum):
    """Message priority levels."""
    CRITICAL = 1    # Immediate hazards, emergencies
    HIGH = 2        # People found, important equipment
    MEDIUM = 3      # General discoveries, map updates
    LOW = 4         # Status updates, telemetry

@dataclass
class Message:
    """Communication message structure."""
    id: str
    priority: MessagePriority
    message_type: str
    timestamp: float
    data: Dict[str, Any]
    retry_count: int = 0
    max_retries: int = 3
    
    def __lt__(self, other):
        """For priority queue comparison."""
        return self.priority.value < other.priority.value

class DataCompressor:
    """
    Data compression utilities for efficient transmission.
    Important for bandwidth-limited drone communications.
    """
    
    @staticmethod
    def compress_detection(detection) -> Dict[str, Any]:
        """Compress detection data for transmission."""
        return {
            'type': detection.object_type.value,
            'conf': round(detection.confidence, 2),
            'pos': [round(detection.position[0], 1), round(detection.position[1], 1)],
            'time': int(detection.timestamp),
            'desc': detection.description[:50]  # Truncate description
        }
    
    @staticmethod
    def compress_map_data(grid_map) -> Dict[str, Any]:
        """Compress map data for transmission."""
        # Send only occupied cells to reduce data
        occupied_cells = []
        for y in range(grid_map.grid_height):
            for x in range(grid_map.grid_width):
                if grid_map.grid[y, x] > 70:  # Occupied threshold
                    world_x, world_y = grid_map.grid_to_world(x, y)
                    occupied_cells.append([round(world_x, 1), round(world_y, 1)])
        
        return {
            'width': grid_map.width,
            'height': grid_map.height,
            'resolution': grid_map.resolution,
            'occupied': occupied_cells,
            'coverage': round(grid_map.get_exploration_progress(), 1)
        }
    
    @staticmethod
    def compress_telemetry(drone) -> Dict[str, Any]:
        """Compress drone telemetry for transmission."""
        status = drone.get_status()
        return {
            'pos': [round(p, 1) for p in status['position']],
            'battery': round(status['battery'], 1),
            'mission_complete': status['mission_complete']
        }

    @staticmethod
    def compress_lidar_keyframe(lidar_data: dict) -> Dict[str, Any]:
        """Compress a 54×42 LiDAR depth grid keyframe (~1-5 KB compressed).

        Encodes ranges as uint16 millimetres (0-9000mm) for compact transmission.
        """
        ranges = lidar_data.get("ranges", [])
        # Quantise to millimetres as uint16
        mm_ranges = [min(65535, int(r * 1000)) for r in ranges]
        return {
            'r': mm_ranges,
            'sa': round(lidar_data.get("start_angle", 0.0), 4),
            'as': round(lidar_data.get("angle_step", 0.0), 5),
            'n': lidar_data.get("num_rays", len(ranges)),
        }

    @staticmethod
    def compress_pose(position, orientation: float) -> Dict[str, Any]:
        """Compress drone pose for transmission (~50-80 bytes)."""
        return {
            'x': round(float(position[0]), 2),
            'y': round(float(position[1]), 2),
            'z': round(float(position[2]), 2) if len(position) > 2 else 0.0,
            'th': round(float(orientation), 3),
        }

    @staticmethod
    def compress_object_detection(detection) -> Dict[str, Any]:
        """Compress an object detection for transmission (~100-500 bytes)."""
        return {
            'type': detection.object_type.value if hasattr(detection.object_type, 'value') else str(detection.object_type),
            'conf': round(detection.confidence, 2),
            'pos': [round(detection.position[0], 1), round(detection.position[1], 1)],
            'time': int(detection.timestamp),
        }

class MessageFormatter:
    """
    Formats messages for different communication protocols.
    Supports JSON, compressed binary, and text formats.
    """
    
    @staticmethod
    def format_detection_alert(detections: List, drone_position: tuple) -> str:
        """Format detection alert for human operators."""
        if not detections:
            return ""
        
        alert_lines = [
            f"ALERT: {len(detections)} object(s) detected at position {drone_position}",
            "Details:"
        ]
        
        for detection in detections:
            alert_lines.append(
                f"- {detection.object_type.value.upper()}: confidence {detection.confidence:.0%} at {detection.position}"
            )
        
        return "\n".join(alert_lines)
    
    @staticmethod
    def format_status_update(drone, slam_system) -> str:
        """Format status update message."""
        drone_status = drone.get_status()
        exploration_progress = slam_system.get_exploration_progress()
        
        return (
            f"STATUS UPDATE:\n"
            f"Position: {drone_status['position']}\n"
            f"Battery: {drone_status['battery']:.1f}%\n"
            f"Map Coverage: {exploration_progress['map_coverage']:.1f}%\n"
            f"Mission Progress: {exploration_progress['points_visited']}/{exploration_progress['total_points']}"
        )
    
    @staticmethod
    def format_emergency_message(message: str, drone_position: tuple) -> str:
        """Format emergency message with location."""
        return f"EMERGENCY at {drone_position}: {message}"

class CommProtocol:
    """
    Communication protocol handler.
    Simulates different communication methods (WiFi, XBee 900 MHz, cellular, satellite).
    Default is xbee_900mhz to match real drone hardware.
    """

    # Maximum XBee fragment payload size (bytes)
    MAX_FRAGMENT_SIZE = 200

    def __init__(self, protocol_type: str = "xbee_900mhz"):
        """Initialize communication protocol."""
        self.protocol_type = protocol_type
        self.bandwidth_limit = self._get_bandwidth_limit()
        self.latency = self._get_latency()
        self.reliability = self._get_reliability()
        self.max_fragment_size = self.MAX_FRAGMENT_SIZE

        # Bandwidth tracking (rolling window)
        self._bandwidth_window_start = time.time()
        self._bandwidth_bytes_this_window = 0
        self._bandwidth_window_secs = 1.0  # 1-second window

        # Transmission statistics
        self.bytes_sent = 0
        self.messages_sent = 0
        self.failed_transmissions = 0

    def _get_bandwidth_limit(self) -> int:
        """Get bandwidth limit based on protocol type (bytes/s)."""
        limits = {
            "xbee_900mhz": 12500,  # 100 kbps = 12.5 KB/s
            "wifi": 1000000,       # 1 MB/s
            "cellular": 100000,    # 100 KB/s
            "satellite": 10000,    # 10 KB/s
            "lora": 1000           # 1 KB/s
        }
        return limits.get(self.protocol_type, 12500)

    def _get_latency(self) -> float:
        """Get typical latency for protocol (seconds)."""
        latencies = {
            "xbee_900mhz": 0.05,  # 50ms average (20-80ms per hop)
            "wifi": 0.01,          # 10ms
            "cellular": 0.1,       # 100ms
            "satellite": 0.6,      # 600ms
            "lora": 1.0            # 1000ms
        }
        return latencies.get(self.protocol_type, 0.05)

    def _get_reliability(self) -> float:
        """Get reliability (success rate) for protocol."""
        reliabilities = {
            "xbee_900mhz": 0.92,  # 8% base packet loss
            "wifi": 0.98,
            "cellular": 0.95,
            "satellite": 0.90,
            "lora": 0.85
        }
        return reliabilities.get(self.protocol_type, 0.92)

    def can_send(self, message_size: int) -> bool:
        """Check if message can be sent given bandwidth constraints."""
        # Check rolling bandwidth budget
        now = time.time()
        if now - self._bandwidth_window_start >= self._bandwidth_window_secs:
            self._bandwidth_window_start = now
            self._bandwidth_bytes_this_window = 0
        remaining = self.bandwidth_limit - self._bandwidth_bytes_this_window
        return message_size <= remaining

    def fragment_message(self, payload_bytes: bytes) -> list:
        """Split large payloads into ≤200-byte fragments.

        Returns list of bytes fragments. Each fragment is at most
        max_fragment_size bytes.
        """
        if len(payload_bytes) <= self.max_fragment_size:
            return [payload_bytes]
        fragments = []
        for offset in range(0, len(payload_bytes), self.max_fragment_size):
            fragments.append(payload_bytes[offset:offset + self.max_fragment_size])
        return fragments

    def simulate_transmission(self, message: Message) -> bool:
        """Simulate message transmission with protocol characteristics."""
        import random

        # Simulate transmission delay
        time.sleep(self.latency * 0.001)  # Convert to actual delay for simulation

        # Simulate transmission success/failure
        success = random.random() < self.reliability

        if success:
            self.messages_sent += 1
            # Estimate message size (rough approximation)
            message_size = len(json.dumps(asdict(message), default=str))
            self.bytes_sent += message_size
            self._bandwidth_bytes_this_window += message_size
        else:
            self.failed_transmissions += 1

        return success

    def get_stats(self) -> Dict[str, Any]:
        """Get transmission statistics."""
        return {
            'protocol': self.protocol_type,
            'bytes_sent': self.bytes_sent,
            'messages_sent': self.messages_sent,
            'failed_transmissions': self.failed_transmissions,
            'bandwidth_limit_bps': self.bandwidth_limit,
            'success_rate': self.messages_sent / (self.messages_sent + self.failed_transmissions) if (self.messages_sent + self.failed_transmissions) > 0 else 0
        }

class CommSystem:
    """
    Complete communication system for drone operations.
    Handles message queuing, prioritization, compression, and transmission.
    """
    
    def __init__(self, protocol_type: str = "xbee_900mhz"):
        """Initialize communication system (default: XBee 900 MHz mesh radio)."""
        self.protocol = CommProtocol(protocol_type)
        self.compressor = DataCompressor()
        self.formatter = MessageFormatter()

        # Message queuing
        self.message_queue = PriorityQueue()
        self.pending_messages = {}
        # Store-and-forward buffer for messages that can't be delivered
        self.store_buffer = []
        # Recent successfully sent message summaries for GUI display
        self._sent_log = []  # list[str]
        # Track which detection object types we've already displayed (GUI dedupe)
        self._seen_detection_types = set()

        # System state
        self.is_active = True
        self.transmission_thread = None
        self.callbacks = {}  # Message type -> callback function

        # Transmission control — adjusted for XBee timing
        self.transmission_interval = 0.5 if protocol_type == "xbee_900mhz" else 1.0
        self.batch_size = 3 if protocol_type == "xbee_900mhz" else 5

        # Start transmission thread
        self._start_transmission_thread()
    
    def send_priority_data(self, detections: List, drone_position: tuple):
        """Send high-priority detection data immediately."""
        if not detections:
            return
        
        # Determine priority based on object types
        priority = MessagePriority.MEDIUM
        for detection in detections:
            if detection.object_type.value in ['person', 'hazard']:
                priority = MessagePriority.HIGH
                break
        
        # Create compressed detection data
        compressed_detections = [self.compressor.compress_detection(d) for d in detections]
        
        message = Message(
            id=str(uuid.uuid4()),
            priority=priority,
            message_type="detection_alert",
            timestamp=time.time(),
            data={
                'detections': compressed_detections,
                'drone_position': [round(drone_position[0], 1), round(drone_position[1], 1)],
                'alert_text': self.formatter.format_detection_alert(detections, drone_position)
            }
        )
        
        self.message_queue.put(message)
    
    def send_map_update(self, grid_map):
        """Send map update with exploration progress."""
        compressed_map = self.compressor.compress_map_data(grid_map)
        
        message = Message(
            id=str(uuid.uuid4()),
            priority=MessagePriority.MEDIUM,
            message_type="map_update",
            timestamp=time.time(),
            data=compressed_map
        )
        
        self.message_queue.put(message)
    
    def send_status_update(self, drone, slam_system):
        """Send routine status update."""
        telemetry = self.compressor.compress_telemetry(drone)
        status_text = self.formatter.format_status_update(drone, slam_system)
        
        message = Message(
            id=str(uuid.uuid4()),
            priority=MessagePriority.LOW,
            message_type="status_update",
            timestamp=time.time(),
            data={
                'telemetry': telemetry,
                'status_text': status_text
            }
        )
        
        self.message_queue.put(message)
    
    def send_emergency(self, emergency_message: str, drone_position: tuple):
        """Send emergency message with highest priority."""
        formatted_message = self.formatter.format_emergency_message(emergency_message, drone_position)
        
        message = Message(
            id=str(uuid.uuid4()),
            priority=MessagePriority.CRITICAL,
            message_type="emergency",
            timestamp=time.time(),
            data={
                'emergency_text': formatted_message,
                'drone_position': drone_position
            }
        )
        
        # Emergency messages bypass queue and send immediately
        self._transmit_message(message)
    
    def register_callback(self, message_type: str, callback: Callable):
        """Register callback for incoming messages."""
        self.callbacks[message_type] = callback
    
    def get_transmission_stats(self) -> Dict[str, Any]:
        """Get communication system statistics."""
        return {
            **self.protocol.get_stats(),
            'queue_size': self.message_queue.qsize(),
            'pending_messages': len(self.pending_messages)
        }
    
    def _start_transmission_thread(self):
        """Start background transmission thread."""
        def transmission_worker():
            while self.is_active:
                try:
                    self._process_message_queue()
                    time.sleep(self.transmission_interval)
                except Exception as e:
                    print(f"Transmission error: {e}")
        
        self.transmission_thread = threading.Thread(target=transmission_worker, daemon=True)
        self.transmission_thread.start()
    
    def _process_message_queue(self):
        """Process pending messages in queue."""
        messages_to_send = []
        
        # Get up to batch_size messages from queue
        for _ in range(self.batch_size):
            if not self.message_queue.empty():
                message = self.message_queue.get()
                messages_to_send.append(message)
            else:
                break
        
        # Send messages
        for message in messages_to_send:
            self._transmit_message(message)
    
    def _transmit_message(self, message: Message) -> bool:
        """Transmit a single message."""
        success = self.protocol.simulate_transmission(message)
        
        if success:
            # Message sent successfully
            if message.id in self.pending_messages:
                del self.pending_messages[message.id]
            
            # Call callback if registered
            if message.message_type in self.callbacks:
                self.callbacks[message.message_type](message)
            # Log summary for GUI communications panel
            # PER USER REQUEST: For detection alerts, DO NOT show the verbose
            # "detection_alert" text. Instead, print each detected object TYPE
            # exactly once for the whole mission.
            if message.message_type == "detection_alert" and isinstance(message.data, dict):
                dets = message.data.get('detections', [])
                for det in dets:
                    t = str(det.get('type', '')).strip()
                    if t and t not in self._seen_detection_types:
                        self._seen_detection_types.add(t)
                        self._sent_log.append(t)
            else:
                summary = f"{message.message_type} [{message.priority.name}]"
                if isinstance(message.data, dict) and 'text' in message.data:
                    summary += f": {str(message.data['text'])[:60]}"
                self._sent_log.append(summary)
            if len(self._sent_log) > 50:
                self._sent_log = self._sent_log[-50:]
            
        else:
            # Transmission failed, handle retry
            message.retry_count += 1
            if message.retry_count < message.max_retries:
                # Add back to queue for retry
                self.message_queue.put(message)
                self.pending_messages[message.id] = message
            else:
                # Max retries exceeded, drop message
                print(f"Message {message.id} dropped after {message.max_retries} retries")
                if message.id in self.pending_messages:
                    del self.pending_messages[message.id]
        
        return success

    def get_recent_log(self, limit: int = 8) -> list:
        """Return the most recent sent message summaries for GUI display."""
        return self._sent_log[-limit:]
    
    def reset(self):
        """Reset communication system."""
        # Clear queues
        while not self.message_queue.empty():
            try:
                self.message_queue.get_nowait()
            except:
                break
        
        self.pending_messages.clear()
        
        # Reset protocol stats
        self.protocol.bytes_sent = 0
        self.protocol.messages_sent = 0
        self.protocol.failed_transmissions = 0
    
    def shutdown(self):
        """Shutdown communication system."""
        self.is_active = False
        if self.transmission_thread:
            self.transmission_thread.join(timeout=2.0)
