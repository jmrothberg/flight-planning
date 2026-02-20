"""
Simulated external sensors (IED, etc.).

This module provides placeholder sensor logic for simulation and a
clean, text-based interface that will be used by the REAL hardware drivers.

REAL HARDWARE INTEGRATION NOTES:
- Replace `IEDSensor.reading_from_environment(...)` with a method that reads
  from your actual IED detection hardware (serial/UDP/SPI/etc.).
- Keep the `IEDReading.to_text()` format the same so other modules (comms/UI)
  continue working without code changes.
"""

from dataclasses import dataclass
from typing import Dict, Tuple, Optional
import math
import random

@dataclass
class IEDReading:
    """Container for IED component probabilities (simulated)."""
    components: Dict[str, float]
    distance_m: float
    confidence: float

    def to_text(self) -> str:
        """Simple text representation used both in simulation and on real systems."""
        parts = [f"{k}:{v:.0%}" for k, v in self.components.items()]
        return f"IED components -> {' | '.join(parts)} | dist:{self.distance_m:.1f}m | conf:{self.confidence:.0%}"

class IEDSensor:
    """
    Simulated IED sensor that produces a component mix based on proximity to
    simulated IED objects in the environment.

    REAL SENSOR HOOK:
    - Implement `read_hardware()` to return an IEDReading from the real device.
    - Keep the `IEDReading` schema and `to_text()` format stable.
    """

    def __init__(self, detection_range_m: float = 3.0):  # 3 meters range
        self.detection_range_m = detection_range_m
        self.last_reading: Optional[IEDReading] = None
        # IED detector fan configuration
        self.num_rays = 10  # 360/36 = 10 rays (every 36 degrees)
        self.fov_degrees = 360  # Full circle coverage
        # Track which IEDs have been reported to avoid duplicates
        self.reported_ieds = set()  # Store (x, y) positions of reported IEDs

    def read(self, drone_xy: Tuple[float, float], environment) -> Optional[IEDReading]:
        """Read from simulation (or real hardware in production)."""
        # In real deployment, replace with: return self.read_hardware()
        reading = self._reading_from_environment(drone_xy, environment)
        self.last_reading = reading
        return reading

    def _reading_from_environment(self, drone_xy: Tuple[float, float], environment) -> Optional[IEDReading]:
        """Compute a simulated IED reading based on nearest IED object."""
        # Find nearest IED object from environment.objects_of_interest
        ied_positions = [(x, y, conf) for (x, y, t, conf) in environment.objects_of_interest if str(t).lower() == "ied"]
        if not ied_positions:
            return None

        nearest = None
        min_dist = 1e9
        for (x, y, conf) in ied_positions:
            d = math.hypot(x - drone_xy[0], y - drone_xy[1])
            if d < min_dist:
                min_dist = d
                nearest = (x, y, conf)

        if nearest is None or min_dist > self.detection_range_m:
            return None
        
        # Check if we've already reported this IED
        ied_pos = (round(nearest[0], 1), round(nearest[1], 1))  # Round to avoid float precision issues
        if ied_pos in self.reported_ieds:
            return None  # Already reported, don't report again
        
        # Mark this IED as reported
        self.reported_ieds.add(ied_pos)

        # Simulate component probabilities increasing as we get closer
        norm = max(0.0, 1.0 - (min_dist / self.detection_range_m))
        noise = lambda: random.uniform(-0.05, 0.05)
        # Five illustrative components common to explosive detection concepts
        components = {
            "nitrogen": max(0.0, min(1.0, 0.6 * norm + noise())),
            "oxidizer": max(0.0, min(1.0, 0.55 * norm + noise())),
            "fuel": max(0.0, min(1.0, 0.5 * norm + noise())),
            "wiring": max(0.0, min(1.0, 0.45 * norm + noise())),
            "casing": max(0.0, min(1.0, 0.4 * norm + noise())),
        }
        confidence = max(0.0, min(1.0, norm))
        return IEDReading(components=components, distance_m=min_dist, confidence=confidence)

    # def read_hardware(self) -> Optional[IEDReading]:
    #     """REAL SENSOR: Replace with hardware read returning IEDReading."""
    #     # Example placeholder: parse a single-line text report
    #     # from serial/udp and convert to IEDReading.
    #     return None


