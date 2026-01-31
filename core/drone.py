"""
Drone class - Core drone flight control and navigation
Designed to be portable to STM32 microcontrollers
"""

import numpy as np
from typing import Tuple, List, Optional
import math

class Drone:
    """
    Core drone class handling flight control, navigation, and mission execution.
    Designed with STM32 portability in mind - uses minimal dependencies.
    """
    
    def __init__(self, position: Tuple[float, float, float] = (0, 0, 0)):
        """Initialize drone with starting position (x, y, z)."""
        # State variables
        self.position = np.array(position, dtype=np.float32)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.orientation = 0.0  # Yaw angle in radians
        self.angular_velocity = 0.0
        
        # Flight parameters (tunable for different drone types)
        self.max_speed = 2.0  # m/s - Reduced for smoother movement
        self.max_angular_speed = 1.0  # rad/s - Slower turns for smoother motion
        self.max_acceleration = 2.0  # m/s² - Gentler acceleration
        
        # Navigation state
        self.current_waypoint = None
        self.waypoint_tolerance = 1.5  # meters - bigger tolerance for faster passes
        self.mission_complete = False
        
        # Safety parameters
        self.min_altitude = 0.5  # meters
        self.max_altitude = 10.0  # meters
        self.collision_radius = 0.3  # meters
        
        # Battery simulation (important for real deployment)
        self.battery_level = 100.0  # percentage
        # Battery lasts ~10 minutes at nominal load -> 10% per minute
        self.power_consumption_rate = 10.0  # %/minute
        
    def update(self, dt: float):
        """Update drone state (called every frame).
        
        NOTE: Position is updated by physics.update_drone() with wall checking.
        """
        # Update battery
        self.battery_level -= self.power_consumption_rate * dt / 60.0
        self.battery_level = max(0.0, self.battery_level)
        
        # Position update done in physics.update_drone() with wall checking
        
        # Update orientation
        self.orientation += self.angular_velocity * dt
        self.orientation = self._normalize_angle(self.orientation)
        
        # Apply altitude constraints
        self.position[2] = np.clip(self.position[2], self.min_altitude, self.max_altitude)
        
        # Check if waypoint reached
        if self.current_waypoint is not None:
            distance = self._distance_to_waypoint()
            if distance < self.waypoint_tolerance:
                self.current_waypoint = None
    
    def navigate_to(self, target: Tuple[float, float, float]):
        """Set navigation target waypoint."""
        self.current_waypoint = np.array(target, dtype=np.float32)
        
        # Calculate desired velocity vector
        direction = self.current_waypoint - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            # Normalize direction and apply speed limit
            direction_normalized = direction / distance
            desired_speed = min(self.max_speed, distance)  # Slow down when close
            self.velocity = direction_normalized * desired_speed
            
            # Calculate desired orientation (face movement direction)
            desired_yaw = math.atan2(direction[1], direction[0])
            yaw_error = self._normalize_angle(desired_yaw - self.orientation)
            
            # Simple P controller for orientation
            self.angular_velocity = np.clip(yaw_error * 2.0, 
                                          -self.max_angular_speed, 
                                          self.max_angular_speed)
    
    def emergency_stop(self):
        """Emergency stop - zero all velocities."""
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.angular_velocity = 0.0
        self.current_waypoint = None
    
    def set_altitude(self, altitude: float):
        """Set target altitude."""
        target_altitude = np.clip(altitude, self.min_altitude, self.max_altitude)
        self.navigate_to((self.position[0], self.position[1], target_altitude))
    
    def is_low_battery(self) -> bool:
        """Check if battery is critically low."""
        return self.battery_level < 20.0
    
    def get_status(self) -> dict:
        """Get current drone status for monitoring."""
        return {
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'orientation': self.orientation,
            'battery': self.battery_level,
            'waypoint': self.current_waypoint.tolist() if self.current_waypoint is not None else None,
            'mission_complete': self.mission_complete
        }
    
    def reset(self):
        """Reset drone to initial state."""
        # Restore original entry approach used at startup: near bottom door (25,-3,3)
        self.position = np.array([25, -3, 3], dtype=np.float32)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.orientation = 0.0
        self.angular_velocity = 0.0
        self.current_waypoint = None
        self.mission_complete = False
        self.battery_level = 100.0
    
    def _distance_to_waypoint(self) -> float:
        """Calculate distance to current waypoint."""
        if self.current_waypoint is None:
            return float('inf')
        return np.linalg.norm(self.current_waypoint - self.position)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

class FlightController:
    """
    Low-level flight control system.
    This would interface directly with motors/ESCs on real hardware.
    """
    
    def __init__(self):
        """Initialize flight controller."""
        # PID controllers for each axis
        self.pid_x = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.pid_y = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.pid_z = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.pid_yaw = PIDController(kp=2.0, ki=0.0, kd=0.1)
    
    def compute_motor_commands(self, drone: Drone, desired_position: np.ndarray, 
                             desired_yaw: float) -> Tuple[float, float, float, float]:
        """
        Compute motor commands for quadcopter.
        Returns (motor1, motor2, motor3, motor4) thrust values.
        """
        # Compute position errors
        pos_error = desired_position - drone.position
        yaw_error = drone._normalize_angle(desired_yaw - drone.orientation)
        
        # Compute PID outputs
        thrust_x = self.pid_x.update(pos_error[0])
        thrust_y = self.pid_y.update(pos_error[1])
        thrust_z = self.pid_z.update(pos_error[2])
        thrust_yaw = self.pid_yaw.update(yaw_error)
        
        # Convert to motor commands (simplified quadcopter mixing)
        base_thrust = 0.5 + thrust_z  # Hover at ~50% throttle + altitude adjustment
        
        # Motor layout: 1=front, 2=right, 3=back, 4=left
        motor1 = base_thrust + thrust_x - thrust_yaw
        motor2 = base_thrust + thrust_y + thrust_yaw
        motor3 = base_thrust - thrust_x - thrust_yaw
        motor4 = base_thrust - thrust_y + thrust_yaw
        
        # Clamp to valid range [0, 1]
        motors = [np.clip(m, 0.0, 1.0) for m in [motor1, motor2, motor3, motor4]]
        return tuple(motors)

class PIDController:
    """Simple PID controller for flight stabilization."""
    
    def __init__(self, kp: float, ki: float, kd: float):
        """Initialize PID controller with gains."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
    
    def update(self, error: float, dt: float = 0.016) -> float:
        """Update PID controller with current error."""
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        self.previous_error = error
        
        # Combine terms
        output = proportional + integral + derivative
        return output
    
    def reset(self):
        """Reset PID controller state."""
        self.previous_error = 0.0
        self.integral = 0.0
