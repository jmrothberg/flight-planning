"""
Physics Engine for Drone Simulation
Handles realistic drone flight dynamics and physics simulation
"""

import numpy as np
import math
from typing import Tuple, List, Optional
from dataclasses import dataclass

@dataclass
class PhysicsState:
    """Physical state of the drone."""
    position: np.ndarray  # [x, y, z] in meters
    velocity: np.ndarray  # [vx, vy, vz] in m/s
    acceleration: np.ndarray  # [ax, ay, az] in m/s²
    orientation: np.ndarray  # [roll, pitch, yaw] in radians
    angular_velocity: np.ndarray  # [wx, wy, wz] in rad/s
    angular_acceleration: np.ndarray  # [ax, ay, az] in rad/s²

class DronePhysics:
    """
    Realistic drone physics model.
    Simulates quadcopter dynamics with proper flight characteristics.
    """
    
    def __init__(self):
        """Initialize drone physics parameters."""
        # Physical properties
        self.mass = 1.5  # kg
        self.arm_length = 0.25  # meters from center to motor
        self.inertia_xx = 0.029  # kg*m²
        self.inertia_yy = 0.029  # kg*m²
        self.inertia_zz = 0.055  # kg*m²
        
        # Motor properties
        self.motor_thrust_constant = 8.54e-6  # N/(rad/s)²
        self.motor_torque_constant = 1.6e-7   # Nm/(rad/s)²
        self.max_motor_speed = 1000  # rad/s
        
        # Aerodynamic properties
        self.drag_coefficient = 0.01
        self.air_density = 1.225  # kg/m³
        
        # Environmental constants
        self.gravity = 9.81  # m/s²
        
        # Control limits
        self.max_tilt_angle = math.radians(30)  # 30 degrees
        self.max_thrust = 4 * self.motor_thrust_constant * self.max_motor_speed**2
    
    def compute_forces_and_torques(self, motor_speeds: np.ndarray, 
                                  state: PhysicsState) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forces and torques from motor speeds and current state.
        
        Args:
            motor_speeds: Array of 4 motor speeds [front, right, back, left] in rad/s
            state: Current physics state
            
        Returns:
            Tuple of (forces, torques) in body frame
        """
        # Ensure motor speeds are within limits
        motor_speeds = np.clip(motor_speeds, 0, self.max_motor_speed)
        
        # Compute individual motor thrusts and torques
        thrusts = self.motor_thrust_constant * motor_speeds**2
        motor_torques = self.motor_torque_constant * motor_speeds**2
        
        # Total thrust (upward in body frame)
        total_thrust = np.sum(thrusts)
        forces = np.array([0, 0, total_thrust])
        
        # Compute torques around each axis
        # Roll torque (around x-axis): right motors - left motors
        roll_torque = self.arm_length * (thrusts[1] - thrusts[3])
        
        # Pitch torque (around y-axis): back motors - front motors  
        pitch_torque = self.arm_length * (thrusts[2] - thrusts[0])
        
        # Yaw torque (around z-axis): CCW motors - CW motors
        # Assuming motors 0,2 are CW and motors 1,3 are CCW
        yaw_torque = motor_torques[1] + motor_torques[3] - motor_torques[0] - motor_torques[2]
        
        torques = np.array([roll_torque, pitch_torque, yaw_torque])
        
        return forces, torques
    
    def compute_aerodynamic_forces(self, state: PhysicsState) -> np.ndarray:
        """Compute aerodynamic drag forces."""
        velocity_magnitude = np.linalg.norm(state.velocity)
        if velocity_magnitude < 0.01:
            return np.zeros(3)
        
        # Drag force opposes velocity direction
        drag_force_magnitude = 0.5 * self.air_density * self.drag_coefficient * velocity_magnitude**2
        drag_direction = -state.velocity / velocity_magnitude
        
        return drag_force_magnitude * drag_direction
    
    def body_to_world_frame(self, body_vector: np.ndarray, orientation: np.ndarray) -> np.ndarray:
        """Transform vector from body frame to world frame."""
        roll, pitch, yaw = orientation
        
        # Rotation matrices
        R_x = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
        
        R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        
        R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])
        
        # Combined rotation matrix (Z-Y-X order)
        R = R_z @ R_y @ R_x
        
        return R @ body_vector
    
    def world_to_body_frame(self, world_vector: np.ndarray, orientation: np.ndarray) -> np.ndarray:
        """Transform vector from world frame to body frame."""
        roll, pitch, yaw = orientation
        
        # Inverse rotation (transpose of rotation matrix)
        R_x = np.array([[1, 0, 0],
                       [0, math.cos(roll), math.sin(roll)],
                       [0, -math.sin(roll), math.cos(roll)]])
        
        R_y = np.array([[math.cos(pitch), 0, -math.sin(pitch)],
                       [0, 1, 0],
                       [math.sin(pitch), 0, math.cos(pitch)]])
        
        R_z = np.array([[math.cos(yaw), math.sin(yaw), 0],
                       [-math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])
        
        # Combined inverse rotation matrix
        R_inv = R_x @ R_y @ R_z
        
        return R_inv @ world_vector

class PhysicsEngine:
    """
    Complete physics simulation engine for drone flight.
    Handles integration and collision detection.
    """
    
    def __init__(self):
        """Initialize physics engine."""
        self.drone_physics = DronePhysics()
        self.integration_method = "rk4"  # Runge-Kutta 4th order
        
        # Collision detection
        self.ground_level = 0.0
        self.collision_tolerance = 0.1
        self.wall_collision_radius = 0.2  # Drone radius for wall collisions (0.2m square drone, 0.2m wall clearance)
        
        # Wind simulation
        self.wind_enabled = False
        self.wind_velocity = np.array([0.0, 0.0, 0.0])  # m/s
        self.wind_turbulence = 0.1  # Random turbulence factor
        
        # Environment reference (set by main simulation)
        self.environment = None
    
    def update_drone(self, drone, dt: float):
        """Update drone physics for one time step."""
        # SIMPLE POSITION UPDATE with wall checking
        # Calculate where drone WOULD move to
        new_x = drone.position[0] + drone.velocity[0] * dt
        new_y = drone.position[1] + drone.velocity[1] * dt
        
        # Check if new position is valid (not inside wall)
        if self.environment is not None:
            if self.environment.is_position_valid((new_x, new_y), radius=0.2):
                # Position is valid - update (0.35m = drone body 0.3m + small buffer)
                drone.position[0] = new_x
                drone.position[1] = new_y
            else:
                # Would hit wall - stop velocity
                drone.velocity[0] = 0
                drone.velocity[1] = 0
        else:
            # No environment to check - just update position
            drone.position[0] = new_x
            drone.position[1] = new_y
        
        # Handle collisions (push back if too close to walls) and constraints
        self._handle_collisions(drone)
        self._apply_constraints(drone)
    
    def _get_motor_commands_from_drone(self, drone) -> np.ndarray:
        """
        Convert drone's high-level control to motor speeds.
        Simplified for 2D simulation.
        """
        # Base hovering speed
        hover_speed = math.sqrt(self.drone_physics.mass * self.drone_physics.gravity / 
                               (4 * self.drone_physics.motor_thrust_constant))
        
        # Calculate desired forces for current velocity
        thrust_factor = 1.0
        if np.linalg.norm(drone.velocity) > 0.1:
            thrust_factor = 1.2  # Extra thrust when moving
        
        base_speed = hover_speed * thrust_factor
        
        # Yaw control
        yaw_correction = drone.angular_velocity * 50  # Simple proportional control
        
        # Motor mixing (simplified quadcopter)
        motor1 = base_speed - yaw_correction  # Front (CW)
        motor2 = base_speed + yaw_correction  # Right (CCW) 
        motor3 = base_speed - yaw_correction  # Back (CW)
        motor4 = base_speed + yaw_correction  # Left (CCW)
        
        return np.array([motor1, motor2, motor3, motor4])
    
    def _integrate_physics(self, state: PhysicsState, motor_speeds: np.ndarray, dt: float) -> PhysicsState:
        """Integrate physics equations using selected method."""
        if self.integration_method == "rk4":
            return self._rk4_integration(state, motor_speeds, dt)
        else:
            return self._euler_integration(state, motor_speeds, dt)
    
    def _euler_integration(self, state: PhysicsState, motor_speeds: np.ndarray, dt: float) -> PhysicsState:
        """Simple Euler integration."""
        # Compute forces and torques
        body_forces, body_torques = self.drone_physics.compute_forces_and_torques(motor_speeds, state)
        
        # Transform forces to world frame
        world_forces = self.drone_physics.body_to_world_frame(body_forces, state.orientation)
        
        # Add gravity and aerodynamic forces
        gravity_force = np.array([0, 0, -self.drone_physics.mass * self.drone_physics.gravity])
        aero_forces = self.drone_physics.compute_aerodynamic_forces(state)
        wind_forces = self._compute_wind_forces(state)
        
        total_forces = world_forces + gravity_force + aero_forces + wind_forces
        
        # Compute accelerations
        linear_acceleration = total_forces / self.drone_physics.mass
        
        # Angular acceleration (simplified for 2D)
        angular_acceleration = np.zeros(3)
        angular_acceleration[2] = body_torques[2] / self.drone_physics.inertia_zz
        
        # Integrate
        new_state = PhysicsState(
            position=state.position + state.velocity * dt,
            velocity=state.velocity + linear_acceleration * dt,
            acceleration=linear_acceleration,
            orientation=state.orientation + state.angular_velocity * dt,
            angular_velocity=state.angular_velocity + angular_acceleration * dt,
            angular_acceleration=angular_acceleration
        )
        
        return new_state
    
    def _rk4_integration(self, state: PhysicsState, motor_speeds: np.ndarray, dt: float) -> PhysicsState:
        """4th order Runge-Kutta integration for better accuracy."""
        def compute_derivatives(s):
            body_forces, body_torques = self.drone_physics.compute_forces_and_torques(motor_speeds, s)
            world_forces = self.drone_physics.body_to_world_frame(body_forces, s.orientation)
            
            gravity_force = np.array([0, 0, -self.drone_physics.mass * self.drone_physics.gravity])
            aero_forces = self.drone_physics.compute_aerodynamic_forces(s)
            wind_forces = self._compute_wind_forces(s)
            
            total_forces = world_forces + gravity_force + aero_forces + wind_forces
            linear_acceleration = total_forces / self.drone_physics.mass
            
            angular_acceleration = np.zeros(3)
            angular_acceleration[2] = body_torques[2] / self.drone_physics.inertia_zz
            
            return {
                'pos_dot': s.velocity,
                'vel_dot': linear_acceleration,
                'ori_dot': s.angular_velocity,
                'ang_vel_dot': angular_acceleration
            }
        
        # RK4 steps
        k1 = compute_derivatives(state)
        
        state_k2 = PhysicsState(
            position=state.position + 0.5 * dt * k1['pos_dot'],
            velocity=state.velocity + 0.5 * dt * k1['vel_dot'],
            acceleration=state.acceleration,
            orientation=state.orientation + 0.5 * dt * k1['ori_dot'],
            angular_velocity=state.angular_velocity + 0.5 * dt * k1['ang_vel_dot'],
            angular_acceleration=state.angular_acceleration
        )
        k2 = compute_derivatives(state_k2)
        
        state_k3 = PhysicsState(
            position=state.position + 0.5 * dt * k2['pos_dot'],
            velocity=state.velocity + 0.5 * dt * k2['vel_dot'],
            acceleration=state.acceleration,
            orientation=state.orientation + 0.5 * dt * k2['ori_dot'],
            angular_velocity=state.angular_velocity + 0.5 * dt * k2['ang_vel_dot'],
            angular_acceleration=state.angular_acceleration
        )
        k3 = compute_derivatives(state_k3)
        
        state_k4 = PhysicsState(
            position=state.position + dt * k3['pos_dot'],
            velocity=state.velocity + dt * k3['vel_dot'],
            acceleration=state.acceleration,
            orientation=state.orientation + dt * k3['ori_dot'],
            angular_velocity=state.angular_velocity + dt * k3['ang_vel_dot'],
            angular_acceleration=state.angular_acceleration
        )
        k4 = compute_derivatives(state_k4)
        
        # Combine results
        new_state = PhysicsState(
            position=state.position + dt/6 * (k1['pos_dot'] + 2*k2['pos_dot'] + 2*k3['pos_dot'] + k4['pos_dot']),
            velocity=state.velocity + dt/6 * (k1['vel_dot'] + 2*k2['vel_dot'] + 2*k3['vel_dot'] + k4['vel_dot']),
            acceleration=k1['vel_dot'],  # Current acceleration
            orientation=state.orientation + dt/6 * (k1['ori_dot'] + 2*k2['ori_dot'] + 2*k3['ori_dot'] + k4['ori_dot']),
            angular_velocity=state.angular_velocity + dt/6 * (k1['ang_vel_dot'] + 2*k2['ang_vel_dot'] + 2*k3['ang_vel_dot'] + k4['ang_vel_dot']),
            angular_acceleration=k1['ang_vel_dot']  # Current angular acceleration
        )
        
        return new_state
    
    def _compute_wind_forces(self, state: PhysicsState) -> np.ndarray:
        """Compute wind forces affecting the drone."""
        if not self.wind_enabled:
            return np.zeros(3)
        
        # Relative wind velocity
        relative_wind = self.wind_velocity - state.velocity
        
        # Add turbulence
        turbulence = np.random.normal(0, self.wind_turbulence, 3)
        relative_wind += turbulence
        
        # Wind force proportional to relative velocity squared
        wind_magnitude = np.linalg.norm(relative_wind)
        if wind_magnitude < 0.01:
            return np.zeros(3)
        
        wind_direction = relative_wind / wind_magnitude
        wind_force_magnitude = 0.5 * self.drone_physics.air_density * 0.1 * wind_magnitude**2  # Simplified
        
        return wind_force_magnitude * wind_direction
    
    def _handle_collisions(self, drone):
        """Handle collisions with ground and obstacles."""
        # Ground collision
        if drone.position[2] < self.ground_level + self.collision_tolerance:
            drone.position[2] = self.ground_level + self.collision_tolerance
            if drone.velocity[2] < 0:
                drone.velocity[2] = 0  # Stop downward motion
        
        # CRITICAL: Check if current position is valid - if not, stop immediately
        if self.environment is not None:
            drone_pos = (float(drone.position[0]), float(drone.position[1]))
            
            # If drone is in an invalid position (inside wall), it passed through!
            if not self.environment.is_position_valid(drone_pos, radius=0.2):
                # STOP! Reset velocity and try to find valid position nearby
                drone.velocity[0] = 0
                drone.velocity[1] = 0
                
                # Search for nearby valid position
                for radius in [0.5, 1.0, 1.5, 2.0, 3.0]:
                    found = False
                    for angle in range(0, 360, 30):
                        rad = math.radians(angle)
                        test_x = drone.position[0] + radius * math.cos(rad)
                        test_y = drone.position[1] + radius * math.sin(rad)
                        if self.environment.is_position_valid((test_x, test_y), radius=0.2):
                            drone.position[0] = test_x
                            drone.position[1] = test_y
                            found = True
                            break
                    if found:
                        break
            
            # Standard wall collision push-back
            drone_x, drone_y = drone.position[0], drone.position[1]
            for wall in self.environment.walls:
                dist = self._point_to_line_distance((drone_x, drone_y), wall)
                
                if dist < self.wall_collision_radius:
                    push_dir = self._get_perpendicular_from_wall((drone_x, drone_y), wall)
                    push_distance = self.wall_collision_radius - dist + 0.1
                    drone.position[0] += push_dir[0] * push_distance
                    drone.position[1] += push_dir[1] * push_distance
                    
                    wall_normal = push_dir
                    velocity_toward_wall = (drone.velocity[0] * wall_normal[0] + 
                                          drone.velocity[1] * wall_normal[1])
                    if velocity_toward_wall < 0:
                        drone.velocity[0] -= velocity_toward_wall * wall_normal[0]
                        drone.velocity[1] -= velocity_toward_wall * wall_normal[1]
    
    def _apply_constraints(self, drone):
        """Apply physical constraints to drone state."""
        # Altitude limits
        drone.position[2] = max(drone.min_altitude, 
                               min(drone.max_altitude, drone.position[2]))
        
        # Velocity limits
        velocity_magnitude = np.linalg.norm(drone.velocity)
        if velocity_magnitude > drone.max_speed:
            drone.velocity = drone.velocity / velocity_magnitude * drone.max_speed
        
        # Angular velocity limits
        drone.angular_velocity = np.clip(drone.angular_velocity, 
                                       -drone.max_angular_speed, 
                                       drone.max_angular_speed)
    
    def set_wind(self, wind_velocity: Tuple[float, float, float], turbulence: float = 0.1):
        """Set wind parameters."""
        self.wind_enabled = True
        self.wind_velocity = np.array(wind_velocity)
        self.wind_turbulence = turbulence
    
    def disable_wind(self):
        """Disable wind simulation."""
        self.wind_enabled = False
        self.wind_velocity = np.zeros(3)
    
    def _point_to_line_distance(self, point: Tuple[float, float], 
                                line: List[Tuple[float, float]]) -> float:
        """Calculate minimum distance from point to line segment."""
        x0, y0 = point
        x1, y1 = line[0]
        x2, y2 = line[1]
        
        # Vector from line start to point
        dx = x0 - x1
        dy = y0 - y1
        
        # Line segment vector
        sx = x2 - x1
        sy = y2 - y1
        
        # Line segment length squared
        seg_length_sq = sx * sx + sy * sy
        
        if seg_length_sq < 1e-10:  # Line segment is a point
            return math.sqrt(dx * dx + dy * dy)
        
        # Parameter t for closest point on line (0 <= t <= 1 for segment)
        t = max(0, min(1, (dx * sx + dy * sy) / seg_length_sq))
        
        # Closest point on segment
        closest_x = x1 + t * sx
        closest_y = y1 + t * sy
        
        # Distance to closest point
        dist_x = x0 - closest_x
        dist_y = y0 - closest_y
        
        return math.sqrt(dist_x * dist_x + dist_y * dist_y)
    
    def _get_perpendicular_from_wall(self, point: Tuple[float, float],
                                    wall: List[Tuple[float, float]]) -> Tuple[float, float]:
        """Get unit vector perpendicular to wall, pointing away from it."""
        x0, y0 = point
        x1, y1 = wall[0]
        x2, y2 = wall[1]
        
        # Wall vector
        wx = x2 - x1
        wy = y2 - y1
        wall_length = math.sqrt(wx * wx + wy * wy)
        
        if wall_length < 1e-10:
            return (1.0, 0.0)  # Default direction if wall is degenerate
        
        # Normalize wall vector
        wx /= wall_length
        wy /= wall_length
        
        # Get perpendicular (rotate 90 degrees)
        # Two possible perpendiculars: (-wy, wx) and (wy, -wx)
        perp1 = (-wy, wx)
        perp2 = (wy, -wx)
        
        # Choose the one pointing away from the wall
        # Project point onto wall line
        t = max(0, min(1, ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / (wall_length * wall_length)))
        closest_x = x1 + t * (x2 - x1)
        closest_y = y1 + t * (y2 - y1)
        
        # Vector from closest point on wall to drone
        to_drone_x = x0 - closest_x
        to_drone_y = y0 - closest_y
        
        # Choose perpendicular that aligns with this direction
        dot1 = to_drone_x * perp1[0] + to_drone_y * perp1[1]
        dot2 = to_drone_x * perp2[0] + to_drone_y * perp2[1]
        
        return perp1 if dot1 > dot2 else perp2
