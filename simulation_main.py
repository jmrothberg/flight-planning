#!/usr/bin/env python3
"""
Drone Navigation and Mapping System - Main Simulation Entry Point

=============================================================================
⚠️ CRITICAL REQUIREMENT FOR ALL FUTURE LLMs AND DEVELOPERS ⚠️
The building is COMPLETELY UNKNOWN! Search algorithms MUST NOT have ANY pre-knowledge:
- You can ONLY use information discovered through LIDAR sensors
- You CANNOT assume building dimensions, room layouts, or boundaries
- You CANNOT use predetermined paths or waypoints  
- You MUST discover walls, obstacles, and boundaries through exploration
- Any algorithm that "knows" the building layout beforehand is CHEATING!
=============================================================================

=============================================================================
MISSION GOALS:
1. Search an UNKNOWN building for IEDs (Improvised Explosive Devices)
2. IED detector has 3 METER range - MUST get within 3m to detect!
3. Must cover ALL accessible areas systematically
4. Must NOT get stuck or loop endlessly
5. Must complete search within 7 MINUTES (420 seconds)
6. Return to entry point when done or time expires
=============================================================================

Available Search Methods (change SEARCH_METHOD variable below):
- "systematic_sweep": NEW - Simple back-and-forth sweep like Roomba (reliable, won't oscillate)
- "wall_follower": BEST SO FAR - Follow left wall, 90%+ coverage but can loop
- "frontier_explorer": Frontier-based exploration (oscillates, needs work)
p

Controls:
- SPACE: Start/stop mission
- R: Reset simulation
- N: New object placement (same building)
- B: New building layout (5 realistic connected room configurations)
- P: Save screenshot
- ESC: Exit
"""

# SEARCH METHOD CONFIGURATION - CHANGE THIS LINE TO SWITCH ALGORITHMS
SEARCH_METHOD = "search_fast_ied_sweep"  # Options: "search_fast_ied_sweep" (AGGRESSIVE IED HUNTER), "search_coverage_boustrophedon" (FAST IED SWEEP), "systematic_sweep" (NEW), "wall_follower" (BEST SO FAR), "frontier_explorer", etc.

import pygame
import sys
import numpy as np
from typing import Tuple, List
import time
import math

from core.drone import Drone
from simulation.environment import Environment
from core.slam import SLAMSystem
from core.vision import VisionSystem
from core.communication import CommSystem, Message, MessagePriority
from core.sensors import IEDSensor
from core.minimap import MinimapSystem
from simulation.graphics import GraphicsEngine
from simulation.physics import PhysicsEngine
import os
import importlib

# Static fallback (will be overridden by dynamic discovery below)
from core.search_wall_follower import WallFollower as SearchAlgorithm
SEARCH_CONFIG = {"coverage_radius": 3.0}

class DroneSimulation:
    """Main simulation class that orchestrates all components."""
    
    def __init__(self, window_size: Tuple[int, int] = (1200, 900)):
        """Initialize the drone simulation system."""
        self.window_size = window_size
        self.running = True
        self.clock = pygame.time.Clock()
        
        # Initialize core systems
        self.environment = Environment()
        # Randomize mission objects each run for variability
        self.environment.randomize_objects()
        # Start OUTSIDE the building near front door gap (door ~ x=25 on bottom wall y=0)
        self.drone = Drone(position=(25, -3, 3))  # outside, facing upward (north)
        self.slam = SLAMSystem()
        self.vision = VisionSystem()
        self.comm = CommSystem()
        self.ied_sensor = IEDSensor()
        self.minimap = MinimapSystem()
        # Initialize with static fallback (will be overridden by dynamic discovery below)
        self.current_search_name = SEARCH_METHOD
        self.search_algorithm = SearchAlgorithm(**SEARCH_CONFIG)
        
        # Initialize SLAM and plan exploration
        self.slam.initialize(self.drone.position[:2], self.drone.orientation)
        # Don't plan exploration yet - wait until inside building
        
        # Laser target simulation - the door we need to enter through
        # This simulates a red laser pointing at the door
        self._laser_target = (25.0, 0.0)  # Door position (on the wall gap)
        self._entry_sequence = [
            (25.0, -0.5),  # Just outside the door
            (25.0, 0.5),   # Just inside the door
            (25.0, 2.0),   # Further inside to start exploration
        ]
        self._entry_index = 0
        self._inside_building = False
        self._start_time = time.time()  # Track mission time
        
        # Return-to-entry and inertial breadcrumbs
        self._returning_home = False
        self._breadcrumbs: List[Tuple[float, float]] = []
        self._last_breadcrumb_ts = time.time()
        # ADDED: watchdog for return-home progress using breadcrumb fallbacks
        self._return_progress_distance = None
        self._return_progress_ts = time.time()
        self._return_breadcrumb_offset = 0
        # ADDED: simple debug toggle for navigation prints (keep OFF to reduce noise)
        self._debug_nav = False
        # ADDED: counter to detect repeated A* failures
        self._astar_failures = 0
        self._astar_total_attempts = 0
        self._astar_total_failures = 0
        # Feature flag: breadcrumb-based return fallback (disabled to restore prior behavior)
        self._enable_breadcrumb_fallback = False
        # NEW: hide SLAM occupancy map by default to remove gray/black spray
        self._show_map = False

        # Initialize simulation engines
        self.graphics = GraphicsEngine(window_size)
        self.physics = PhysicsEngine()
        # Pass environment to physics for wall collision detection
        self.physics.environment = self.environment
        
        # Mission parameters
        self.mission_active = False
        self.exploration_complete = False
        
        # Sensor data storage
        self.lidar_data = None
        # Discover available search_* modules and enable dynamic default
        try:
            core_dir = os.path.join(os.path.dirname(__file__), 'core')
        except NameError:
            core_dir = os.path.join(os.getcwd(), 'core')
        self.search_options = []
        try:
            for fname in sorted(os.listdir(core_dir)):
                if fname.startswith('search_') and fname.endswith('.py'):
                    self.search_options.append(fname[:-3])
        except Exception:
            self.search_options = []

        # If we discovered search_* files, set the default to the FIRST in the list
        # and instantiate dynamically so new algorithms work automatically.
        if self.search_options:
            try:
                self.current_search_name = self.search_options[0]
                self.search_algorithm = self._load_search_by_module(self.current_search_name)
                print(f"Using search method: {self.current_search_name}")
            except Exception as _e:
                # Fall back to the statically imported SearchAlgorithm above
                pass

    def _load_search_by_module(self, module_name: str):
        """Load a search algorithm class from a core.search_* module by convention.
        The class is detected by scanning attributes for a callable with required methods.
        """
        module = importlib.import_module(f"core.{module_name}")
        # Preferred class names commonly used in this repo
        preferred = [
            'SystematicSweep',  # ADDED: Simple systematic sweep like Roomba
            'FrontierExplorer',  # Frontier-based exploration algorithm
            'SafeExplorer','FastIEDSweep','LidarMapper','WallFollower',
            'SimpleGridNavigator','CoverageNavigator','RandomWalkNavigator',
            'STCCoverage','AdaptiveRoomSweep'
        ]
        for name in preferred:
            if hasattr(module, name):
                cls = getattr(module, name)
                return self._instantiate_search_class(cls)
        # Fallback: pick the first class with a get_next_waypoint and reset
        for attr in dir(module):
            obj = getattr(module, attr)
            if callable(obj) and hasattr(obj, '__name__'):
                try:
                    if hasattr(obj, 'get_next_waypoint') or hasattr(obj, 'reset'):
                        # It's probably an instance, skip
                        continue
                except Exception:
                    pass
        # If none matched preferred names, try attributes that are classes
        for attr in dir(module):
            obj = getattr(module, attr)
            if isinstance(obj, type):
                if all(hasattr(obj, m) for m in ['get_next_waypoint','reset']):
                    return self._instantiate_search_class(obj)
        raise RuntimeError(f"No search class found in {module_name}")

    def _instantiate_search_class(self, cls):
        # Try common constructor signatures
        try:
            return cls(detection_radius=3.0)
        except Exception:
            pass
        try:
            return cls(coverage_radius=3.0)
        except Exception:
            pass
        try:
            return cls(sensor_range=10.0, sweep_spacing=2.5)
        except Exception:
            pass
        # Last resort: no-arg
        return cls()
        
    def run(self):
        """Main simulation loop."""
        print("Starting Drone Navigation and Mapping Simulation...")
        print("Controls: SPACE=start/stop, R=reset, N=new map, P=screenshot, ESC=exit")
        
        while self.running:
            dt = self.clock.tick(60) / 1000.0  # Delta time in seconds
            
            # Handle events
            self._handle_events()
            
            if self.mission_active:
                # Update core systems
                self._update_systems(dt)

                # Check mission completion
                if self.drone.mission_complete:
                    self.mission_active = False
                    print("Mission completed!")
                    print("Simulation stopped - Clock frozen at mission completion.")
                    print("Press SPACE to start new mission or ESC to exit")
                    self.running = False  # Stop the simulation loop
            
            # Render everything
            self._render()
            
        self._cleanup()
    
    def _handle_events(self):
        """Handle pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    # Start new mission or restart after completion
                    if self.drone.mission_complete:
                        self._reset_mission_keep_map()  # Reset for new mission but keep map
                        self.mission_active = True
                        print("Starting new mission with existing map...")
                    else:
                        self.mission_active = not self.mission_active
                        if self.mission_active:
                            print("Mission started - Beginning autonomous exploration...")
                        else:
                            print("Mission paused")
                elif event.key == pygame.K_r:
                    self._reset_simulation()
                elif event.key == pygame.K_n:
                    # New random object placement (same building)
                    print("Randomizing object placement...")
                    self.environment.randomize_objects()
                    # Reset mission to search for new IED location
                    self._reset_simulation()
                elif event.key == pygame.K_b:
                    # New building layout (different walls/rooms)
                    print("Generating new building layout...")
                    self.environment.generate_new_building()
                    # Reset everything for new building
                    self._reset_simulation()
                elif event.key == pygame.K_p:
                    # Save a PNG snapshot of the current GUI window
                    try:
                        self._save_screenshot()
                    except Exception as e:
                        print(f"Failed to save screenshot: {e}")
                # Number keys 1-9: switch search method dynamically (optional, not changing default)
                elif event.key in [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4, pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9]:
                    # Dynamic search switch at runtime (non-persistent)
                    idx = event.key - pygame.K_1
                    if 0 <= idx < len(self.search_options):
                        new_module = self.search_options[idx]
                        try:
                            self.search_algorithm = self._load_search_by_module(new_module)
                            self.current_search_name = new_module
                            print(f"Switched search to: {new_module}")
                        except Exception as e:
                            print(f"Failed to switch search: {e}")
    
    def _update_systems(self, dt: float):
        """Update all simulation systems."""
        # Update drone state (including battery consumption)
        self.drone.update(dt)
        
        # Update physics
        self.physics.update_drone(self.drone, dt)
        
        # Inertial breadcrumbs (dead-reckoning trail for return). In real system this
        # will be fed by the IMU/odometry. We only record 2D for path home.
        if (time.time() - self._last_breadcrumb_ts) >= 0.2:  # More frequent breadcrumbs for smoother trail
            self._breadcrumbs.append((float(self.drone.position[0]), float(self.drone.position[1])))
            if len(self._breadcrumbs) > 2000:
                self._breadcrumbs = self._breadcrumbs[-2000:]
            self._last_breadcrumb_ts = time.time()
        
        # Get sensor data
        self.lidar_data = self.environment.get_lidar_scan(self.drone.position, self.drone.orientation)
        camera_image = self.environment.get_camera_view(self.drone.position, self.drone.orientation)
        
        # Update SLAM with sensor data
        self.slam.update(self.drone.position[:2], self.lidar_data)
        
        # Update minimap with sensor data
        if self.lidar_data:
            self.minimap.add_lidar_scan(self.drone.position[:2], self.lidar_data, self.drone.orientation)
        
        # No need for exploration planning with wall following algorithm
        
        # Process vision data
        detected_objects = self.vision.process_frame(camera_image)
        
        # Add vision detections to minimap
        if detected_objects:
            for detection in detected_objects:
                self.minimap.add_vision_detection(
                    detection.position, 
                    detection.object_type.value if hasattr(detection.object_type, 'value') else str(detection.object_type),
                    detection.confidence
                )
        
        # Simulated IED sensor read (console + GUI + comms)
        ied_reading = self.ied_sensor.read((float(self.drone.position[0]), float(self.drone.position[1])), self.environment)
        if ied_reading and ied_reading.confidence > 0.2:
            text = ied_reading.to_text()
            print(f"IED ALERT: {text}")
            
            # Add IED detection to minimap
            self.minimap.add_ied_detection(
                (float(self.drone.position[0]), float(self.drone.position[1])),
                ied_reading.confidence
            )
            
            # Send as a high-priority message too
            self.comm.message_queue.put(Message(
                id=str(time.time()),
                priority=MessagePriority.HIGH,
                message_type="ied_alert",
                timestamp=time.time(),
                data={"text": text}
            ))
            # Flash radio banner for IED as well
            self.graphics.notify_radio_sent()

        # Update drone navigation
        # Entry sequence: follow laser to door, then enter building
        next_target_2d = None
        
        # First, follow the entry sequence to get inside - USE DIRECT NAVIGATION
        if not self._inside_building and self._entry_index < len(self._entry_sequence):
            target = self._entry_sequence[self._entry_index]
            dist_to_target = np.linalg.norm(np.array(target) - self.drone.position[:2])
            
            if dist_to_target < 0.8:
                self._entry_index += 1
                if self._entry_index >= len(self._entry_sequence):
                    # Now inside - start selected search algorithm
                    self._inside_building = True
                    self._start_time = time.time()
                    # Print the actual active search algorithm (introspect object to avoid stale name)
                    try:
                        mod = getattr(self.search_algorithm.__class__, '__module__', '')
                        active_name = mod.split('.')[-1] if mod else self.current_search_name
                    except Exception:
                        active_name = self.current_search_name
                    print(f"Entered building, beginning {active_name} search...")
            else:
                # DIRECT navigation for entry - no pathfinding needed outside
                waypoint_3d = (target[0], target[1], float(self.drone.position[2]))
                self.drone.navigate_to(waypoint_3d)
                next_target_2d = "handled"  # Mark as handled to skip pathfinding below
        
        # Exploration using selected search algorithm
        if next_target_2d is None and not self._returning_home:
            # OLD: Use wall follower to get next waypoint
            # next_target_2d = self.wall_follower.get_next_waypoint(
            #     self.drone.position[:2],
            #     self.lidar_data if hasattr(self, 'lidar_data') else [],
            #     self.drone.orientation
            # )
            # Use selected search algorithm for exploration
            next_target_2d = self.search_algorithm.get_next_waypoint(
                self.drone.position[:2],
                self.lidar_data if hasattr(self, 'lidar_data') else [],
                self.drone.orientation
            )
            if self._debug_nav and isinstance(next_target_2d, tuple):
                # concise debug: print waypoint and distance to it
                dx = next_target_2d[0] - float(self.drone.position[0])
                dy = next_target_2d[1] - float(self.drone.position[1])
                print(f"{SEARCH_METHOD} wp {next_target_2d} | d={np.hypot(dx,dy):.1f}m")
            
            # Return conditions: ONLY if IED found (high confidence) or 7 minutes elapsed
            ied_found = (self.ied_sensor.last_reading is not None and 
                         self.ied_sensor.last_reading.confidence >= 0.85)
            mission_elapsed = time.time() - self._start_time
            if ied_found or mission_elapsed > 420.0:  # 7 minutes = 420 seconds
                self._returning_home = True
                next_target_2d = self.slam.entry_point
                reason = "IED found" if ied_found else "time limit"
                progress = self.slam.get_exploration_progress()
                print(f"Returning to entry ({reason}).")
        elif next_target_2d is None and self._returning_home:
            # If returning, check early for completion before planning more moves
            dist_home_now = np.linalg.norm(np.array(self.slam.entry_point) - self.drone.position[:2])
            if (dist_home_now < 1.5 and self.drone.position[1] < 0.5) or self.drone.position[1] < -2.0:
                # Declare success and stop motion/spin
                self.drone.emergency_stop()
                self.drone.mission_complete = True
                self.mission_active = False
                print("Mission complete! Returned to entry point.")
                print("Press SPACE to start new mission with same map, N for new map")
                self.running = False  # Stop the simulation loop
                return
            next_target_2d = self.slam.entry_point
        
        # ADDED: if returning home, monitor progress; if stalled, optionally step along breadcrumbs
        if self._returning_home and self._enable_breadcrumb_fallback:
            dist_home_now = np.linalg.norm(np.array(self.slam.entry_point) - self.drone.position[:2])
            if self._return_progress_distance is None or dist_home_now < (self._return_progress_distance - 0.5):
                # Made progress of at least 0.5m: reset stall timer and offset
                self._return_progress_distance = float(dist_home_now)
                self._return_progress_ts = time.time()
                self._return_breadcrumb_offset = 0
            elif time.time() - self._return_progress_ts > 6.0 and len(self._breadcrumbs) > 10:
                # Stalled for >6s: pick an older breadcrumb as an intermediate waypoint
                self._return_breadcrumb_offset = min(self._return_breadcrumb_offset + 30, len(self._breadcrumbs) - 1)
                idx = max(0, len(self._breadcrumbs) - 1 - self._return_breadcrumb_offset)
                next_target_2d = self._breadcrumbs[idx]
                if self._debug_nav:
                    print("Return stalled - using breadcrumb fallback waypoint...")
                self._return_progress_ts = time.time()
        
        if next_target_2d and next_target_2d != "handled":
            # ALWAYS use A* pathfinding to avoid obstacles
            current_2d = (float(self.drone.position[0]), float(self.drone.position[1]))
            self._astar_total_attempts += 1
            path = self.slam.path_planner.get_path_to_next_point(current_2d, next_target_2d, self.environment)
            
            if path and len(path) > 0:
                # Take reasonable strides along path
                stride = 3 if self._inside_building else 5
                idx = min(len(path) - 1, stride)
                next_step = path[idx]
                
                waypoint_3d = (next_step[0], next_step[1], float(self.drone.position[2]))
                self.drone.navigate_to(waypoint_3d)
                if self._debug_nav:
                    print(f"A* step -> {next_step} | path_len={len(path)}")
                self._astar_failures = 0  # reset failure counter on success
            else:
                # ADDED: No path found. If returning home, optionally try a breadcrumb fallback step.
                if self._returning_home and self._enable_breadcrumb_fallback and len(self._breadcrumbs) > 10:
                    # Progressively step back through breadcrumbs until a valid one is found
                    self._astar_failures += 1
                    # Increase offset faster if many failures
                    step_back = 10 + min(50, 5 * self._astar_failures)
                    self._return_breadcrumb_offset = min(
                        self._return_breadcrumb_offset + step_back, len(self._breadcrumbs) - 1
                    )
                    # Find a valid breadcrumb
                    chosen = None
                    seek_idx = len(self._breadcrumbs) - 1 - self._return_breadcrumb_offset
                    while seek_idx >= 0:
                        candidate = self._breadcrumbs[seek_idx]
                        if self.environment.is_position_valid(candidate):
                            chosen = candidate
                            break
                        seek_idx -= 5
                    if chosen is None:
                        # Fall back to entry point if none valid
                        chosen = self.slam.entry_point
                    waypoint_3d = (chosen[0], chosen[1], float(self.drone.position[2]))
                    if self._debug_nav:
                        print(f"No A* path - navigating to breadcrumb fallback at idx={seek_idx} -> {chosen}...")
                    self.drone.navigate_to(waypoint_3d)
                else:
                    # A* failed to find path
                    self._astar_total_failures += 1
                    self._astar_failures += 1
                    
                    # Try to move directly toward target (bypass A* when it fails)
                    if next_target_2d:
                        direction_x = next_target_2d[0] - current_2d[0]
                        direction_y = next_target_2d[1] - current_2d[1]
                        distance = math.sqrt(direction_x**2 + direction_y**2)
                        if distance > 0:
                            # Move 1 meter toward target
                            move_dist = min(1.0, distance)
                            next_x = current_2d[0] + (direction_x / distance) * move_dist
                            next_y = current_2d[1] + (direction_y / distance) * move_dist
                            waypoint_3d = (next_x, next_y, float(self.drone.position[2]))
                            self.drone.navigate_to(waypoint_3d)
        elif next_target_2d != "handled":
            # Only check completion if we're actually done, not during entry
            if self._returning_home:
                dist_home = np.linalg.norm(np.array(self.slam.entry_point) - self.drone.position[:2])
                # More precise completion: must be near entry AND outside building
                if (dist_home < 1.5 and self.drone.position[1] < 0.5) or self.drone.position[1] < -2.0:
                    self.drone.emergency_stop()  # stop spinning
                    self.drone.mission_complete = True
                    self.mission_active = False  # Stop updates immediately
                    print("Mission complete! Returned to entry point.")
                    print("Press SPACE to start new mission with same map, N for new map")
                    self.running = False  # Stop the simulation loop
        
        # Send important findings via communication system
        if detected_objects:
            self.comm.send_priority_data(detected_objects, self.drone.position[:2])

        # If we just reached/changed exploration index (door crossing heuristic), send map update
        # Heuristic: when waypoint cleared (no current_waypoint) and exploration index advanced
        progress = self.slam.get_exploration_progress()
        if getattr(self, '_last_points_visited', None) is None:
            self._last_points_visited = progress['points_visited']
        if progress['points_visited'] > self._last_points_visited:
            self._last_points_visited = progress['points_visited']
            # Count and send a radio map update
            self.slam.room_exit_events += 1
            self.comm.send_map_update(self.slam.get_map())
            # Flash a banner in UI
            self.graphics.notify_radio_sent()
    
    def _render(self):
        """Render the simulation."""
        self.graphics.clear()
        
        # Render environment
        self.graphics.draw_environment(self.environment)
        
        # Draw laser target on door if not yet inside
        if not self._inside_building:
            laser_screen_pos = self.graphics._world_to_screen(self._laser_target)
            # Draw red laser target marker
            pygame.draw.circle(self.graphics.screen, (255, 0, 0), laser_screen_pos, 10, 2)
            pygame.draw.line(self.graphics.screen, (255, 0, 0), 
                           (laser_screen_pos[0]-15, laser_screen_pos[1]), 
                           (laser_screen_pos[0]+15, laser_screen_pos[1]), 2)
            pygame.draw.line(self.graphics.screen, (255, 0, 0),
                           (laser_screen_pos[0], laser_screen_pos[1]-15),
                           (laser_screen_pos[0], laser_screen_pos[1]+15), 2)
        
        # Render drone
        self.graphics.draw_drone(self.drone)
        
        # Render LIDAR rays (red) and IED detector rays (blue)
        if hasattr(self, 'lidar_data'):
            self.graphics.draw_lidar_scan(self.drone, self.lidar_data)
            
            # Draw IED detector rays - 3 meters range, 10 rays (every 36 degrees), blue color
            try:
                import math
                center = self.graphics._world_to_screen(self.drone.position[:2])
                ied_range_meters = 3.0  # IED detector range is 3 meters
                radius_px = int(ied_range_meters * self.graphics.scale)  # Convert to pixels
                num_rays = 10  # 360/36 = 10 rays (every 36 degrees)
                
                # Draw rays every 36 degrees in a full circle around the drone
                for i in range(num_rays):
                    angle = self.drone.orientation + (i * 2 * math.pi / num_rays)
                    end_x = center[0] + radius_px * math.cos(angle)
                    end_y = center[1] + radius_px * math.sin(angle)
                    end = (int(end_x), int(end_y))
                    # Draw blue ray for IED detector (thicker than LIDAR)
                    pygame.draw.line(self.graphics.screen, (0, 150, 255), center, end, 2)
                    # Draw small circle at end of each ray
                    pygame.draw.circle(self.graphics.screen, (0, 100, 255), end, 3)
            except Exception:
                pass
        
        # Render SLAM map only if enabled (hidden by default to avoid visual clutter)
        if self._show_map:
            self.graphics.draw_slam_map(self.slam.get_map())
        # Draw breadcrumbs as simple red dotted path for debugging
        if len(self._breadcrumbs) > 1:
            for i in range(1, len(self._breadcrumbs), 2):  # Show every 2nd point for more breadcrumbs
                if i < len(self._breadcrumbs):
                    pos = self.graphics._world_to_screen(self._breadcrumbs[i])
                    pygame.draw.circle(self.graphics.screen, (200, 0, 0), pos, 2)  # Small red dots
        
        # Render detected objects (throttle to reduce draw load)
        detected_objects = self.vision.get_recent_detections()
        if detected_objects:
            self.graphics.draw_detections(detected_objects[:50])
        
        # Render minimap showing discovered features
        minimap_data = self.minimap.get_minimap_data()
        # Add breadcrumbs to minimap data so they can be displayed
        if minimap_data:
            minimap_data["breadcrumbs"] = self._breadcrumbs
        self.graphics.draw_minimap(minimap_data, self.drone.position[:2])
        
        # Render UI (pass mission start time for timer). Keep timer visible after mission completion.
        self.graphics.draw_ui(
            self.drone,
            self.slam,
            self.comm,
            self._start_time,  # Always show timer if mission has started
            self.current_search_name,
            self.search_options,
        )

        # Overlay mission info: door points are noisy; only show when debug enabled
        if self._debug_nav:
            try:
                door_points = self.slam.path_planner.get_door_points(self.environment)
            except Exception:
                door_points = []
        else:
            door_points = []
        self.graphics.draw_overlay_mission(door_points, self.slam.get_exploration_progress())
        
        self.graphics.present()
    
    def _reset_simulation(self):
        """Reset the simulation to initial state."""
        print("Resetting simulation...")
        self.drone.reset()
        self.slam.reset()
        self.vision.reset()
        self.comm.reset()
        self.minimap.reset()
        # Reset search algorithm
        self.search_algorithm.reset()
        # Re-initialize SLAM but don't plan exploration yet
        self.slam.initialize(self.drone.position[:2], self.drone.orientation)
        self.mission_active = False
        self._returning_home = False
        self._breadcrumbs = []
        self._entry_index = 0
        self._inside_building = False
        self._start_time = time.time()
        self._astar_failures = 0
        self._astar_total_attempts = 0
        self._astar_total_failures = 0
    
    def _reset_mission_keep_map(self):
        """Reset for new mission but keep existing map."""
        print("Starting new mission with existing map...")
        # Reset drone to start position
        self.drone.reset()
        self.drone.position = np.array([25, -3, 3], dtype=np.float32)  # Back outside
        
        # Don't reset SLAM (keep the map), but reset exploration
        self.slam.current_exploration_index = 0
        # Don't plan exploration yet - wait until inside
        
        # Reset mission state
        self._returning_home = False
        self._breadcrumbs = []
        self._entry_index = 0
        self._inside_building = False
        self.vision.reset()
        # Don't reset minimap here - keep discovered map for new mission
        self.search_algorithm.reset()  # Reset search algorithm for new mission
        self.drone.mission_complete = False
        self._astar_failures = 0
        self._astar_total_attempts = 0
        self._astar_total_failures = 0
    
    def _cleanup(self):
        """Cleanup resources."""
        pygame.quit()
        print("Simulation ended.")

    def _save_screenshot(self):
        """Save a PNG of the current GUI window with time and search method in the filename."""
        # Build filename: search_<method>_MMSS.png
        elapsed = int(time.time() - self._start_time) if self._inside_building else 0
        mm = elapsed // 60
        ss = elapsed % 60
        method = self.current_search_name
        filename = f"screenshot_{method}_{mm:02d}{ss:02d}.png"
        try:
            pygame.image.save(self.graphics.screen, filename)
            print(f"Saved screenshot: {filename}")
        except Exception as e:
            print(f"Error saving screenshot: {e}")

def main():
    """Main entry point."""
    try:
        # Ensure pygame video system is initialized before creating the simulation window
        try:
            import pygame  # already imported above; local guard
            if not pygame.get_init():
                pygame.init()
            if not pygame.display.get_init():
                pygame.display.init()
        except Exception:
            pass
        simulation = DroneSimulation()
        simulation.run()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
