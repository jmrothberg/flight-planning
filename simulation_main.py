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
- D: Cycle drone count (1-4) for multi-drone mode
- +/=: Increase communication range
- -: Decrease communication range
- P: Save screenshot
- ESC: Exit
"""

import pygame
import sys
import numpy as np
from typing import Tuple, List, Optional
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
# Multi-drone support
from core.drone_manager import DroneManager
import os
import importlib
import argparse
import json

# SEARCH METHOD CONFIGURATION - LOADED FROM algorithm_config.py
from algorithm_config import ALGORITHM as SEARCH_METHOD

# AUTO-RUN CONFIGURATION (for automated testing)
AUTO_START = True  # Auto-start mission on launch
AUTO_SCREENSHOT_INTERVAL = 0  # Screenshot interval in simulated seconds
AUTO_EXIT_TIME = 50  # Wall-clock exit time
SIM_SPEED = 3.0  # Simulation speed multiplier (reduced for smooth flight)

# Dynamic search algorithm loading (no static fallback)
SEARCH_CONFIG = {"coverage_radius": 3.0}

class DroneSimulation:
    """Main simulation class that orchestrates all components."""

    def __init__(self, window_size: Tuple[int, int] = (1200, 1300)):
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
        # Initialize search algorithm dynamically
        self.current_search_name = SEARCH_METHOD
        self.search_algorithm = None  # Will be set by dynamic loading below
        
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
        # ADDED: simple debug toggle for navigation prints (OFF for clean output)
        self._debug_nav = False
        # ADDED: counter to detect repeated A* failures
        self._astar_failures = 0
        self._astar_total_attempts = 0
        self._astar_total_failures = 0
        # Feature flag: breadcrumb-based return fallback (disabled to restore prior behavior)
        self._enable_breadcrumb_fallback = False
        # NEW: hide SLAM occupancy map by default to remove gray/black spray
        self._show_map = False
        
        # MULTI-DRONE SUPPORT
        self.drone_count = 1  # Number of drones (1-4)
        self.comm_range = 10.0  # Communication range in meters
        self.drone_manager: Optional[DroneManager] = None  # Created when drone_count > 1
        self.multi_drone_mode = False  # Flag for multi-drone operation

        # Initialize simulation engines
        self.graphics = GraphicsEngine(window_size)
        self.physics = PhysicsEngine()
        # Pass environment to physics for wall collision detection
        self.physics.environment = self.environment
        
        # Mission parameters
        self.mission_active = False
        self.exploration_complete = False
        
        # AUTO-RUN: Auto-start mission if enabled
        if AUTO_START:
            self.mission_active = True
            print("AUTO-START: Mission starting automatically...")
        
        # AUTO-SCREENSHOT: Track for periodic screenshots
        self._auto_screenshot_last = time.time()
        self._auto_start_time = time.time()
        
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

        # Use SEARCH_METHOD variable exclusively
        try:
            self.search_algorithm = self._load_search_by_module(SEARCH_METHOD)
            self.current_search_name = SEARCH_METHOD
            print(f"Using search method: {self.current_search_name}")
        except Exception as e:
            print(f"Failed to load {SEARCH_METHOD}: {e}")
            # Emergency fallback - load wall_follower
            try:
                self.search_algorithm = self._load_search_by_module("search_wall_follower")
                self.current_search_name = "search_wall_follower"
                print(f"Emergency fallback to: {self.current_search_name}")
            except Exception as e2:
                print(f"CRITICAL ERROR: No search algorithm available: {e2}")
                sys.exit(1)

    def _load_search_by_module(self, module_name: str):
        """Load a search algorithm class from a core.search_* module by convention.
        The class is detected by scanning attributes for a callable with required methods.
        """
        module = importlib.import_module(f"core.{module_name}")
        # Preferred class names commonly used in this repo
        preferred = [
            'SystematicMapper',  # NEW: Occupancy-based systematic coverage
            'AutoSearch',  # AI-iterated search algorithm
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
            dt = self.clock.tick(60) / 1000.0 * SIM_SPEED  # Delta time in seconds (scaled by speed)
            
            # Handle events
            self._handle_events()
            
            # Check if 7-minute mission time reached (420 seconds simulated)
            if self.mission_active and self._start_time is not None:
                simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED
                if simulated_elapsed >= 420.0 and not getattr(self, '_time_limit_reached', False):
                    self._time_limit_reached = True  # Only trigger once
                    print("=" * 50)
                    print(f"TIME LIMIT! 7 minutes reached ({simulated_elapsed:.0f}s simulated)")
                    cov = self.search_algorithm.get_coverage_stats().get('coverage_pct', 0)
                    print(f"Final coverage: {cov}%")
                    print("=" * 50)
                    self._save_screenshot("final_7min")  # Save final screenshot
                    self.mission_active = False  # Stop the mission
                    self.drone.emergency_stop()  # Stop drone movement
                    print("Press SPACE to start new mission, R to reset, ESC to exit")
                    # DON'T exit - wait for user input

            # AUTO-SCREENSHOT: Take mid-mission screenshot at 3.5 minutes simulated
            if not hasattr(self, '_mid_screenshot_taken'):
                self._mid_screenshot_taken = False
            if self.mission_active and self._start_time is not None and not self._mid_screenshot_taken:
                simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED
                if simulated_elapsed >= 210.0:  # Half of 7 minutes
                    self._save_screenshot("mid_3_5min")
                    self._mid_screenshot_taken = True
            
            if self.mission_active:
                # Update core systems
                self._update_systems(dt)

                # Check mission completion
                all_complete = False
                if self.multi_drone_mode and self.drone_manager:
                    # Multi-drone: check if ALL drones are complete
                    all_complete = all(d.mission_complete for d in self.drone_manager.drones)
                else:
                    # Single drone
                    all_complete = self.drone.mission_complete
                
                if all_complete:
                    self.mission_active = False
                    print("=" * 50)
                    print("MISSION COMPLETE! All drones returned HOME.")
                    print("=" * 50)
                    self._save_screenshot("final_complete")  # Save final screenshot
                    print("Press SPACE to start new mission, R to reset, ESC to exit")
                    # DON'T exit - wait for user input

            # Render everything
            self._render()
            
        self._cleanup()
    
    def _take_periodic_screenshot(self):
        """Take periodic screenshots in headless mode."""
        if not hasattr(self, '_last_screenshot_time'):
            self._last_screenshot_time = time.time()

        current_time = time.time()
        if current_time - self._last_screenshot_time >= AUTO_SCREENSHOT_INTERVAL:
            self._save_screenshot("periodic")
            self._last_screenshot_time = current_time

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
                elif event.key == pygame.K_d:
                    # Cycle drone count (1 -> 2 -> 3 -> 4 -> 1)
                    # DON'T reset - just add/remove drones while preserving current state
                    old_count = self.drone_count
                    self.drone_count = (self.drone_count % 4) + 1
                    print(f"Drone count: {old_count} -> {self.drone_count}")
                    self._setup_multi_drone(preserve_drone_0=True)
                elif event.key in [pygame.K_PLUS, pygame.K_EQUALS]:
                    # Increase communication range
                    self.comm_range = min(50.0, self.comm_range + 2.0)
                    print(f"Comm range: {self.comm_range:.0f}m")
                    if self.drone_manager:
                        self.drone_manager.set_comm_range(self.comm_range)
                elif event.key == pygame.K_MINUS:
                    # Decrease communication range
                    self.comm_range = max(2.0, self.comm_range - 2.0)
                    print(f"Comm range: {self.comm_range:.0f}m")
                    if self.drone_manager:
                        self.drone_manager.set_comm_range(self.comm_range)
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
        # Multi-drone mode: delegate to multi-drone update
        if self.multi_drone_mode and self.drone_manager:
            self._update_multi_drone(dt)
            # Still update drone 0's reference for compatibility with single-drone render code
            if self.drone_manager.drones:
                self.drone = self.drone_manager.drones[0]
            return
        
        # Single drone mode (original logic)
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
            # V11.5g: Pass SIMULATED time to search algorithm (critical for return timing)
            simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED
            if hasattr(self.search_algorithm, 'set_simulated_time'):
                self.search_algorithm.set_simulated_time(simulated_elapsed)
            
            # Use selected search algorithm for exploration
            next_target_2d = self.search_algorithm.get_next_waypoint(
                self.drone.position[:2],
                self.lidar_data if hasattr(self, 'lidar_data') else [],
                self.drone.orientation
            )
            
            # Check if search algorithm reports completion
            search_complete = False
            if hasattr(self.search_algorithm, 'is_mission_complete'):
                search_complete = self.search_algorithm.is_mission_complete()
            
            # Return is handled by search algorithm using simulated time
            # This is just a fallback for search_complete
            if search_complete:
                self._returning_home = True
                next_target_2d = self.slam.entry_point
                if not getattr(self, '_return_msg_printed', False):
                    print(f"Search complete, returning to entry.")
                    self._return_msg_printed = True
        elif next_target_2d is None and self._returning_home:
            # If returning, check early for completion before planning more moves
            dist_home_now = np.linalg.norm(np.array(self.slam.entry_point) - self.drone.position[:2])
            if (dist_home_now < 1.5 and self.drone.position[1] < 0.5) or self.drone.position[1] < -2.0:
                # Declare success and stop motion/spin
                self.drone.emergency_stop()
                self.drone.mission_complete = True
                self.mission_active = False
                print("=" * 50)
                print("HOME! Mission complete - drone returned to start.")
                print("=" * 50)
                self._save_screenshot("mission_complete")
                print("Press SPACE to start new mission, R to reset, ESC to exit")
                # DON'T exit - wait for user input
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
                # Take SMALL strides along path for smooth movement
                stride = 1 if self._inside_building else 2  # Much smaller steps
                idx = min(len(path) - 1, stride)
                next_step = path[idx]
                
                # WALL CHECK: Verify path doesn't pass through wall before navigating
                if self.environment.is_position_valid((next_step[0], next_step[1])):
                    waypoint_3d = (next_step[0], next_step[1], float(self.drone.position[2]))
                    self.drone.navigate_to(waypoint_3d)
                    if self._debug_nav:
                        print(f"A* step -> {next_step} | path_len={len(path)}")
                    self._astar_failures = 0  # reset failure counter on success
                else:
                    # A* gave us a path through wall - treat as A* failure
                    self._astar_total_failures += 1
                    self._astar_failures += 1
                    if self._debug_nav:
                        print(f"A* path goes through wall at {next_step}!")
            else:
                # A* failed - use DIRECT navigation toward target
                self._astar_total_failures += 1
                self._astar_failures += 1
                
                if self._debug_nav:
                    print(f"A* FAILED! Target={next_target_2d}, trying direct navigation...")
                
                # Calculate direction to target
                dx = next_target_2d[0] - current_2d[0]
                dy = next_target_2d[1] - current_2d[1]
                dist = np.hypot(dx, dy)
                moved = False
                
                if dist > 0.3:
                    step_size = min(0.8, dist)  # Smaller steps (was 1.5) to avoid passing through walls
                    
                    # Try multiple directions: toward target, then perpendicular, then others
                    angles_to_try = [
                        np.arctan2(dy, dx),  # Direct to target
                        np.arctan2(dy, dx) + np.pi/4,
                        np.arctan2(dy, dx) - np.pi/4,
                        np.arctan2(dy, dx) + np.pi/2,
                        np.arctan2(dy, dx) - np.pi/2,
                        self.drone.orientation,  # Current heading
                        self.drone.orientation + np.pi/4,
                        self.drone.orientation - np.pi/4,
                    ]
                    
                    for angle in angles_to_try:
                        nx = current_2d[0] + step_size * np.cos(angle)
                        ny = current_2d[1] + step_size * np.sin(angle)
                        # WALL CHECK: verify position is valid before navigating
                        if self.environment.is_position_valid((nx, ny)):
                            waypoint_3d = (nx, ny, float(self.drone.position[2]))
                            self.drone.navigate_to(waypoint_3d)
                            moved = True
                            if self._debug_nav:
                                print(f"Direct nav -> ({nx:.1f}, {ny:.1f})")
                            break
                
                if not moved and self._debug_nav:
                    print(f"WARNING: Could not find ANY valid move direction!")
        elif next_target_2d != "handled":
            # Only check completion if we're actually done, not during entry
            if self._returning_home:
                dist_home = np.linalg.norm(np.array(self.slam.entry_point) - self.drone.position[:2])
                # More precise completion: must be near entry AND outside building
                if (dist_home < 1.5 and self.drone.position[1] < 0.5) or self.drone.position[1] < -2.0:
                    self.drone.emergency_stop()  # stop spinning
                    self.drone.mission_complete = True
                    self.mission_active = False  # Stop updates immediately
                    print("=" * 50)
                    print("HOME! Mission complete - drone returned to start.")
                    print("=" * 50)
                    print("Press SPACE to start new mission, R to reset, ESC to exit")
                    # DON'T exit - wait for user input
        
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
        
        # Render drone(s) and LIDAR
        if self.multi_drone_mode and self.drone_manager:
            # Multi-drone mode: draw each drone with its color
            for i in range(self.drone_manager.count):
                drone = self.drone_manager.drones[i]
                color = self.drone_manager.get_color(i)
                
                # Get LIDAR data for this drone
                lidar = self.environment.get_lidar_scan(drone.position, drone.orientation)
                
                # Draw LIDAR in drone's color
                if lidar:
                    self.graphics.draw_lidar_scan(drone, lidar, color=color)
                
                # Draw drone marker
                center = self.graphics._world_to_screen(drone.position[:2])
                drone_radius = int(0.4 * self.graphics.scale)
                pygame.draw.circle(self.graphics.screen, color, center, drone_radius)
                pygame.draw.circle(self.graphics.screen, (0, 0, 0), center, drone_radius, 2)
                
                # Draw orientation arrow
                arrow_len = drone_radius + 8
                arrow_end = (
                    int(center[0] + arrow_len * math.cos(drone.orientation)),
                    int(center[1] + arrow_len * math.sin(drone.orientation))
                )
                pygame.draw.line(self.graphics.screen, (0, 0, 0), center, arrow_end, 3)
                
                # Draw drone ID
                font = pygame.font.SysFont('Arial', 12, bold=True)
                label = font.render(f"D{i}", True, (255, 255, 255))
                self.graphics.screen.blit(label, (center[0] - 8, center[1] - 6))
                
                # Draw IED detector circle (3m range) in drone's color
                ied_range_px = int(3.0 * self.graphics.scale)
                pygame.draw.circle(self.graphics.screen, color, center, ied_range_px, 1)
        else:
            # Single drone mode
            self.graphics.draw_drone(self.drone)
            
            # Render LIDAR rays (red) and IED detector rays (blue)
            if hasattr(self, 'lidar_data'):
                self.graphics.draw_lidar_scan(self.drone, self.lidar_data)
                
                # Draw IED detector rays - 3 meters range
                try:
                    center = self.graphics._world_to_screen(self.drone.position[:2])
                    ied_range_meters = 3.0
                    radius_px = int(ied_range_meters * self.graphics.scale)
                    num_rays = 10
                    
                    for j in range(num_rays):
                        angle = self.drone.orientation + (j * 2 * math.pi / num_rays)
                        end_x = center[0] + radius_px * math.cos(angle)
                        end_y = center[1] + radius_px * math.sin(angle)
                        end = (int(end_x), int(end_y))
                        pygame.draw.line(self.graphics.screen, (0, 150, 255), center, end, 2)
                        pygame.draw.circle(self.graphics.screen, (0, 100, 255), end, 3)
                except Exception:
                    pass
        
        # Render SLAM map only if enabled (hidden by default to avoid visual clutter)
        if self._show_map:
            self.graphics.draw_slam_map(self.slam.get_map())
        
        # Draw breadcrumbs as numbered path for debugging
        font = pygame.font.SysFont('Arial', 10)
        if self.multi_drone_mode and self.drone_manager:
            # Multi-drone mode: draw breadcrumbs for each drone in its color
            for drone_id in range(self.drone_manager.count):
                color = self.drone_manager.get_color(drone_id)
                breadcrumbs = self.drone_manager.breadcrumbs[drone_id] if drone_id < len(self.drone_manager.breadcrumbs) else []
                # Draw every 10th point to reduce clutter
                for idx, point in enumerate(breadcrumbs):
                    if idx % 10 != 0:
                        continue
                    pos = self.graphics._world_to_screen(point)
                    pygame.draw.circle(self.graphics.screen, color, pos, 3)
                    # Draw number in drone's color
                    text = font.render(str(idx), True, color)
                    self.graphics.screen.blit(text, (pos[0]+4, pos[1]-4))
        elif len(self._breadcrumbs) > 0:
            # Single drone mode
            for i, point in enumerate(self._breadcrumbs):
                if i % 10 != 0:
                    continue
                pos = self.graphics._world_to_screen(point)
                pygame.draw.circle(self.graphics.screen, (0, 80, 200), pos, 4)
                text = font.render(str(i), True, (0, 0, 180))
                self.graphics.screen.blit(text, (pos[0]+4, pos[1]-4))
        
        # Render detected objects (throttle to reduce draw load)
        detected_objects = self.vision.get_recent_detections()
        if detected_objects:
            self.graphics.draw_detections(detected_objects[:50])
        
        # Render minimap showing discovered features
        minimap_data = self.minimap.get_minimap_data()
        # Add breadcrumbs to minimap data so they can be displayed
        if minimap_data:
            minimap_data["breadcrumbs"] = self._breadcrumbs
            
            # Add search grid info for coverage visualization
            if self.multi_drone_mode and self.drone_manager:
                # Multi-drone: collect searched cells from all drones with colors
                all_searched = []
                all_frontiers = []
                searched_by_drone = {}  # drone_id -> list of cells
                
                frontiers_by_drone = {}  # drone_id -> list of frontier cells
                
                for i in range(self.drone_manager.count):
                    search = self.drone_manager.get_search(i)
                    color = self.drone_manager.get_color(i)
                    if hasattr(search, 'get_grid_data'):
                        grid_data = search.get_grid_data()
                        cells = grid_data.get('searched_cells', [])
                        frontiers = grid_data.get('frontier_cells', [])
                        # Add cells with drone color info
                        for cell in cells:
                            all_searched.append((cell, color))
                        all_frontiers.extend(frontiers)
                        searched_by_drone[i] = cells
                        frontiers_by_drone[i] = frontiers
                
                minimap_data["searched_cells_colored"] = all_searched
                minimap_data["searched_cells"] = [c[0] for c in all_searched]  # Fallback
                minimap_data["frontier_cells"] = all_frontiers
                minimap_data["frontiers_by_drone"] = frontiers_by_drone  # Per-drone frontiers
                minimap_data["grid_size"] = 3.0
                minimap_data["drone_colors"] = {i: self.drone_manager.get_color(i) 
                                                 for i in range(self.drone_manager.count)}
                # Add all drone positions for minimap display
                minimap_data["drone_positions"] = [
                    (self.drone_manager.drones[i].position[:2], self.drone_manager.get_color(i))
                    for i in range(self.drone_manager.count)
                ]
            elif hasattr(self.search_algorithm, 'get_grid_data'):
                # Single drone mode
                grid_data = self.search_algorithm.get_grid_data()
                minimap_data["searched_cells"] = grid_data.get('searched_cells', [])
                minimap_data["frontier_cells"] = grid_data.get('frontier_cells', [])
                minimap_data["grid_size"] = grid_data.get('grid_size', 3.0)
        
        self.graphics.draw_minimap(minimap_data, self.drone.position[:2])
        
        # Draw per-drone individual maps (multi-drone mode only)
        # After communication, maps show merged data (local + shared knowledge)
        if self.multi_drone_mode and self.drone_manager:
            drone_maps_data = []
            # Build drone_colors dict for all drones (for color-coded searched cells)
            drone_colors = {j: self.drone_manager.get_color(j) for j in range(self.drone_manager.count)}
            
            for i in range(self.drone_manager.count):
                search = self.drone_manager.get_search(i)
                drone = self.drone_manager.drones[i]
                gossip = self.drone_manager.gossip_maps[i]
                
                # Merge local + shared knowledge for display
                # This shows what each drone knows (including from communication)
                merged_free = search.free_cells.copy()
                merged_searched = search.searched_cells.copy()
                merged_walls = search.wall_cells.copy()
                
                # Add shared knowledge from gossip
                merged_free.update(gossip.get_global_free())
                merged_searched.update(gossip.get_global_searched())
                merged_walls.update(gossip.get_global_walls())
                
                # Get features (IEDs, objects) from gossip
                features = gossip.features if hasattr(gossip, 'features') else []
                
                # Get searched_by_drone from gossip for color-coded display
                # This ensures ALL drone maps show cells in the color of who searched them
                searched_by_drone = gossip.get_searched_by_drone()
                # Also include this drone's local searched cells
                if i not in searched_by_drone:
                    searched_by_drone[i] = set()
                searched_by_drone[i].update(search.searched_cells)
                
                drone_maps_data.append({
                    'drone_id': i,
                    'free_cells': merged_free,
                    'searched_cells': merged_searched,
                    'wall_cells': merged_walls,
                    'searched_by_drone': searched_by_drone,  # For color-coded display
                    'drone_colors': drone_colors,  # Colors for all drones
                    'color': self.drone_manager.get_color(i),
                    'position': drone.position[:2],
                    'features': features,
                })
            world_bounds = minimap_data.get("world_bounds", (-5, -5, 55, 45))
            # Pass wall segments from environment for building outline
            wall_segments = minimap_data.get("wall_segments", [])
            self.graphics.draw_drone_maps(drone_maps_data, world_bounds, wall_segments)
        
        # Get debug info from search algorithm (needed for UI display)
        search_debug = {}
        try:
            if hasattr(self.search_algorithm, 'get_debug_info'):
                search_debug = self.search_algorithm.get_debug_info()
        except Exception:
            pass
        
        # Add objects found to debug info
        objects_found = []
        for feature in self.minimap.discovered_features:
            obj_type = feature.additional_data.get('detection_type', '') if feature.additional_data else ''
            if obj_type and obj_type not in objects_found:
                objects_found.append(obj_type)
        search_debug['objects_found'] = objects_found
        
        # Render UI (pass simulated mission time for timer)
        simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED if self._start_time else None
        self.graphics.draw_ui(
            self.drone,
            self.slam,
            self.comm,
            simulated_elapsed,
            self.current_search_name,
            self.search_options,
            search_debug,  # Pass search debug info for mode/coverage display
        )
        
        self.graphics.draw_search_debug(search_debug, self.drone.position[:2])
        
        # Draw multi-drone overlay if enabled
        if self.multi_drone_mode and self.drone_manager:
            self._render_multi_drone_overlay()
        
        self.graphics.present()
    
    def _setup_multi_drone(self, preserve_drone_0: bool = False):
        """Setup multi-drone mode based on drone_count.
        
        Args:
            preserve_drone_0: If True, keep existing drones' positions/states
        """
        if self.drone_count > 1:
            # Save ALL existing drones' states INCLUDING search algorithm data
            existing_states = []
            existing_search_data = []
            if preserve_drone_0 and self.drone_manager:
                for i in range(min(self.drone_manager.count, self.drone_count)):
                    state = {
                        'position': self.drone_manager.drones[i].position.copy(),
                        'orientation': self.drone_manager.drones[i].orientation,
                        'inside_building': self.drone_manager.inside_building[i],
                        'returning_home': self.drone_manager.returning_home[i],
                        'entry_idx': self.drone_manager.entry_indices[i],
                        'breadcrumbs': self.drone_manager.breadcrumbs[i].copy() if i < len(self.drone_manager.breadcrumbs) else [],
                    }
                    existing_states.append(state)
                    
                    # Save search algorithm data (free_cells, searched_cells, wall_cells)
                    search = self.drone_manager.get_search(i)
                    search_data = {
                        'free_cells': search.free_cells.copy(),
                        'searched_cells': search.searched_cells.copy(),
                        'wall_cells': search.wall_cells.copy(),
                        'target': search.target,
                        'target_cell': search.target_cell,
                        'entry_point': search.entry_point,
                        'start_time': search.start_time,
                    }
                    existing_search_data.append(search_data)
            elif preserve_drone_0 and hasattr(self, 'drone') and self.drone:
                # Single drone mode -> multi-drone: preserve drone 0
                existing_states.append({
                    'position': self.drone.position.copy(),
                    'orientation': self.drone.orientation,
                    'inside_building': self._inside_building,
                    'returning_home': self._returning_home,
                    'entry_idx': self._entry_index,
                    'breadcrumbs': self._breadcrumbs.copy(),
                })
                # Save single drone search data
                if hasattr(self, 'search_algorithm'):
                    search_data = {
                        'free_cells': self.search_algorithm.free_cells.copy(),
                        'searched_cells': self.search_algorithm.searched_cells.copy(),
                        'wall_cells': self.search_algorithm.wall_cells.copy(),
                        'target': self.search_algorithm.target,
                        'target_cell': self.search_algorithm.target_cell,
                        'entry_point': self.search_algorithm.entry_point,
                        'start_time': self.search_algorithm.start_time,
                    }
                    existing_search_data.append(search_data)
            
            self.multi_drone_mode = True
            self.drone_manager = DroneManager(
                count=self.drone_count,
                comm_range=self.comm_range
            )
            self.drone_manager.initialize(entry_point=(25.0, -3.0))
            
            # Restore existing drones' states
            for i, state in enumerate(existing_states):
                if i < len(self.drone_manager.drones):
                    self.drone_manager.drones[i].position = state['position']
                    self.drone_manager.drones[i].orientation = state['orientation']
                    self.drone_manager.inside_building[i] = state['inside_building']
                    self.drone_manager.returning_home[i] = state['returning_home']
                    self.drone_manager.entry_indices[i] = state['entry_idx']
                    self.drone_manager.breadcrumbs[i] = state['breadcrumbs']
                    if state['inside_building']:
                        self.drone_manager.entry_indices[i] = len(self.drone_manager.entry_sequences[i])
                        if not self.drone_manager.mission_start_times[i]:
                            self.drone_manager.mission_start_times[i] = time.time()
            
            # Restore search algorithm data for existing drones
            for i, search_data in enumerate(existing_search_data):
                if i < self.drone_manager.count:
                    search = self.drone_manager.get_search(i)
                    search.free_cells = search_data['free_cells']
                    search.searched_cells = search_data['searched_cells']
                    search.wall_cells = search_data['wall_cells']
                    search.target = search_data['target']
                    search.target_cell = search_data['target_cell']
                    search.entry_point = search_data['entry_point']
                    search.start_time = search_data['start_time']
            
            print(f"Multi-drone mode: {self.drone_count} drones, {self.comm_range:.0f}m comm range (preserved {len(existing_states)} existing)")
        else:
            self.multi_drone_mode = False
            self.drone_manager = None
            print("Single drone mode")
    
    def _update_multi_drone(self, dt: float):
        """Update all drones in multi-drone mode."""
        if not self.drone_manager:
            return
        
        # Periodic status logging (every 30 seconds)
        if not hasattr(self, '_last_status_log'):
            self._last_status_log = 0
        now = time.time()
        if now - self._last_status_log > 30:
            self._last_status_log = now
            stats = self.drone_manager.get_global_coverage_stats()
            print(f"\n=== Multi-Drone Status (Coverage: {stats['coverage_pct']}%) ===")
            for i in range(self.drone_manager.count):
                search = self.drone_manager.get_search(i)
                drone = self.drone_manager.drones[i]
                target_info = f"target={search.target_cell}" if search.target_cell else "no target"
                failed_count = len(search.failed_targets)
                print(f"  D{i}: pos=({drone.position[0]:.1f},{drone.position[1]:.1f}) {target_info} failed={failed_count}")
            print()
        
        # Store lidar_data for drone 0 (for render compatibility)
        self.lidar_data = self.environment.get_lidar_scan(
            self.drone_manager.drones[0].position,
            self.drone_manager.drones[0].orientation
        )
        
        # Update breadcrumbs for all drones
        self.drone_manager.update_breadcrumbs()
        
        # Update mesh networking (HELLO beacons, message routing)
        messages = self.drone_manager.update_mesh()
        
        # Process gossip messages (map sharing)
        self.drone_manager.process_gossip(messages)
        
        # ===== PHASE 1: Sync local data to gossip for ALL drones =====
        # Do this FIRST so broadcast has fresh data
        for i in range(self.drone_manager.count):
            if self.drone_manager.inside_building[i]:
                self.drone_manager.sync_local_to_gossip(i)
        
        # ===== PHASE 2: Broadcast/sync gossip between drones =====
        # Now all drones have synced their local data, broadcast to share
        self.drone_manager.update_positions()
        self.drone_manager.broadcast_map_updates()
        
        # ===== PHASE 3: Update and navigate each drone =====
        for i in range(self.drone_manager.count):
            drone = self.drone_manager.get_drone(i)
            search = self.drone_manager.get_search(i)
            gossip = self.drone_manager.get_gossip_map(i)
            
            # Update drone physics
            drone.update(dt)
            self.physics.update_drone(drone, dt)
            
            # Check entry sequence for this drone
            if not self.drone_manager.inside_building[i]:
                entry_seq = self.drone_manager.entry_sequences[i]
                entry_idx = self.drone_manager.entry_indices[i]
                
                # Still update LIDAR during entry so map starts building
                lidar_data = self.environment.get_lidar_scan(drone.position, drone.orientation)
                if lidar_data:
                    self.slam.update(drone.position[:2], lidar_data)
                    self.minimap.add_lidar_scan(drone.position[:2], lidar_data, drone.orientation)
                
                # Track entry stuck time
                if not hasattr(self, '_entry_start_times'):
                    self._entry_start_times = {}
                if i not in self._entry_start_times:
                    self._entry_start_times[i] = time.time()
                
                # If stuck at entry for >10 seconds, force transition to search mode
                entry_elapsed = time.time() - self._entry_start_times[i]
                if entry_elapsed > 10.0 and drone.position[1] > -2:
                    # Drone is close enough to building, force entry
                    self.drone_manager.inside_building[i] = True
                    self.drone_manager.entry_indices[i] = len(entry_seq)
                    self.drone_manager.mission_start_times[i] = time.time()
                    print(f"Drone {i} forced entry after {entry_elapsed:.0f}s")
                    del self._entry_start_times[i]
                    continue
                
                if entry_idx < len(entry_seq):
                    target = entry_seq[entry_idx]
                    dist = np.linalg.norm(np.array(target) - drone.position[:2])
                    
                    if dist < 1.5:  # Increased tolerance
                        self.drone_manager.entry_indices[i] += 1
                        if self.drone_manager.entry_indices[i] >= len(entry_seq):
                            self.drone_manager.inside_building[i] = True
                            self.drone_manager.mission_start_times[i] = time.time()
                            print(f"Drone {i} entered building")
                            if i in self._entry_start_times:
                                del self._entry_start_times[i]
                    else:
                        # Use pathfinding for entry (more reliable than direct navigation)
                        path = self.slam.path_planner.get_path_to_next_point(
                            (float(drone.position[0]), float(drone.position[1])),
                            target,
                            self.environment
                        )
                        if path and len(path) > 0:
                            next_step = path[min(len(path)-1, 1)]
                            current_pos = (float(drone.position[0]), float(drone.position[1]))
                            # WALL CHECK: verify BOTH position AND path are valid
                            if (self.environment.is_position_valid((next_step[0], next_step[1])) and
                                self.environment.is_path_clear(current_pos, (next_step[0], next_step[1]))):
                                waypoint_3d = (next_step[0], next_step[1], float(drone.position[2]))
                                drone.navigate_to(waypoint_3d)
                            # else: invalid position, don't move this frame
                        else:
                            # Fallback to direct navigation - MUST check wall AND path first
                            current_pos = (float(drone.position[0]), float(drone.position[1]))
                            if (self.environment.is_position_valid((target[0], target[1])) and
                                self.environment.is_path_clear(current_pos, (target[0], target[1]))):
                                waypoint_3d = (target[0], target[1], float(drone.position[2]))
                                drone.navigate_to(waypoint_3d)
                            # else: target blocked by wall, don't move
                continue  # Skip main search phase for drones still in entry
            
            # Main search phase - drone is inside building
            # Get LIDAR data for this drone
            lidar_data = self.environment.get_lidar_scan(drone.position, drone.orientation)
            
            # Update SLAM with sensor data (for pathfinding)
            self.slam.update(drone.position[:2], lidar_data)
            
            # Update minimap with LIDAR data (for visualization)
            if lidar_data:
                self.minimap.add_lidar_scan(drone.position[:2], lidar_data, drone.orientation)
            
            # Update search algorithm with global knowledge (gossip already synced in Phase 1-2)
            self.drone_manager.update_search_from_gossip(i)
            
            # Pass claimed targets to search (avoid cells other drones are targeting)
            claimed = self.drone_manager.get_claimed_by_others(i)
            search.set_claimed_by_others(claimed)
            
            # V11.5g: Pass SIMULATED time to search algorithm
            start_time = self.drone_manager.mission_start_times[i]
            if start_time:
                simulated_elapsed = (time.time() - start_time) * SIM_SPEED
                if hasattr(search, 'set_simulated_time'):
                    search.set_simulated_time(simulated_elapsed)
                
                # Return is handled by search algorithm, but check for completion
                search_complete = search.is_mission_complete()
                if search_complete and not self.drone_manager.returning_home[i]:
                    self.drone_manager.returning_home[i] = True
                    print(f"Drone {i} search complete, returning")
            
            # Navigation
            if self.drone_manager.returning_home[i]:
                target = self.slam.entry_point
                dist_home = np.linalg.norm(np.array(target) - drone.position[:2])
                if dist_home < 1.5 and drone.position[1] < 0.5:
                    if not drone.mission_complete:
                        drone.mission_complete = True
                        print(f"Drone {i} mission complete!")
            else:
                # Get next waypoint from search algorithm (handles return internally)
                target = search.get_next_waypoint(
                    drone.position[:2],
                    lidar_data,
                    drone.orientation
                )
                # Check if search returned entry point (meaning it's time to return)
                if target and search.entry_point:
                    if abs(target[0] - search.entry_point[0]) < 0.1 and abs(target[1] - search.entry_point[1]) < 0.1:
                        self.drone_manager.returning_home[i] = True
                
                # Claim the target cell so other drones don't compete
                if target and search.target_cell:
                    self.drone_manager.claim_target(i, search.target_cell)
                
                # Fallback: if no target, explore in a new direction
                if target is None:
                    # No unsearched cells - try moving toward unexplored areas
                    # Find direction with most open space (LIDAR is a LIST, not dict)
                    if lidar_data and len(lidar_data) > 0:
                        best_angle = drone.orientation
                        best_dist = 0
                        angle_step = 2 * math.pi / len(lidar_data)
                        for idx, dist in enumerate(lidar_data):
                            if dist > best_dist:
                                best_dist = dist
                                best_angle = drone.orientation + idx * angle_step
                        # Move toward most open direction - WITH WALL CHECK
                        step = min(best_dist * 0.5, 2.0)
                        test_pos = (
                            drone.position[0] + step * math.cos(best_angle),
                            drone.position[1] + step * math.sin(best_angle)
                        )
                        # Only set target if position is valid (not through wall)
                        if self.environment.is_position_valid(test_pos):
                            target = test_pos
            
            if target:
                # Use A* pathfinding (same as single-drone)
                current_pos = (float(drone.position[0]), float(drone.position[1]))
                path = self.slam.path_planner.get_path_to_next_point(
                    current_pos,
                    target,
                    self.environment
                )
                if path and len(path) > 1:
                    # Use pathfinding result - just check position is valid (like single-drone)
                    next_step = path[min(len(path)-1, 1)]
                    if self.environment.is_position_valid((next_step[0], next_step[1])):
                        waypoint_3d = (next_step[0], next_step[1], float(drone.position[2]))
                        drone.navigate_to(waypoint_3d)
                    else:
                        # A* gave invalid position - use direct navigation fallback
                        path = None  # Fall through to fallback
                
                # Fallback: A* failed or returned invalid - use SAME approach as single-drone
                if not path or len(path) <= 1:
                    dx = target[0] - drone.position[0]
                    dy = target[1] - drone.position[1]
                    dist = np.hypot(dx, dy)
                    
                    if dist > 0.3:
                        step_size = min(0.8, dist)
                        
                        # Try multiple directions (SAME as single-drone fallback)
                        angles_to_try = [
                            np.arctan2(dy, dx),  # Direct to target
                            np.arctan2(dy, dx) + np.pi/4,
                            np.arctan2(dy, dx) - np.pi/4,
                            np.arctan2(dy, dx) + np.pi/2,
                            np.arctan2(dy, dx) - np.pi/2,
                            drone.orientation,  # Current heading
                            drone.orientation + np.pi/4,
                            drone.orientation - np.pi/4,
                        ]
                        
                        for angle in angles_to_try:
                            nx = current_pos[0] + step_size * np.cos(angle)
                            ny = current_pos[1] + step_size * np.sin(angle)
                            # WALL CHECK: verify position is valid (like single-drone)
                            if self.environment.is_position_valid((nx, ny)):
                                waypoint_3d = (nx, ny, float(drone.position[2]))
                                drone.navigate_to(waypoint_3d)
                                break
            
            # IED sensor for this drone
            ied_reading = self.ied_sensor.read(
                (float(drone.position[0]), float(drone.position[1])),
                self.environment
            )
            if ied_reading and ied_reading.confidence > 0.2:
                print(f"Drone {i} IED ALERT: {ied_reading.to_text()}")
                gossip.add_local_feature("ied", drone.position[:2], ied_reading.confidence)
        
        # Gossip sync now happens in Phase 1-2 at START of this function
    
    def _render_multi_drone_overlay(self):
        """Render multi-drone specific overlays."""
        if not self.drone_manager:
            return
        
        # Draw mesh links between drones (green lines)
        for (d1, d2) in self.drone_manager.get_mesh_links():
            pos1 = self.graphics._world_to_screen(self.drone_manager.drones[d1].position[:2])
            pos2 = self.graphics._world_to_screen(self.drone_manager.drones[d2].position[:2])
            pygame.draw.line(self.graphics.screen, (0, 255, 0), pos1, pos2, 3)
        
        # Draw comm range circles (drones themselves are drawn in main _render)
        for i in range(self.drone_manager.count):
            drone = self.drone_manager.drones[i]
            color = self.drone_manager.get_color(i)
            center = self.graphics._world_to_screen(drone.position[:2])
            
            # Draw comm range circle (dashed style using segments)
            radius = int(self.comm_range * self.graphics.scale)
            pygame.draw.circle(self.graphics.screen, color, center, radius, 1)
        
        # Draw multi-drone status panel
        self._draw_multi_drone_panel()
    
    def _draw_multi_drone_panel(self):
        """Draw status panel for multi-drone mode."""
        if not self.drone_manager:
            return
        
        # Panel position (bottom right area) - inside grey Drone Status column
        panel_x = self.window_size[0] - 250  # Moved 40px right (was -290)
        panel_y = self.window_size[1] - 200
        panel_w = 240  # 40px narrower (was 280)
        panel_h = 190
        
        # Draw panel background
        panel_rect = pygame.Rect(panel_x, panel_y, panel_w, panel_h)
        pygame.draw.rect(self.graphics.screen, (240, 240, 255), panel_rect)
        pygame.draw.rect(self.graphics.screen, (0, 0, 0), panel_rect, 2)
        
        font = pygame.font.SysFont('Arial', 14)
        y = panel_y + 10
        
        # Title
        title = font.render(f"Multi-Drone: {self.drone_count} drones", True, (0, 0, 0))
        self.graphics.screen.blit(title, (panel_x + 10, y))
        y += 20
        
        # Comm range
        range_text = font.render(f"Comm Range: {self.comm_range:.0f}m (+/- to adjust)", True, (0, 0, 0))
        self.graphics.screen.blit(range_text, (panel_x + 10, y))
        y += 20
        
        # Mesh links
        links = self.drone_manager.get_mesh_links()
        mesh_text = font.render(f"Mesh Links: {len(links)}", True, (0, 200, 0) if links else (200, 0, 0))
        self.graphics.screen.blit(mesh_text, (panel_x + 10, y))
        y += 20
        
        # Per-drone status
        stats = self.drone_manager.get_global_coverage_stats()
        cov_text = font.render(f"Global Coverage: {stats['coverage_pct']}%", True, (0, 0, 0))
        self.graphics.screen.blit(cov_text, (panel_x + 10, y))
        y += 25
        
        # Individual drone status
        for i in range(self.drone_manager.count):
            drone = self.drone_manager.drones[i]
            color = self.drone_manager.get_color(i)
            inside = self.drone_manager.inside_building[i]
            returning = self.drone_manager.returning_home[i]
            complete = drone.mission_complete
            
            status = "COMPLETE" if complete else "RETURN" if returning else "SEARCH" if inside else "ENTRY"
            status_text = font.render(f"D{i}: {status}", True, color)
            self.graphics.screen.blit(status_text, (panel_x + 10, y))
            y += 18
    
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
        self._time_limit_reached = False  # Reset time limit flag
        self._mid_screenshot_taken = False  # Reset mid-mission screenshot flag
        
        # Reset multi-drone manager if active
        if self.multi_drone_mode and self.drone_manager:
            self.drone_manager.reset(entry_point=(25.0, -3.0))
    
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
        self._time_limit_reached = False  # Reset time limit flag
        self._mid_screenshot_taken = False  # Reset mid-mission screenshot flag
    
    def _cleanup(self):
        """Cleanup resources."""
        pygame.quit()
        print("Simulation ended.")

    def _save_screenshot(self, tag: str = ""):
        """Save a PNG with method, time, coverage stats, and tag for tracking."""
        # Get version from search algorithm docstring if available
        version = "v2"  # Default to v2 for systematic_mapper
        try:
            doc = self.search_algorithm.__class__.__module__
            import importlib
            mod = importlib.import_module(doc)
            if mod.__doc__ and "VERSION:" in mod.__doc__.upper():
                for line in mod.__doc__.split('\n'):
                    if "VERSION:" in line.upper():
                        version = "v" + line.split(":")[-1].strip().split()[0]
                        break
        except Exception:
            pass

        # Get coverage stats from search algorithm if available
        coverage_str = ""
        try:
            if hasattr(self.search_algorithm, 'get_coverage_stats'):
                stats = self.search_algorithm.get_coverage_stats()
                coverage_pct = int(stats.get('coverage_pct', 0))
                coverage_str = f"_cov{coverage_pct}pct"
        except Exception:
            pass

        # Build filename: METHOD_VERSION_TIME_COVERAGE_TAG_UNIQUE.png
        elapsed = int(time.time() - self._start_time) if self._inside_building else 0
        mm = elapsed // 60
        ss = elapsed % 60
        method = self.current_search_name.replace("search_", "")
        tag_str = f"_{tag}" if tag else ""
        # Add unique timestamp to prevent overwriting
        unique_id = int(time.time() * 1000) % 100000  # Last 5 digits of milliseconds
        # Shortened prefix: r_s_m_ instead of result_systematic_mapper_
        filename = f"r_s_m_{version}_{mm:02d}m{ss:02d}s{coverage_str}{tag_str}_{unique_id}.png"
        try:
            pygame.image.save(self.graphics.screen, filename)
            print(f"Saved: {filename}")
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
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
