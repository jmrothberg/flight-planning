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
2. IED detector has 2 METER range - MUST get within 2m to detect!
3. Must cover ALL accessible areas systematically
4. Must NOT get stuck or loop endlessly
5. Must complete search within 7 MINUTES (420 seconds)
6. Return to entry point when done or time expires
=============================================================================

Search Algorithm:
- Configured in algorithm_config.py (default: search_systematic_mapper)
- Bonus-based frontier exploration + grid coverage
- 2m grid matches IED detector range

Controls:
- SPACE: Start/stop mission
- R: Reset simulation
- N: New object placement (same building)
- B: New building layout (5 realistic connected room configurations)
- D: Cycle drone count (1-12) for multi-drone mode
- +/=: Increase communication range
- -: Decrease communication range
- P: Save screenshot
- ESC: Exit
"""

import pygame
import sys
import numpy as np
from typing import Tuple, List, Optional, Dict
import time
import math
import cv2

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
SEARCH_CONFIG = {"coverage_radius": 2.0}

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
        # Door center x scales with building (design door at x=25 in 50m building)
        self._door_x = 25.0 * self.environment.building_scale
        # Start OUTSIDE the building near front door gap
        self.drone = Drone(position=(self._door_x, -3, 3))  # outside, facing upward (north)
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
        self._laser_target = (self._door_x, 0.0)  # Door position (on the wall gap)
        self._entry_sequence = [
            (self._door_x, -0.5),  # Just outside the door
            (self._door_x, 0.5),   # Just inside the door
            (self._door_x, 2.0),   # Further inside to start exploration
        ]
        self._entry_index = 0
        self._inside_building = False
        self._start_time = time.time()  # Track mission time
        
        # Return-to-entry and inertial breadcrumbs
        self._returning_home = False
        self._breadcrumbs: List[Tuple[float, float]] = []
        self._last_breadcrumb_ts = time.time()
        # Hide SLAM occupancy map by default to remove gray/black spray
        self._show_map = False
        
        # Periodic rotation for limited-FoV LiDAR (STM 54×42, 59° HFoV)
        # Drone rotates in-place periodically to build 360° awareness
        self._rotation_scan_interval = 1.5   # seconds between rotation scans (fast for small rooms)
        self._last_rotation_scan = 0.0       # timestamp of last rotation scan
        self._rotation_scan_active = False   # currently performing rotation scan
        self._rotation_scan_target = 0.0     # target orientation for scan
        self._rotation_scan_steps = 0        # number of rotation steps remaining

        # Coverage milestone logging (avoid spam — only log each milestone once)
        self._coverage_milestones_logged = set()

        # MULTI-DRONE SUPPORT
        self.drone_count = 1  # Number of drones (1-12)
        self.comm_range = 10.0  # Communication range in meters
        self.drone_manager: Optional[DroneManager] = None  # Created when drone_count > 1
        self.multi_drone_mode = False  # Flag for multi-drone operation

        # Initialize simulation engines
        self.graphics = GraphicsEngine(window_size)
        # Scale graphics to fill same screen area regardless of building size
        self.graphics.scale = 10.0 / self.environment.building_scale
        self.physics = PhysicsEngine()
        # Pass environment to physics for wall collision detection
        self.physics.environment = self.environment
        
        # Mission parameters
        self.mission_active = False
        self.exploration_complete = False

        # Video recording state (must be before AUTO_START which calls _start_video_recording)
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._video_frame_count: int = 0
        self._video_filename: str = ""

        # AUTO-RUN: Auto-start mission if enabled
        if AUTO_START:
            self.mission_active = True
            self._start_video_recording()
            print("AUTO-START: Mission starting automatically...")

        # AUTO-SCREENSHOT: Track for periodic screenshots
        self._auto_screenshot_last = time.time()
        self._auto_start_time = time.time()

        # Per-drone screenshots pending (saved after render)
        self._pending_drone_screenshots: List[int] = []
        # Per-drone frozen elapsed time at completion (for timer freeze)
        self._drone_completion_elapsed: Dict[int, float] = {}

        # Navigation stuck detection — catches when A* finds a path but drone
        # physically can't follow it (wall collision at doorways).
        # Keyed by drone_id (0 for single-drone mode).
        self._nav_stuck_pos: Dict[int, Tuple[float, float]] = {}
        self._nav_stuck_time: Dict[int, float] = {}

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
        # Try common constructor signatures (pass building bounds when possible)
        bw = self.environment.width
        bh = self.environment.height
        try:
            return cls(coverage_radius=2.0, building_width=bw, building_height=bh)
        except Exception:
            pass
        try:
            return cls(detection_radius=2.0)
        except Exception:
            pass
        try:
            return cls(coverage_radius=2.0)
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
            
            # Check per-drone timers: 6 min (360s) = forced return, 7 min (420s) = battery dead
            if self.mission_active and self._start_time is not None:
                if self.multi_drone_mode and self.drone_manager:
                    now = time.time()
                    all_dead = True
                    for i in range(self.drone_manager.count):
                        drone_i = self.drone_manager.drones[i]
                        if drone_i.mission_complete:
                            continue  # Already home or dead
                        st = self.drone_manager.mission_start_times[i]
                        if not st:
                            all_dead = False
                            continue  # Not yet entered building
                        elapsed_i = (now - st) * SIM_SPEED

                        # 6 MINUTES (360s): force return home
                        if elapsed_i >= 360.0 and not self.drone_manager.returning_home[i]:
                            self.drone_manager.returning_home[i] = True
                            search_i = self.drone_manager.get_search(i)
                            cov = search_i.get_coverage_stats().get('coverage_pct', 0)
                            print(f"[6 MIN] Drone {i}: forcing return ({elapsed_i:.0f}s sim, {cov}% coverage)")

                        # 7 MINUTES (420s): battery dead — mission over for this drone
                        if elapsed_i >= 420.0:
                            if not drone_i.mission_complete:
                                drone_i.mission_complete = True
                                drone_i.emergency_stop()
                                search_i = self.drone_manager.get_search(i)
                                cov = search_i.get_coverage_stats().get('coverage_pct', 0)
                                self._drone_completion_elapsed[i] = elapsed_i
                                print(f"[BATTERY DEAD] Drone {i}: 7 min reached ({elapsed_i:.0f}s sim, {cov}% coverage)")
                                self._pending_drone_screenshots.append(i)
                        else:
                            all_dead = False

                    # End mission when ALL drones are done (home or dead)
                    if all_dead and not getattr(self, '_time_limit_reached', False):
                        self._time_limit_reached = True
                        print("=" * 50)
                        cov = self.drone_manager.get_global_coverage_stats()['coverage_pct']
                        print(f"ALL DRONES DONE. Global coverage: {cov}%")
                        print("=" * 50)
                        self._save_screenshot("final_7min")
                        self._finalize_video()
                        self.mission_active = False
                        print("Press SPACE to start new mission, R to reset, ESC to exit")
                else:
                    # Single drone mode: original behavior
                    simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED
                    if simulated_elapsed >= 420.0 and not getattr(self, '_time_limit_reached', False):
                        self._time_limit_reached = True
                        print("=" * 50)
                        print(f"TIME LIMIT! 7 minutes reached ({simulated_elapsed:.0f}s simulated)")
                        cov = self.search_algorithm.get_coverage_stats().get('coverage_pct', 0)
                        print(f"Final coverage: {cov}%")
                        print("=" * 50)
                        self._save_screenshot("final_7min")
                        self._finalize_video()
                        self.mission_active = False
                        self.drone.emergency_stop()
                        print("Press SPACE to start new mission, R to reset, ESC to exit")

            # AUTO-SCREENSHOT: Take mid-mission screenshot at 3.5 minutes simulated
            if not hasattr(self, '_mid_screenshot_taken'):
                self._mid_screenshot_taken = False
            if self.mission_active and self._start_time is not None and not self._mid_screenshot_taken:
                simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED
                if simulated_elapsed >= 210.0:  # Half of 7 minutes
                    self._save_screenshot("mid_3_5min")
                    self._mid_screenshot_taken = True
            
            if self.mission_active:
                # Update core systems (may set mission_active=False if drone arrives home)
                self._update_systems(dt)

            # Check mission completion AFTER update (mission_active may have changed)
            if self.mission_active:
                all_complete = False
                if self.multi_drone_mode and self.drone_manager:
                    all_complete = all(d.mission_complete for d in self.drone_manager.drones)
                else:
                    all_complete = self.drone.mission_complete

                if all_complete:
                    self.mission_active = False
                    print("=" * 50)
                    print("MISSION COMPLETE! All drones returned HOME.")
                    print("=" * 50)
                    self._save_screenshot("final_complete")
                    self._finalize_video()  # Save video recording
                    print("Press SPACE to start new mission, R to reset, ESC to exit")

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
                        self._start_video_recording()
                        print("Starting new mission with existing map...")
                    else:
                        self.mission_active = not self.mission_active
                        if self.mission_active:
                            self._start_video_recording()
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
                    # Cycle drone count (1 -> 2 -> ... -> 12 -> 1)
                    # DON'T reset - just add/remove drones while preserving current state
                    old_count = self.drone_count
                    self.drone_count = (self.drone_count % 12) + 1
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
        
        # Periodic rotation scan for limited-FoV LiDAR (59° HFoV)
        # Every few seconds when inside building, rotate in-place to accumulate
        # a 360° picture from multiple 59° sweeps.
        if self._inside_building:
            simulated_now = (time.time() - self._start_time) * SIM_SPEED if self._start_time else 0
            if simulated_now - self._last_rotation_scan >= self._rotation_scan_interval:
                self._last_rotation_scan = simulated_now
                # Do a quick rotation: take scans at multiple orientations
                base_ori = self.drone.orientation
                hfov = self.environment.lidar_hfov  # ~59°
                num_extra_scans = max(1, int(2 * math.pi / hfov) - 1)  # ~5 extra scans
                for scan_i in range(1, num_extra_scans + 1):
                    scan_angle = base_ori + scan_i * hfov
                    extra_scan = self.environment.get_lidar_scan(self.drone.position, scan_angle)
                    self.slam.update(self.drone.position[:2], extra_scan)
                    self.minimap.add_lidar_scan(self.drone.position[:2], extra_scan, scan_angle)
                    # CRITICAL: Also feed rotation scans to search algorithm so it
                    # discovers doorways/rooms outside the forward 59° cone
                    if hasattr(self.search_algorithm, 'feed_lidar_scan'):
                        self.search_algorithm.feed_lidar_scan(
                            float(self.drone.position[0]), float(self.drone.position[1]),
                            extra_scan, scan_angle
                        )

        # Coverage milestone logging (25%, 50%, 75%, 90%)
        if self._inside_building and hasattr(self.search_algorithm, 'get_coverage_stats'):
            cov_pct = self.search_algorithm.get_coverage_stats().get('coverage_pct', 0)
            for milestone in [25, 50, 75, 90]:
                if cov_pct >= milestone and milestone not in self._coverage_milestones_logged:
                    self._coverage_milestones_logged.add(milestone)
                    sim_elapsed = (time.time() - self._start_time) * SIM_SPEED if self._start_time else 0
                    free_n = len([c for c in self.search_algorithm.free_cells if c[1] >= 0])
                    print(f"[COVERAGE] {milestone}% at {sim_elapsed:.0f}s sim | {free_n} free cells discovered")

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

            # Track time at current entry waypoint for stuck timeout
            if not hasattr(self, '_entry_wp_time'):
                self._entry_wp_time = time.time()

            entry_wp_elapsed = time.time() - self._entry_wp_time
            # Progressive tolerance: starts 0.8m, widens to 2.0m after 3s
            tolerance = 0.8 if entry_wp_elapsed < 3.0 else 2.0

            if dist_to_target < tolerance:
                self._entry_index += 1
                self._entry_wp_time = time.time()
                if self._entry_index >= len(self._entry_sequence):
                    # Now inside - start selected search algorithm
                    self._inside_building = True
                    self._start_time = time.time()
                    # IMMEDIATE 360° rotation scan on entry so search algorithm
                    # has a full view of the entry area before first target selection.
                    # Without this, the 59° FoV means most nearby cells are undiscovered.
                    self._do_full_rotation_scan()
                    try:
                        mod = getattr(self.search_algorithm.__class__, '__module__', '')
                        active_name = mod.split('.')[-1] if mod else self.current_search_name
                    except Exception:
                        active_name = self.current_search_name
                    print(f"Entered building, beginning {active_name} search...")
            elif entry_wp_elapsed > 5.0 and self.drone.position[1] > -4.0:
                # Force advance after 5s stuck at this waypoint
                print(f"Entry waypoint stuck for {entry_wp_elapsed:.0f}s at y={self.drone.position[1]:.1f}, advancing...")
                self._entry_index += 1
                self._entry_wp_time = time.time()
                if self._entry_index >= len(self._entry_sequence):
                    self._inside_building = True
                    self._start_time = time.time()
                    self._do_full_rotation_scan()
                    print("Forced entry - beginning search...")
            else:
                # DIRECT navigation for entry - no pathfinding needed outside
                waypoint_3d = (target[0], target[1], float(self.drone.position[2]))
                self.drone.navigate_to(waypoint_3d)
                next_target_2d = "handled"  # Mark as handled to skip pathfinding below
        
        # Exploration using selected search algorithm
        if next_target_2d is None and not self._returning_home:
            # Pass simulated time to search algorithm (critical for return timing)
            simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED
            if hasattr(self.search_algorithm, 'set_simulated_time'):
                self.search_algorithm.set_simulated_time(simulated_elapsed)
            
            # Use selected search algorithm for exploration
            next_target_2d = self.search_algorithm.get_next_waypoint(
                self.drone.position[:2],
                self.lidar_data if hasattr(self, 'lidar_data') else [],
                self.drone.orientation
            )

            # If search algorithm found no targets, do a 360° rotation scan to
            # discover doorways/rooms, then retry. Without this, the 59° FoV
            # means the drone may be next to a doorway it can't see.
            if next_target_2d is None:
                self._do_full_rotation_scan()
                # Retry target selection with the new data
                next_target_2d = self.search_algorithm.get_next_waypoint(
                    self.drone.position[:2],
                    self.lidar_data if hasattr(self, 'lidar_data') else [],
                    self.drone.orientation
                )
            # STILL no target after rotation scan — move toward most open LiDAR direction
            if next_target_2d is None and self.lidar_data:
                if isinstance(self.lidar_data, dict):
                    ranges = self.lidar_data["ranges"]
                    _sa = self.lidar_data["start_angle"]
                    _as = self.lidar_data["angle_step"]
                else:
                    ranges = self.lidar_data
                    _sa = self.drone.orientation
                    _as = 2 * math.pi / len(ranges) if ranges else 0
                best_angle, best_dist = self.drone.orientation, 0
                for idx, d in enumerate(ranges):
                    if d > best_dist:
                        best_dist = d
                        best_angle = _sa + idx * _as
                step = min(best_dist * 0.5, 2.0)
                if step > 0.3:
                    tp = (self.drone.position[0] + step * math.cos(best_angle),
                          self.drone.position[1] + step * math.sin(best_angle))
                    if self.environment.is_position_valid(tp, radius=0.2):
                        next_target_2d = tp

            # Check if search algorithm reports completion or is in RETURN mode
            search_complete = False
            if hasattr(self.search_algorithm, 'is_mission_complete'):
                search_complete = self.search_algorithm.is_mission_complete()

            # Detect when search algorithm has switched to RETURN mode
            # (handles return timing internally, may trigger before is_mission_complete)
            search_returning = getattr(self.search_algorithm, '_mode', '') == "RETURN"

            if (search_complete or search_returning) and not self._returning_home:
                self._returning_home = True
                if not getattr(self, '_return_msg_printed', False):
                    cov = self.search_algorithm.get_coverage_stats().get('coverage_pct', 0)
                    print(f"[HOME] Returning — coverage: {cov}%")
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
                self._finalize_video()  # Save video recording
                print("Press SPACE to start new mission, R to reset, ESC to exit")
                # DON'T exit - wait for user input
                return
            next_target_2d = self.slam.entry_point
        
        if next_target_2d and next_target_2d != "handled":
            # Navigation stuck escape — if drone hasn't moved for 0.5s, break free
            escaped = (self._inside_building and
                       self._nav_escape_if_stuck(self.drone, 0, self.search_algorithm))

            # Use A* pathfinding to avoid obstacles
            current_2d = (float(self.drone.position[0]), float(self.drone.position[1]))
            path = self.slam.path_planner.get_path_to_next_point(current_2d, next_target_2d, self.environment)

            if escaped:
                pass  # Escape already handled navigation this frame
            elif path and len(path) > 0:
                # Adaptive stride: try 3, 2, 1 — pick furthest that has a clear
                # straight-line path (no wall corners cut).  Stride=3 helps commit
                # through doorways; fall back to 1 at corners.
                chosen = None
                for s in ([3, 2, 1] if self._inside_building else [2, 1]):
                    idx = min(len(path) - 1, s)
                    step = path[idx]
                    if (self.environment.is_position_valid((step[0], step[1]), radius=0.2) and
                            self.environment.is_path_clear(current_2d, (step[0], step[1]))):
                        chosen = step
                        break
                if chosen is None:
                    # Even stride=1 fails path-clear — use it anyway (collision will handle)
                    chosen = path[min(len(path) - 1, 1)]
                if self.environment.is_position_valid((chosen[0], chosen[1]), radius=0.2):
                    waypoint_3d = (chosen[0], chosen[1], float(self.drone.position[2]))
                    self.drone.navigate_to(waypoint_3d)
            else:
                # A* failed — tell search algorithm immediately so it retargets
                # instead of waiting for slow stuck detection (~4.5s wasted)
                if (hasattr(self, 'search_algorithm') and
                        hasattr(self.search_algorithm, 'mark_target_unreachable') and
                        not self._returning_home):
                    self.search_algorithm.mark_target_unreachable()

                # Try direct navigation toward target with multi-angle fallback
                dx = next_target_2d[0] - current_2d[0]
                dy = next_target_2d[1] - current_2d[1]
                dist = np.hypot(dx, dy)
                moved = False

                if dist > 0.3:
                    step_size = min(0.8, dist)
                    angles_to_try = [
                        np.arctan2(dy, dx),
                        np.arctan2(dy, dx) + np.pi/4,
                        np.arctan2(dy, dx) - np.pi/4,
                        np.arctan2(dy, dx) + np.pi/2,
                        np.arctan2(dy, dx) - np.pi/2,
                        self.drone.orientation,
                        self.drone.orientation + np.pi/4,
                        self.drone.orientation - np.pi/4,
                    ]
                    for angle in angles_to_try:
                        nx = current_2d[0] + step_size * np.cos(angle)
                        ny = current_2d[1] + step_size * np.sin(angle)
                        if self.environment.is_position_valid((nx, ny), radius=0.2):
                            self.drone.navigate_to((nx, ny, float(self.drone.position[2])))
                            moved = True
                            break

                # Emergency escape: 24 angles with shrinking steps
                if not moved:
                    for angle_deg in range(0, 360, 15):
                        angle = math.radians(angle_deg)
                        for step in [0.5, 0.3, 0.15]:
                            nx = current_2d[0] + step * math.cos(angle)
                            ny = current_2d[1] + step * math.sin(angle)
                            if self.environment.is_position_valid((nx, ny), radius=0.15):
                                self.drone.navigate_to((nx, ny, float(self.drone.position[2])))
                                moved = True
                                break
                        if moved:
                            break
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
                    self._finalize_video()  # Save video recording
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
    
    def _nav_escape_if_stuck(self, drone, drone_id: int, search_algo=None) -> bool:
        """Detect if drone is stuck at a wall/door edge and force an escape move.

        A* may find a valid path through a doorway, but wall-collision pushback
        physically prevents the drone from following it.  The search algorithm's
        stuck detection only triggers after ~1.5 s and only blacklists the
        target — it doesn't move the drone.  This method detects that the drone
        hasn't moved 0.5 m in 1.5 real seconds and forces a move away.

        Has a 3-second cooldown between escapes to prevent oscillation (escape →
        retarget same area → stuck → escape → repeat).

        Returns True if an escape was performed (caller should skip normal nav).
        """
        pos = (float(drone.position[0]), float(drone.position[1]))
        now = time.time()

        # Initialize cooldown tracker
        if not hasattr(self, '_nav_escape_cooldown'):
            self._nav_escape_cooldown: Dict[int, float] = {}

        # Cooldown: don't escape again for 3 real seconds after last escape
        last_escape = self._nav_escape_cooldown.get(drone_id, 0)
        if now - last_escape < 3.0:
            # During cooldown, just reset tracking so we don't build up stale data
            self._nav_stuck_pos[drone_id] = pos
            self._nav_stuck_time[drone_id] = now
            return False

        ref_pos = self._nav_stuck_pos.get(drone_id)
        ref_time = self._nav_stuck_time.get(drone_id, now)

        if ref_pos is None:
            self._nav_stuck_pos[drone_id] = pos
            self._nav_stuck_time[drone_id] = now
            return False

        dist = math.hypot(pos[0] - ref_pos[0], pos[1] - ref_pos[1])
        if dist > 0.5:
            # Drone moved — reset
            self._nav_stuck_pos[drone_id] = pos
            self._nav_stuck_time[drone_id] = now
            return False

        elapsed = now - ref_time
        if elapsed < 1.5:
            return False  # Not stuck long enough yet

        # --- STUCK: haven't moved 0.5 m in 1.5 real seconds ---
        # Blacklist current target so search picks something else
        if search_algo and hasattr(search_algo, 'mark_target_unreachable'):
            search_algo.mark_target_unreachable()

        # Force escape: try 16 directions, prefer AWAY from facing (back into room)
        escape_base = drone.orientation + math.pi  # opposite of facing
        moved = False
        for offset in [0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.6, -1.6, 2.0, -2.0, math.pi]:
            angle = escape_base + offset
            for step in [1.0, 0.5, 0.3]:
                nx = pos[0] + step * math.cos(angle)
                ny = pos[1] + step * math.sin(angle)
                if (self.environment.is_position_valid((nx, ny), radius=0.2) and
                        self.environment.is_path_clear(pos, (nx, ny))):
                    drone.navigate_to((nx, ny, float(drone.position[2])))
                    moved = True
                    break
            if moved:
                break

        # Reset tracker and set cooldown
        self._nav_stuck_pos[drone_id] = pos
        self._nav_stuck_time[drone_id] = now
        self._nav_escape_cooldown[drone_id] = now
        return moved

    def _do_full_rotation_scan(self):
        """Do an immediate 360° rotation scan to build full map awareness.

        Called on building entry and whenever the search algorithm has no targets.
        With only 59° HFoV, a single forward scan misses most of the room.
        """
        base_ori = self.drone.orientation
        hfov = self.environment.lidar_hfov  # ~59° = ~1.03 rad
        num_scans = max(1, int(2 * math.pi / hfov))  # ~6 scans for 360°
        for si in range(num_scans):
            scan_angle = base_ori + si * hfov
            scan_data = self.environment.get_lidar_scan(self.drone.position, scan_angle)
            if scan_data:
                self.slam.update(self.drone.position[:2], scan_data)
                self.minimap.add_lidar_scan(self.drone.position[:2], scan_data, scan_angle)
                if hasattr(self.search_algorithm, 'feed_lidar_scan'):
                    self.search_algorithm.feed_lidar_scan(
                        float(self.drone.position[0]), float(self.drone.position[1]),
                        scan_data, scan_angle
                    )
        # Reset rotation scan timer so periodic scan starts fresh
        self._last_rotation_scan = (time.time() - self._start_time) * SIM_SPEED if self._start_time else 0

    def _do_multi_drone_rotation_scan(self, drone_idx: int, drone, search):
        """360° rotation scan for a multi-drone entry."""
        base_ori = drone.orientation
        hfov = self.environment.lidar_hfov
        num_scans = max(1, int(2 * math.pi / hfov))
        for si in range(num_scans):
            scan_angle = base_ori + si * hfov
            scan_data = self.environment.get_lidar_scan(drone.position, scan_angle)
            if scan_data:
                self.slam.update(drone.position[:2], scan_data)
                self.minimap.add_lidar_scan(drone.position[:2], scan_data, scan_angle)
                if hasattr(search, 'feed_lidar_scan'):
                    search.feed_lidar_scan(
                        float(drone.position[0]), float(drone.position[1]),
                        scan_data, scan_angle
                    )

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
        
        # Render drone(s) and LIDAR — unified style for single and multi-drone
        # All drones: colored circle body, LIDAR rays in drone color, thin IED circle
        if self.multi_drone_mode and self.drone_manager:
            drone_list = [(i, self.drone_manager.drones[i], self.drone_manager.get_color(i))
                          for i in range(self.drone_manager.count)]
        else:
            # Single drone uses same style — red (drone 0 color)
            drone_list = [(0, self.drone, (255, 80, 80))]

        font_drone = pygame.font.SysFont('Arial', 12, bold=True)
        for i, drone, color in drone_list:
            # LIDAR rays in drone's color
            lidar = self.environment.get_lidar_scan(drone.position, drone.orientation)
            if lidar:
                self.graphics.draw_lidar_scan(drone, lidar, color=color)

            # Drone marker (colored circle with black border)
            center = self.graphics._world_to_screen(drone.position[:2])
            drone_radius = int(0.4 * self.graphics.scale)
            pygame.draw.circle(self.graphics.screen, color, center, drone_radius)
            pygame.draw.circle(self.graphics.screen, (0, 0, 0), center, drone_radius, 2)

            # Orientation arrow
            arrow_len = drone_radius + 8
            arrow_end = (
                int(center[0] + arrow_len * math.cos(drone.orientation)),
                int(center[1] + arrow_len * math.sin(drone.orientation))
            )
            pygame.draw.line(self.graphics.screen, (0, 0, 0), center, arrow_end, 3)

            # Drone ID label
            label = font_drone.render(f"D{i}", True, (255, 255, 255))
            self.graphics.screen.blit(label, (center[0] - 8, center[1] - 6))

            # IED detector circle (2m range) — thin ring, no spokes
            ied_range_px = int(2.0 * self.graphics.scale)
            pygame.draw.circle(self.graphics.screen, color, center, ied_range_px, 1)
        
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
                searched_by_drone = {}  # drone_id -> list of cells
                frontiers_by_drone = {}  # drone_id -> list of frontier cells

                # Collect raw cell sets from ALL drones for correct frontier calc
                combined_free = set()
                combined_searched = set()
                combined_walls = set()

                for i in range(self.drone_manager.count):
                    search = self.drone_manager.get_search(i)
                    color = self.drone_manager.get_color(i)
                    combined_free.update(search.free_cells)
                    combined_searched.update(search.searched_cells)
                    combined_walls.update(search.wall_cells)
                    if hasattr(search, 'get_grid_data'):
                        grid_data = search.get_grid_data()
                        cells = grid_data.get('searched_cells', [])
                        # Add cells with drone color info
                        for cell in cells:
                            all_searched.append((cell, color))
                        searched_by_drone[i] = cells

                # Compute frontiers same way as individual drone maps:
                # frontier = free - searched - walls (interior only)
                # This ensures combined map shows the SAME markers as individual maps
                grid_size = 2.0
                all_frontiers = []
                for cell in combined_free:
                    if cell[1] < 0:
                        continue  # Skip exterior
                    if cell in combined_searched or cell in combined_walls:
                        continue
                    wx = cell[0] * grid_size + grid_size / 2
                    wy = cell[1] * grid_size + grid_size / 2
                    all_frontiers.append((wx, wy))

                # Assign frontiers to nearest drone for per-drone coloring
                for i in range(self.drone_manager.count):
                    search = self.drone_manager.get_search(i)
                    drone_frontier = []
                    for cell in search.free_cells:
                        if cell[1] < 0:
                            continue
                        if cell in combined_searched or cell in combined_walls:
                            continue
                        wx = cell[0] * grid_size + grid_size / 2
                        wy = cell[1] * grid_size + grid_size / 2
                        drone_frontier.append((wx, wy))
                    frontiers_by_drone[i] = drone_frontier

                minimap_data["searched_cells_colored"] = all_searched
                minimap_data["searched_cells"] = [c[0] for c in all_searched]  # Fallback
                minimap_data["frontier_cells"] = all_frontiers
                minimap_data["frontiers_by_drone"] = frontiers_by_drone  # Per-drone frontiers
                minimap_data["grid_size"] = 2.0
                minimap_data["drone_colors"] = {i: self.drone_manager.get_color(i) 
                                                 for i in range(self.drone_manager.count)}
                # Add all drone positions for minimap display
                minimap_data["drone_positions"] = [
                    (self.drone_manager.drones[i].position[:2], self.drone_manager.get_color(i))
                    for i in range(self.drone_manager.count)
                ]
            elif hasattr(self.search_algorithm, 'get_grid_data'):
                # Single drone mode — use same frontier def as drone maps
                grid_data = self.search_algorithm.get_grid_data()
                minimap_data["searched_cells"] = grid_data.get('searched_cells', [])
                minimap_data["grid_size"] = grid_data.get('grid_size', 2.0)
                # Frontier = free - searched - walls (consistent with individual maps)
                s = self.search_algorithm
                gs = s.grid_size
                frontier_world = []
                for cell in s.free_cells:
                    if cell[1] < 0:
                        continue
                    if cell in s.searched_cells or cell in s.wall_cells:
                        continue
                    frontier_world.append((cell[0] * gs + gs / 2, cell[1] * gs + gs / 2))
                minimap_data["frontier_cells"] = frontier_world
        
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
                    'coverage_pct': search.get_coverage_stats()['coverage_pct'],
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
        
        # Build multi-drone data for UI
        multi_drone_data = None
        if self.multi_drone_mode and self.drone_manager:
            drones_info = []
            now = time.time()
            for i in range(self.drone_manager.count):
                drone_i = self.drone_manager.drones[i]
                search_i = self.drone_manager.get_search(i)
                gossip_i = self.drone_manager.gossip_maps[i]
                start_t = self.drone_manager.mission_start_times[i]

                # Use frozen time if drone completed, otherwise live time
                if drone_i.mission_complete and i in self._drone_completion_elapsed:
                    elapsed_i = self._drone_completion_elapsed[i]
                else:
                    elapsed_i = (now - start_t) * SIM_SPEED if start_t else 0

                # Sync check: use gossip last_sync OR check if in range of any other drone
                sync_times = list(gossip_i.last_sync.values())
                gossip_synced = bool(sync_times and (now - max(sync_times)) < 2.0)
                # Also check direct distance (gossip_links already computed above)
                in_range_of_any = any(
                    d1 == i or d2 == i
                    for d1, d2, _ in self.drone_manager.get_gossip_links()
                )
                sync_ok = gossip_synced or in_range_of_any

                # Count merged cells (global minus local)
                merged = max(0, len(gossip_i.get_global_searched()) - len(search_i.searched_cells))

                inside = self.drone_manager.inside_building[i]
                returning = self.drone_manager.returning_home[i]
                complete = drone_i.mission_complete
                status_str = "DONE" if complete else "RTN" if returning else "SRCH" if inside else "ENTRY"

                drones_info.append({
                    'id': i,
                    'color': self.drone_manager.get_color(i),
                    'battery': drone_i.battery_level,
                    'elapsed': elapsed_i,
                    'status': status_str,
                    'coverage_pct': search_i.get_coverage_stats()['coverage_pct'],
                    'sync_ok': sync_ok,
                    'merged_cells': merged,
                })

            gossip_links = self.drone_manager.get_gossip_links()
            stats = self.drone_manager.get_global_coverage_stats()
            multi_drone_data = {
                'drones': drones_info,
                'mesh_links': len(gossip_links),
                'global_coverage': stats['coverage_pct'],
                'comm_range': self.comm_range,
            }

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
            multi_drone_data,
        )
        
        self.graphics.draw_search_debug(search_debug, self.drone.position[:2])
        
        # Draw multi-drone overlay if enabled
        if self.multi_drone_mode and self.drone_manager:
            self._render_multi_drone_overlay()

        self.graphics.present()

        # Record video frame (after present so screen is fully drawn)
        if self._video_writer is not None:
            self._record_frame()

        # Save pending per-drone screenshots (after render so screen is current)
        for drone_id in self._pending_drone_screenshots:
            self._save_drone_screenshot(drone_id)
        self._pending_drone_screenshots.clear()
    
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
                        'mission_start_time': self.drone_manager.mission_start_times[i],
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
                        'exit_point': search.exit_point,
                        'start_time': search.start_time,
                    }
                    existing_search_data.append(search_data)
            elif preserve_drone_0 and hasattr(self, 'drone') and self.drone:
                # Single drone mode -> multi-drone: preserve drone 0
                # Use self._start_time as mission start (set when drone entered building)
                existing_states.append({
                    'position': self.drone.position.copy(),
                    'orientation': self.drone.orientation,
                    'inside_building': self._inside_building,
                    'returning_home': self._returning_home,
                    'entry_idx': self._entry_index,
                    'breadcrumbs': self._breadcrumbs.copy(),
                    'mission_start_time': self._start_time if self._inside_building else None,
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
                        'exit_point': self.search_algorithm.exit_point,
                        'start_time': self.search_algorithm.start_time,
                    }
                    existing_search_data.append(search_data)
            
            self.multi_drone_mode = True
            self.drone_manager = DroneManager(
                count=self.drone_count,
                comm_range=self.comm_range,
                building_width=self.environment.width,
                building_height=self.environment.height
            )
            self.drone_manager.initialize(entry_point=(self._door_x, -3.0))
            
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
                        # Preserve original mission start time (don't reset timer)
                        saved_start = state.get('mission_start_time')
                        self.drone_manager.mission_start_times[i] = saved_start if saved_start else time.time()
            
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
                    search.exit_point = search_data.get('exit_point', None)
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

        # Start ALL drone clocks on first call — battery drains from launch, not entry
        for i in range(self.drone_manager.count):
            if self.drone_manager.mission_start_times[i] is None:
                self.drone_manager.mission_start_times[i] = time.time()

        # Periodic status logging (every 30 seconds)
        if not hasattr(self, '_last_status_log'):
            self._last_status_log = 0
        now = time.time()
        if now - self._last_status_log > 30:
            self._last_status_log = now
            stats = self.drone_manager.get_global_coverage_stats()
            gossip_links = self.drone_manager.get_gossip_links()
            # Show distances between all pairs for debugging
            pair_dists = []
            for i in range(self.drone_manager.count):
                for j in range(i + 1, self.drone_manager.count):
                    pi = self.drone_manager.positions.get(i)
                    pj = self.drone_manager.positions.get(j)
                    if pi and pj:
                        d = math.hypot(pj[0] - pi[0], pj[1] - pi[1])
                        pair_dists.append(f"D{i}-D{j}:{d:.0f}m")
            dist_str = " | ".join(pair_dists) if pair_dists else ""
            print(f"\n=== Multi-Drone Status (Coverage: {stats['coverage_pct']}%, Links: {len(gossip_links)}, {dist_str}) ===")
            for i in range(self.drone_manager.count):
                search = self.drone_manager.get_search(i)
                drone = self.drone_manager.drones[i]
                inside = self.drone_manager.inside_building[i]
                returning = self.drone_manager.returning_home[i]
                complete = drone.mission_complete
                status = "COMPLETE" if complete else "RETURN" if returning else "SEARCH" if inside else "ENTRY"
                cov = search.get_coverage_stats()['coverage_pct']
                failed_count = len(search.failed_targets)
                print(f"  D{i}: {status} {cov}% pos=({drone.position[0]:.1f},{drone.position[1]:.1f}) failed={failed_count}")
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
        # Sync ALL drones every frame (empty sets are a no-op)
        for i in range(self.drone_manager.count):
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

            # CRITICAL: Skip completed/dead drones entirely — no physics, no updates.
            # A dead drone (battery=0 or returned home) must not move or update maps.
            if drone.mission_complete:
                continue

            # Update drone physics (only for alive drones)
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
                    self._do_multi_drone_rotation_scan(i, drone, search)
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
                            # Immediate 360° rotation scan on entry
                            self._do_multi_drone_rotation_scan(i, drone, search)
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
                            if (self.environment.is_position_valid((next_step[0], next_step[1]), radius=0.2) and
                                self.environment.is_path_clear(current_pos, (next_step[0], next_step[1]))):
                                waypoint_3d = (next_step[0], next_step[1], float(drone.position[2]))
                                drone.navigate_to(waypoint_3d)
                            # else: invalid position, don't move this frame
                        else:
                            # Fallback to direct navigation - MUST check wall AND path first
                            current_pos = (float(drone.position[0]), float(drone.position[1]))
                            if (self.environment.is_position_valid((target[0], target[1]), radius=0.2) and
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

            # Periodic rotation scan for limited-FoV LiDAR (59° HFoV)
            start_time_i = self.drone_manager.mission_start_times[i]
            if start_time_i:
                sim_now = (time.time() - start_time_i) * SIM_SPEED
                if not hasattr(self, '_multi_last_rotation'):
                    self._multi_last_rotation = {}
                last_rot = self._multi_last_rotation.get(i, 0.0)
                if sim_now - last_rot >= self._rotation_scan_interval:
                    self._multi_last_rotation[i] = sim_now
                    base_ori = drone.orientation
                    hfov = self.environment.lidar_hfov
                    num_extra = max(1, int(2 * math.pi / hfov) - 1)
                    for si in range(1, num_extra + 1):
                        scan_angle = base_ori + si * hfov
                        extra = self.environment.get_lidar_scan(drone.position, scan_angle)
                        self.slam.update(drone.position[:2], extra)
                        self.minimap.add_lidar_scan(drone.position[:2], extra, scan_angle)
                        # CRITICAL: Feed rotation scans to search algorithm
                        if hasattr(search, 'feed_lidar_scan'):
                            search.feed_lidar_scan(
                                float(drone.position[0]), float(drone.position[1]),
                                extra, scan_angle
                            )

            # Update search algorithm with global knowledge (gossip already synced in Phase 1-2)
            self.drone_manager.update_search_from_gossip(i)
            
            # Pass claimed targets to search (avoid cells other drones are targeting)
            claimed = self.drone_manager.get_claimed_by_others(i)
            search.set_claimed_by_others(claimed)
            
            # Pass simulated time to search algorithm
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
                home = self.slam.entry_point
                dist_home = np.linalg.norm(np.array(home) - drone.position[:2])
                if dist_home < 1.5 and drone.position[1] < 0.5:
                    if not drone.mission_complete:
                        drone.mission_complete = True
                        cov = search.get_coverage_stats()['coverage_pct']
                        # Freeze timer at completion time
                        start_t = self.drone_manager.mission_start_times[i]
                        if start_t:
                            self._drone_completion_elapsed[i] = (time.time() - start_t) * SIM_SPEED
                        print(f"Drone {i} mission complete! Coverage: {cov}%")
                        self._pending_drone_screenshots.append(i)

                # Try A* to home first; if it fails, use intermediate hops
                target = home
                current_pos = (float(drone.position[0]), float(drone.position[1]))
                path = self.slam.path_planner.get_path_to_next_point(
                    current_pos, home, self.environment)
                if not path or len(path) <= 1:
                    # A* can't reach home — try closer intermediate targets
                    dx = home[0] - current_pos[0]
                    dy = home[1] - current_pos[1]
                    angle_home = math.atan2(dy, dx)
                    for hop in [8.0, 5.0, 3.0]:
                        if hop >= dist_home:
                            continue
                        intermediate = (
                            current_pos[0] + hop * math.cos(angle_home),
                            current_pos[1] + hop * math.sin(angle_home),
                        )
                        hop_path = self.slam.path_planner.get_path_to_next_point(
                            current_pos, intermediate, self.environment)
                        if hop_path and len(hop_path) > 1:
                            target = intermediate
                            break
            else:
                # Get next waypoint from search algorithm (handles return internally)
                target = search.get_next_waypoint(
                    drone.position[:2],
                    lidar_data,
                    drone.orientation
                )

                # Rotation scan + retry if no target
                if target is None:
                    self._do_multi_drone_rotation_scan(i, drone, search)
                    target = search.get_next_waypoint(
                        drone.position[:2], lidar_data, drone.orientation)
                # STILL no target after rotation — move toward most open LiDAR direction
                if target is None and lidar_data:
                    if isinstance(lidar_data, dict):
                        ranges = lidar_data["ranges"]
                        _sa = lidar_data["start_angle"]
                        _as = lidar_data["angle_step"]
                    else:
                        ranges = lidar_data
                        _sa = drone.orientation
                        _as = 2 * math.pi / len(ranges) if ranges else 0
                    best_angle, best_dist = drone.orientation, 0
                    for idx, d in enumerate(ranges):
                        if d > best_dist:
                            best_dist = d
                            best_angle = _sa + idx * _as
                    step = min(best_dist * 0.5, 2.0)
                    if step > 0.3:
                        tp = (drone.position[0] + step * math.cos(best_angle),
                              drone.position[1] + step * math.sin(best_angle))
                        if self.environment.is_position_valid(tp, radius=0.2):
                            target = tp

                # Also detect RETURN mode in multi-drone
                if getattr(search, '_mode', '') == "RETURN" and not self.drone_manager.returning_home[i]:
                    self.drone_manager.returning_home[i] = True
                    cov = search.get_coverage_stats().get('coverage_pct', 0)
                    st = self.drone_manager.mission_start_times[i]
                    sim_t = int((time.time() - st) * SIM_SPEED) if st else 0
                    print(f"[HOME] D{i}: RETURN at {sim_t}s sim, {cov}% coverage, exit={search.exit_point}")

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
                    # Find direction with most open space in LiDAR data
                    if lidar_data:
                        if isinstance(lidar_data, dict):
                            ranges = lidar_data["ranges"]
                            _start_angle = lidar_data["start_angle"]
                            _angle_step = lidar_data["angle_step"]
                        else:
                            ranges = lidar_data
                            _start_angle = drone.orientation
                            _angle_step = 2 * math.pi / len(ranges) if len(ranges) > 0 else 0
                        best_angle = drone.orientation
                        best_dist = 0
                        for idx, dist in enumerate(ranges):
                            if dist > best_dist:
                                best_dist = dist
                                best_angle = _start_angle + idx * _angle_step
                        # Move toward most open direction - WITH WALL CHECK
                        step = min(best_dist * 0.5, 2.0)
                        test_pos = (
                            drone.position[0] + step * math.cos(best_angle),
                            drone.position[1] + step * math.sin(best_angle)
                        )
                        # Only set target if position is valid (not through wall)
                        if self.environment.is_position_valid(test_pos, radius=0.2):
                            target = test_pos
            
            if target:
                # Navigation stuck escape — break free from door edges
                escaped = (self.drone_manager.inside_building[i] and
                           self._nav_escape_if_stuck(drone, i, search))

                # Use A* pathfinding (same as single-drone)
                current_pos = (float(drone.position[0]), float(drone.position[1]))
                path = self.slam.path_planner.get_path_to_next_point(
                    current_pos,
                    target,
                    self.environment
                )
                if escaped:
                    pass  # Escape already handled navigation
                elif path and len(path) > 1:
                    # Adaptive stride: try 3, 2, 1 — furthest with clear path
                    chosen = None
                    for s in [3, 2, 1]:
                        idx = min(len(path) - 1, s)
                        step = path[idx]
                        if (self.environment.is_position_valid((step[0], step[1]), radius=0.2) and
                                self.environment.is_path_clear(current_pos, (step[0], step[1]))):
                            chosen = step
                            break
                    if chosen is None:
                        chosen = path[min(len(path) - 1, 1)]
                    if self.environment.is_position_valid((chosen[0], chosen[1]), radius=0.2):
                        waypoint_3d = (chosen[0], chosen[1], float(drone.position[2]))
                        drone.navigate_to(waypoint_3d)
                    else:
                        path = None  # Fall through to fallback
                
                # Fallback: A* failed or returned invalid (skip if escape handled it)
                if not escaped and (not path or len(path) <= 1):
                    # Tell search algorithm immediately so it retargets
                    if (hasattr(search, 'mark_target_unreachable') and
                            not self.drone_manager.returning_home[i]):
                        search.mark_target_unreachable()

                    moved = False
                    dx = target[0] - drone.position[0]
                    dy = target[1] - drone.position[1]
                    dist = np.hypot(dx, dy)

                    if dist > 0.3:
                        step_size = min(0.8, dist)
                        angles_to_try = [
                            np.arctan2(dy, dx),
                            np.arctan2(dy, dx) + np.pi/4,
                            np.arctan2(dy, dx) - np.pi/4,
                            np.arctan2(dy, dx) + np.pi/2,
                            np.arctan2(dy, dx) - np.pi/2,
                            drone.orientation,
                            drone.orientation + np.pi/4,
                            drone.orientation - np.pi/4,
                        ]
                        for angle in angles_to_try:
                            nx = current_pos[0] + step_size * np.cos(angle)
                            ny = current_pos[1] + step_size * np.sin(angle)
                            if self.environment.is_position_valid((nx, ny), radius=0.2):
                                drone.navigate_to((nx, ny, float(drone.position[2])))
                                moved = True
                                break

                    # EMERGENCY ESCAPE: try 24 angles with shrinking steps
                    if not moved:
                        for angle_deg in range(0, 360, 15):
                            angle = math.radians(angle_deg)
                            for step in [0.5, 0.3, 0.15]:
                                nx = current_pos[0] + step * math.cos(angle)
                                ny = current_pos[1] + step * math.sin(angle)
                                if self.environment.is_position_valid((nx, ny), radius=0.15):
                                    drone.navigate_to((nx, ny, float(drone.position[2])))
                                    moved = True
                                    break
                            if moved:
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

        # Draw sync visualization (dashed lines in transmitting drone's color)
        font_dist = pygame.font.SysFont('Arial', 11)
        now = time.time()
        for sync_time, sender_id, receiver_id in self.drone_manager.get_recent_syncs():
            age = now - sync_time
            if age > 2.0:
                continue
            # Fade color with age (newer = brighter)
            alpha = max(0.3, 1.0 - age / 2.0)
            sender_color = self.drone_manager.get_color(sender_id)
            line_color = (
                int(sender_color[0] * alpha),
                int(sender_color[1] * alpha),
                int(sender_color[2] * alpha),
            )
            pos1 = self.graphics._world_to_screen(
                self.drone_manager.drones[sender_id].position[:2])
            pos2 = self.graphics._world_to_screen(
                self.drone_manager.drones[receiver_id].position[:2])
            self._draw_dashed_line(pos1, pos2, line_color, width=2)

        # Draw comm range circles (drones themselves are drawn in main _render)
        for i in range(self.drone_manager.count):
            drone = self.drone_manager.drones[i]
            color = self.drone_manager.get_color(i)
            center = self.graphics._world_to_screen(drone.position[:2])

            # Draw comm range circle (thin ring)
            radius = int(self.comm_range * self.graphics.scale)
            pygame.draw.circle(self.graphics.screen, color, center, radius, 1)

        # If drones are NOT linked, show distance between pairs so user understands why
        gossip_links = self.drone_manager.get_gossip_links()
        if not gossip_links and self.drone_manager.count >= 2:
            for i in range(self.drone_manager.count):
                for j in range(i + 1, self.drone_manager.count):
                    pos_i = self.drone_manager.positions.get(i)
                    pos_j = self.drone_manager.positions.get(j)
                    if pos_i and pos_j:
                        dist = math.hypot(pos_j[0] - pos_i[0], pos_j[1] - pos_i[1])
                        sp1 = self.graphics._world_to_screen(
                            self.drone_manager.drones[i].position[:2])
                        sp2 = self.graphics._world_to_screen(
                            self.drone_manager.drones[j].position[:2])
                        mid = ((sp1[0] + sp2[0]) // 2, (sp1[1] + sp2[1]) // 2 - 10)
                        clr = (200, 0, 0) if dist > self.comm_range else (0, 200, 0)
                        dist_surf = font_dist.render(f"{dist:.0f}m (need <{self.comm_range:.0f}m)", True, clr)
                        self.graphics.screen.blit(dist_surf, mid)

        # Draw multi-drone status panel
        self._draw_multi_drone_panel()

    def _draw_dashed_line(self, pos1, pos2, color, dash_len=8, gap_len=5, width=2):
        """Draw a dashed line between two screen positions."""
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        dist = math.hypot(dx, dy)
        if dist < 1:
            return
        nx, ny = dx / dist, dy / dist
        pos = 0
        while pos < dist:
            seg_end = min(pos + dash_len, dist)
            p1 = (int(pos1[0] + nx * pos), int(pos1[1] + ny * pos))
            p2 = (int(pos1[0] + nx * seg_end), int(pos1[1] + ny * seg_end))
            pygame.draw.line(self.graphics.screen, color, p1, p2, width)
            pos += dash_len + gap_len
    
    def _draw_multi_drone_panel(self):
        """Draw status panel for multi-drone mode with coverage and sync info."""
        if not self.drone_manager:
            return

        # Panel position (bottom right area) - inside grey Drone Status column
        panel_x = self.window_size[0] - 250
        panel_y = self.window_size[1] - 220
        panel_w = 240
        panel_h = 210

        # Draw panel background
        panel_rect = pygame.Rect(panel_x, panel_y, panel_w, panel_h)
        pygame.draw.rect(self.graphics.screen, (240, 240, 255), panel_rect)
        pygame.draw.rect(self.graphics.screen, (0, 0, 0), panel_rect, 2)

        font = pygame.font.SysFont('Arial', 14)
        small_font = pygame.font.SysFont('Arial', 12)
        y = panel_y + 8

        # Title
        title = font.render(f"Multi-Drone: {self.drone_count} drones", True, (0, 0, 0))
        self.graphics.screen.blit(title, (panel_x + 10, y))
        y += 18

        # Comm range
        range_text = small_font.render(f"Comm: {self.comm_range:.0f}m (+/- adjust)", True, (0, 0, 0))
        self.graphics.screen.blit(range_text, (panel_x + 10, y))
        y += 16

        # Gossip links (actual distance-based connectivity)
        gossip_links = self.drone_manager.get_gossip_links()
        link_text = f"Links: {len(gossip_links)}"
        if gossip_links:
            link_text += f" ({gossip_links[0][2]:.0f}m)" if len(gossip_links) == 1 else ""
        mesh_text = font.render(link_text, True, (0, 200, 0) if gossip_links else (200, 0, 0))
        self.graphics.screen.blit(mesh_text, (panel_x + 10, y))
        y += 18

        # Global coverage
        stats = self.drone_manager.get_global_coverage_stats()
        cov_text = font.render(f"Global Coverage: {stats['coverage_pct']}%", True, (0, 0, 0))
        self.graphics.screen.blit(cov_text, (panel_x + 10, y))
        y += 22

        # Per-drone status: D0: SEARCH 45% sync:OK
        now = time.time()
        for i in range(self.drone_manager.count):
            drone = self.drone_manager.drones[i]
            color = self.drone_manager.get_color(i)
            inside = self.drone_manager.inside_building[i]
            returning = self.drone_manager.returning_home[i]
            complete = drone.mission_complete

            status = "COMPLETE" if complete else "RETURN" if returning else "SEARCH" if inside else "ENTRY"

            # Per-drone coverage
            search = self.drone_manager.get_search(i)
            cov = search.get_coverage_stats()['coverage_pct']

            # Sync indicator: OK if gossiped within last 2s
            gossip = self.drone_manager.gossip_maps[i]
            last_sync_times = gossip.last_sync.values()
            if last_sync_times and (now - max(last_sync_times)) < 2.0:
                sync_str = "OK"
                sync_color = (0, 180, 0)
            elif not last_sync_times:
                sync_str = "--"
                sync_color = (150, 150, 150)
            else:
                sync_str = "NO"
                sync_color = (200, 0, 0)

            line = f"D{i}: {status} {cov}%"
            text_surf = small_font.render(line, True, color)
            self.graphics.screen.blit(text_surf, (panel_x + 10, y))
            # Sync indicator on the right
            sync_surf = small_font.render(f"sync:{sync_str}", True, sync_color)
            self.graphics.screen.blit(sync_surf, (panel_x + 160, y))
            y += 16
    
    def _start_video_recording(self):
        """Start recording the pygame window to an MP4 video."""
        if self._video_writer is not None:
            self._finalize_video()
        self._video_frame_count = 0
        self._video_filename = "_recording_in_progress.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        w, h = self.window_size
        self._video_writer = cv2.VideoWriter(self._video_filename, fourcc, 20, (w, h))
        if self._video_writer.isOpened():
            print(f"Video recording started ({w}x{h} @ 20fps)")
        else:
            print("WARNING: Failed to open video writer")
            self._video_writer = None

    def _record_frame(self):
        """Capture the current pygame frame to the video writer (every 3rd frame)."""
        if self._video_writer is None:
            return
        self._video_frame_count += 1
        if self._video_frame_count % 3 != 0:
            return
        surface = self.graphics.screen
        frame = pygame.surfarray.array3d(surface)  # (W, H, 3) RGB
        frame = np.transpose(frame, (1, 0, 2))     # (H, W, 3)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self._video_writer.write(frame)

    def _finalize_video(self):
        """Finalize the video recording: release writer and rename to final filename."""
        if self._video_writer is None:
            return
        self._video_writer.release()
        self._video_writer = None

        # Build final filename matching screenshot convention
        cov = 0
        try:
            if self.multi_drone_mode and self.drone_manager:
                cov = self.drone_manager.get_global_coverage_stats()['coverage_pct']
            elif hasattr(self.search_algorithm, 'get_coverage_stats'):
                cov = int(self.search_algorithm.get_coverage_stats().get('coverage_pct', 0))
        except Exception:
            pass

        # Elapsed time: from earliest drone entry to now
        elapsed = 0
        if self.multi_drone_mode and self.drone_manager:
            earliest = None
            for st in self.drone_manager.mission_start_times:
                if st and (earliest is None or st < earliest):
                    earliest = st
            if earliest:
                elapsed = int((time.time() - earliest) * SIM_SPEED)
        elif self._start_time:
            elapsed = int((time.time() - self._start_time) * SIM_SPEED)

        mm = elapsed // 60
        ss = elapsed % 60
        n = self.drone_count
        final_name = f"sim_{n}d_{cov}pct_{mm}m{ss:02d}s.mp4"

        try:
            if os.path.exists(self._video_filename):
                if os.path.exists(final_name):
                    os.remove(final_name)
                os.rename(self._video_filename, final_name)
                print(f"Video saved: {final_name}")
            else:
                print("WARNING: Temp video file not found")
        except Exception as e:
            print(f"Error renaming video: {e}")

        self._video_frame_count = 0
        self._video_filename = ""

    def _reset_simulation(self):
        """Reset the simulation to initial state."""
        self._finalize_video()  # Save any in-progress recording
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
        self._entry_wp_time = time.time()
        self._return_msg_printed = False
        self._time_limit_reached = False
        self._mid_screenshot_taken = False
        self._drone_completion_elapsed.clear()
        self._pending_drone_screenshots.clear()

        # Reset multi-drone manager if active
        if self.multi_drone_mode and self.drone_manager:
            self.drone_manager.reset(entry_point=(self._door_x, -3.0))

    def _reset_mission_keep_map(self):
        """Reset for new mission but keep existing map."""
        print("Starting new mission with existing map...")
        # Reset drone to start position
        self.drone.reset()
        self.drone.position = np.array([self._door_x, -3, 3], dtype=np.float32)  # Back outside
        
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
        self._time_limit_reached = False
        self._mid_screenshot_taken = False
    
    def _cleanup(self):
        """Cleanup resources."""
        self._finalize_video()  # Save any in-progress recording
        pygame.quit()
        print("Simulation ended.")

    def _save_screenshot(self, tag: str = ""):
        """Save a PNG with concise, informative filename."""
        # Get coverage
        cov = 0
        try:
            if self.multi_drone_mode and self.drone_manager:
                cov = self.drone_manager.get_global_coverage_stats()['coverage_pct']
            elif hasattr(self.search_algorithm, 'get_coverage_stats'):
                cov = int(self.search_algorithm.get_coverage_stats().get('coverage_pct', 0))
        except Exception:
            pass

        elapsed = int((time.time() - self._start_time) * SIM_SPEED) if self._start_time else 0
        mm = elapsed // 60
        ss = elapsed % 60
        n = self.drone_count
        tag_str = f"_{tag}" if tag else ""
        filename = f"sim_{n}d_{mm}m{ss:02d}s_{cov}pct{tag_str}.png"
        try:
            pygame.image.save(self.graphics.screen, filename)
            print(f"Saved: {filename}")
        except Exception as e:
            print(f"Error saving screenshot: {e}")

    def _save_drone_screenshot(self, drone_id: int):
        """Save screenshot when a specific drone returns home."""
        if not self.drone_manager:
            return
        search = self.drone_manager.get_search(drone_id)
        cov = search.get_coverage_stats()['coverage_pct']
        start_t = self.drone_manager.mission_start_times[drone_id]
        elapsed = int((time.time() - start_t) * SIM_SPEED) if start_t else 0
        mm = elapsed // 60
        ss = elapsed % 60
        filename = f"d{drone_id}_done_{mm}m{ss:02d}s_{cov}pct.png"
        try:
            pygame.image.save(self.graphics.screen, filename)
            print(f"Saved: {filename}")
        except Exception as e:
            print(f"Error saving drone screenshot: {e}")

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
