#!/usr/bin/env python3
"""
Simulation GUI — Ground Station + Simulated World

This is the 4th program in the 3-processor architecture. On real hardware,
there is no equivalent single program — the world is real and the ground
station is a separate laptop. In simulation, this program:

  1. Simulates the WORLD (building, physics, LiDAR returns, IED placement)
  2. Creates per-drone instances of all 3 processor apps (H7, N6, WL)
  3. Wires inter-processor buses (passes data between processor apps)
  4. Runs the pygame GUI (rendering, events, video, screenshots)

Architecture:
  ┌──────────────────────────────────────────────────────┐
  │              SimulationGUI (this file)                │
  │                                                      │
  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │
  │  │ H7 Flight   │  │ N6 Neural   │  │ WL Mesh     │  │
  │  │ Controller  │◄─┤ Processor   │◄─┤ Radio       │  │
  │  │  (per drone)│─►│  (per drone)│─►│  (per drone) │  │
  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │
  │         │                │                │          │
  │  ┌──────▼────────────────▼────────────────▼──────┐  │
  │  │        Simulated World (environment.py)        │  │
  │  │        Physics Engine (physics.py)             │  │
  │  │        Graphics Engine (graphics.py)           │  │
  │  └───────────────────────────────────────────────┘  │
  └──────────────────────────────────────────────────────┘

Controls:
  SPACE: Start/stop mission
  R: Reset simulation
  N: New object placement (same building)
  B: New building layout
  D: Cycle drone count (1-12)
  +/=: Increase communication range
  -: Decrease communication range
  P: Save screenshot
  ESC: Exit
"""

import pygame
import sys
import numpy as np
from typing import Tuple, List, Optional, Dict, Set
import time
import math
import cv2
import os
import importlib

# ── Simulation World (not on any processor — this IS the real world) ─────
from simulation.environment import Environment
from simulation.graphics import GraphicsEngine
from simulation.physics import PhysicsEngine

# ── Processor Apps (one instance per drone per processor) ────────────────
from core.stm32h7.stm32h7_main import FlightControllerApp
from core.stm32n6.stm32n6_main import NeuralProcessorApp
from core.stm32wl.stm32wl_main import MeshRadioApp

# ── Inter-processor bus definitions ──────────────────────────────────────
from core.processor_bus import (
    PositionUpdate, WaypointCommand, LocalMapData, RemoteMapData, PositionBeacon,
    ManualControlInput,
)
from simulation.joystick_widget import JoystickPanel

# ── Shared types (imported via processor packages) ───────────────────────
from core.stm32n6.sensors import IEDSensor
from core.stm32n6.slam import SLAMSystem
from core.stm32n6.minimap import MinimapSystem
from core.stm32n6.vision import VisionSystem
from core.stm32wl.communication import CommSystem, Message, MessagePriority
from core.stm32wl.mesh_network import SimulatedMeshProtocol, MeshMessage

# ── Legacy support: DroneManager for multi-drone coordination ────────────
from core.drone_manager import DroneManager

# Search method config
from algorithm_config import ALGORITHM as SEARCH_METHOD

# ── Simulation constants ────────────────────────────────────────────────
AUTO_START = True
SIM_SPEED = 3.0
SEARCH_CONFIG = {"coverage_radius": 2.0}


class PerDroneProcessors:
    """All 3 processor app instances for a single drone.

    On real hardware each of these runs on a separate physical chip.
    In simulation they are Python objects called sequentially."""

    def __init__(self, drone_id: int, comm_range: float,
                 building_width: float, building_height: float,
                 positions_ref: dict, message_bus: list):
        self.drone_id = drone_id

        # ── H7: Flight Controller ────────────────────────────────────
        self.h7 = FlightControllerApp(drone_id=drone_id)

        # ── N6: Neural Processor ─────────────────────────────────────
        self.n6 = NeuralProcessorApp(
            drone_id=drone_id,
            building_width=building_width,
            building_height=building_height,
        )

        # ── WL: Mesh Radio ───────────────────────────────────────────
        self.wl = MeshRadioApp(drone_id=drone_id, comm_range=comm_range)

        # Wire up mesh protocol (simulation uses shared memory bus)
        protocol = SimulatedMeshProtocol(
            drone_id=drone_id,
            comm_range=comm_range,
            positions_ref=positions_ref,
            message_bus=message_bus,
        )
        self.wl.init_mesh(protocol)
        self.protocol = protocol

    def init_all(self):
        """Initialize all 3 processors (power-on sequence)."""
        self.h7.init()
        self.n6.init()
        self.wl.init()


class DroneSimulationGUI:
    """
    Main simulation GUI — the 'ground station' + simulated world.

    This class replaces simulation_main.py's DroneSimulation with an
    architecture that cleanly separates processor concerns. Every data
    exchange between processors goes through the bus dataclasses.
    """

    def __init__(self, window_size: Tuple[int, int] = (1200, 1300)):
        self.window_size = window_size
        self.running = True
        self.clock = pygame.time.Clock()

        # ══════════════════════════════════════════════════════════════
        # SIMULATED WORLD (environment, physics, objects, IEDs)
        # On real hardware: this is the actual physical world
        # ══════════════════════════════════════════════════════════════
        self.environment = Environment()
        self.environment.randomize_objects()
        self._door_x = 25.0 * self.environment.building_scale

        # Shared simulation systems (SLAM, minimap, vision, comms)
        # These span processors in simulation but would be split on real HW
        self.slam = SLAMSystem()
        self.slam.initialize((self._door_x, -3.0), 0.0)
        self.vision = VisionSystem()
        self.comm = CommSystem()
        self.ied_sensor = IEDSensor()
        self.minimap = MinimapSystem()

        # ══════════════════════════════════════════════════════════════
        # PER-DRONE PROCESSOR INSTANCES
        # ══════════════════════════════════════════════════════════════
        self.drone_count = 1
        self.comm_range = 10.0
        self.multi_drone_mode = False
        self.drone_manager: Optional[DroneManager] = None

        # Single-drone mode: create one set of processors
        self._message_bus: List[MeshMessage] = []
        self._positions: Dict[int, Tuple[float, float]] = {0: (self._door_x, -3.0)}

        # We still use DroneManager for multi-drone (it orchestrates gossip phases)
        # For single drone, we use the H7 app's drone directly

        # Single-drone search algorithm (loaded dynamically)
        self.current_search_name = SEARCH_METHOD
        self.search_algorithm = None
        self.search_options = self._discover_search_modules()
        self.search_algorithm = self._load_search_by_module(SEARCH_METHOD)

        # ══════════════════════════════════════════════════════════════
        # SINGLE-DRONE STATE (mirrors original simulation_main.py)
        # ══════════════════════════════════════════════════════════════
        # The original code used a Drone object directly. We create one
        # via the H7 flight controller app for architectural clarity.
        from core.stm32h7.drone import Drone
        self.drone = Drone(position=(self._door_x, -3, 3))

        self._laser_target = (self._door_x, 0.0)
        self._entry_sequence = [
            (self._door_x, -0.5),
            (self._door_x, 0.5),
            (self._door_x, 2.0),
        ]
        self._entry_index = 0
        self._inside_building = False
        self._start_time = time.time()
        self._returning_home = False
        self._breadcrumbs: List[Tuple[float, float]] = []
        self._last_breadcrumb_ts = time.time()
        self._show_map = False
        self._rotation_scan_interval = 1.5
        self._last_rotation_scan = 0.0
        self._coverage_milestones_logged: Set[int] = set()
        self._return_msg_printed = False

        # ══════════════════════════════════════════════════════════════
        # RENDERING & VIDEO
        # ══════════════════════════════════════════════════════════════
        self.graphics = GraphicsEngine(window_size)
        self.graphics.scale = 10.0 / self.environment.building_scale
        self.physics = PhysicsEngine()
        self.physics.environment = self.environment

        self.mission_active = False
        self.exploration_complete = False

        # Video recording
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._video_frame_count: int = 0
        self._video_frames_written: int = 0
        self._video_filename: str = ""

        # AUTO-START
        if AUTO_START:
            self.mission_active = True
            self._start_video_recording()
            print("AUTO-START: Mission starting automatically...")

        self._auto_screenshot_last = time.time()
        self._auto_start_time = time.time()

        # Per-drone screenshots
        self._pending_drone_screenshots: List[Tuple[int, str]] = []
        self._drone_mid_screenshots_taken: Set[int] = set()
        self._drone_completion_elapsed: Dict[int, float] = {}

        # Navigation stuck detection
        self._nav_stuck_pos: Dict[int, Tuple[float, float]] = {}
        self._nav_stuck_time: Dict[int, float] = {}
        self._nav_escape_cooldown: Dict[int, float] = {}

        # Sensor data storage
        self.lidar_data = None

        # Manual drone control state
        self._selected_drone_id: Optional[int] = None
        self._manual_mode: Dict[int, bool] = {}
        self._joystick_panel: Optional[JoystickPanel] = None

        # Wall collision tracking (per drone ID -> count)
        self._wall_collisions: Dict[int, int] = {}
        self._wall_collision_cooldown: Dict[int, float] = {}  # drone -> last collision time

        # Per-drone mission system
        DRONE_MISSIONS = ["map", "destroy"]
        self._drone_missions: Dict[int, str] = {}  # default "map" for all
        self._destroyed_ieds: Set[Tuple[float, float]] = set()
        self._drone_destroying: Dict[int, Tuple[float, float]] = {}  # drone -> IED pos it's flying to
        self._mission_button_rects: list = []  # [(drone_id, pygame.Rect), ...]

    # ══════════════════════════════════════════════════════════════════
    # SEARCH ALGORITHM LOADING (dynamically loads core.search_* modules)
    # ══════════════════════════════════════════════════════════════════

    def _discover_search_modules(self) -> list:
        try:
            core_dir = os.path.join(os.path.dirname(__file__), 'core', 'stm32n6')
        except NameError:
            core_dir = os.path.join(os.getcwd(), 'core', 'stm32n6')
        options = []
        try:
            for fname in sorted(os.listdir(core_dir)):
                if fname.startswith('search_') and fname.endswith('.py'):
                    options.append(fname[:-3])
        except Exception:
            pass
        return options

    def _load_search_by_module(self, module_name: str):
        module = importlib.import_module(f"core.stm32n6.{module_name}")
        preferred = [
            'SystematicMapper', 'AutoSearch', 'SystematicSweep',
            'FrontierExplorer', 'SafeExplorer', 'FastIEDSweep',
            'LidarMapper', 'WallFollower', 'SimpleGridNavigator',
            'CoverageNavigator', 'RandomWalkNavigator',
            'STCCoverage', 'AdaptiveRoomSweep',
        ]
        for name in preferred:
            if hasattr(module, name):
                return self._instantiate_search_class(getattr(module, name))
        for attr in dir(module):
            obj = getattr(module, attr)
            if isinstance(obj, type) and all(hasattr(obj, m) for m in ['get_next_waypoint', 'reset']):
                return self._instantiate_search_class(obj)
        raise RuntimeError(f"No search class found in {module_name}")

    def _instantiate_search_class(self, cls):
        bw = self.environment.width
        bh = self.environment.height
        for kwargs in [
            dict(coverage_radius=2.0, building_width=bw, building_height=bh),
            dict(detection_radius=2.0),
            dict(coverage_radius=2.0),
            dict(sensor_range=10.0, sweep_spacing=2.5),
            {},
        ]:
            try:
                return cls(**kwargs)
            except Exception:
                continue
        raise RuntimeError(f"Cannot instantiate {cls.__name__}")

    # ══════════════════════════════════════════════════════════════════
    # MAIN LOOP
    # ══════════════════════════════════════════════════════════════════

    def run(self):
        """Main simulation loop — identical behavior to simulation_main.py."""
        print("Starting Drone Simulation (3-Processor Architecture)...")
        print("Controls: SPACE=start/stop, R=reset, D=drones, P=screenshot, ESC=exit")

        while self.running:
            dt = self.clock.tick(60) / 1000.0 * SIM_SPEED

            self._handle_events()

            # ── Per-drone timer checks (6 min return, 7 min death) ───
            if self.mission_active and self._start_time is not None:
                if self.multi_drone_mode and self.drone_manager:
                    self._check_multi_drone_timers()
                else:
                    self._check_single_drone_timers()

            if self.mission_active:
                self._update_systems(dt)

            # Check mission completion AFTER update
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
                    self._finalize_video()
                    print("Press SPACE to start new mission, R to reset, ESC to exit")

            self._render()

        self._cleanup()

    # ══════════════════════════════════════════════════════════════════
    # TIMER CHECKS
    # ══════════════════════════════════════════════════════════════════

    def _check_single_drone_timers(self):
        simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED

        if simulated_elapsed >= 180.0 and 0 not in self._drone_mid_screenshots_taken:
            self._drone_mid_screenshots_taken.add(0)
            self._pending_drone_screenshots.append((0, "mid"))

        if simulated_elapsed >= 420.0 and not getattr(self, '_time_limit_reached', False):
            self._time_limit_reached = True
            print("=" * 50)
            print(f"TIME LIMIT! 7 minutes reached ({simulated_elapsed:.0f}s simulated)")
            cov = self.search_algorithm.get_coverage_stats().get('coverage_pct', 0)
            print(f"Final coverage: {cov}%")
            print("=" * 50)
            self._pending_drone_screenshots.append((0, "done"))
            self._finalize_video()
            self.mission_active = False
            self.drone.emergency_stop()
            print("Press SPACE to start new mission, R to reset, ESC to exit")

    def _check_multi_drone_timers(self):
        now = time.time()
        all_dead = True
        for i in range(self.drone_manager.count):
            drone_i = self.drone_manager.drones[i]
            if drone_i.mission_complete:
                continue
            st = self.drone_manager.mission_start_times[i]
            if not st:
                all_dead = False
                continue
            elapsed_i = (now - st) * SIM_SPEED

            # 3 min: mid-mission screenshot
            if elapsed_i >= 180.0 and i not in self._drone_mid_screenshots_taken:
                self._drone_mid_screenshots_taken.add(i)
                self._pending_drone_screenshots.append((i, "mid"))

            # 6 min: force return
            if elapsed_i >= 360.0 and not self.drone_manager.returning_home[i]:
                self.drone_manager.returning_home[i] = True
                if self._manual_mode.get(i, False):
                    self._release_manual(i)
                    print(f"[6 MIN] D{i}: manual mode auto-released for return")
                search_i = self.drone_manager.get_search(i)
                cov = search_i.get_coverage_stats().get('coverage_pct', 0)
                print(f"[6 MIN] Drone {i}: forcing return ({elapsed_i:.0f}s sim, {cov}% coverage)")

            # 7 min: battery dead
            if elapsed_i >= 420.0:
                if not drone_i.mission_complete:
                    if self._manual_mode.get(i, False):
                        self._release_manual(i)
                    drone_i.mission_complete = True
                    drone_i.emergency_stop()
                    search_i = self.drone_manager.get_search(i)
                    cov = search_i.get_coverage_stats().get('coverage_pct', 0)
                    self._drone_completion_elapsed[i] = elapsed_i
                    print(f"[BATTERY DEAD] Drone {i}: 7 min reached ({elapsed_i:.0f}s sim, {cov}% coverage)")
                    self._pending_drone_screenshots.append((i, "done"))
            else:
                all_dead = False

        if all_dead and not getattr(self, '_time_limit_reached', False):
            self._time_limit_reached = True
            print("=" * 50)
            cov = self.drone_manager.get_global_coverage_stats()['coverage_pct']
            print(f"ALL DRONES DONE. Global coverage: {cov}%")
            print("=" * 50)
            self._finalize_video()
            self.mission_active = False
            print("Press SPACE to start new mission, R to reset, ESC to exit")

    # ══════════════════════════════════════════════════════════════════
    # EVENT HANDLING
    # ══════════════════════════════════════════════════════════════════

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            # ── Mouse events: joystick panel + mission buttons + drone selection
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                # Joystick panel gets first priority
                if self._joystick_panel and self._joystick_panel.handle_mouse_down(mx, my):
                    pass  # captured by joystick
                elif self._check_mission_button_click(mx, my):
                    pass  # captured by mission button
                else:
                    self._try_select_drone(mx, my)

            elif event.type == pygame.MOUSEMOTION:
                if self._joystick_panel:
                    mx, my = event.pos
                    self._joystick_panel.handle_mouse_motion(mx, my)

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if self._joystick_panel:
                    self._joystick_panel.handle_mouse_up()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    if self.drone.mission_complete:
                        self._reset_mission_keep_map()
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
                    print("Randomizing object placement...")
                    self.environment.randomize_objects()
                    self._reset_simulation()
                elif event.key == pygame.K_b:
                    print("Generating new building layout...")
                    self.environment.generate_new_building()
                    self._reset_simulation()
                elif event.key == pygame.K_p:
                    try:
                        self._save_screenshot()
                    except Exception as e:
                        print(f"Failed to save screenshot: {e}")
                elif event.key == pygame.K_d:
                    old_count = self.drone_count
                    self.drone_count = (self.drone_count % 12) + 1
                    print(f"Drone count: {old_count} -> {self.drone_count}")
                    self._setup_multi_drone(preserve_drone_0=True)
                elif event.key in [pygame.K_PLUS, pygame.K_EQUALS]:
                    self.comm_range = min(50.0, self.comm_range + 2.0)
                    print(f"Comm range: {self.comm_range:.0f}m")
                    if self.drone_manager:
                        self.drone_manager.set_comm_range(self.comm_range)
                elif event.key == pygame.K_MINUS:
                    self.comm_range = max(2.0, self.comm_range - 2.0)
                    print(f"Comm range: {self.comm_range:.0f}m")
                    if self.drone_manager:
                        self.drone_manager.set_comm_range(self.comm_range)
                elif event.key == pygame.K_m:
                    self._toggle_manual_mode()
                elif event.key in [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4,
                                   pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9]:
                    idx = event.key - pygame.K_1
                    if 0 <= idx < len(self.search_options):
                        new_module = self.search_options[idx]
                        try:
                            self.search_algorithm = self._load_search_by_module(new_module)
                            self.current_search_name = new_module
                            print(f"Switched search to: {new_module}")
                        except Exception as e:
                            print(f"Failed to switch search: {e}")

    # ══════════════════════════════════════════════════════════════════
    # MANUAL DRONE CONTROL
    # ══════════════════════════════════════════════════════════════════

    def _try_select_drone(self, screen_x: int, screen_y: int):
        """Click on main map to select a drone (1m hit radius)."""
        world_pos = self.graphics._screen_to_world((screen_x, screen_y))

        if self.multi_drone_mode and self.drone_manager:
            drone_list = [(i, self.drone_manager.drones[i])
                          for i in range(self.drone_manager.count)]
        else:
            drone_list = [(0, self.drone)]

        best_id = None
        best_dist = float('inf')
        for did, d in drone_list:
            dx = world_pos[0] - d.position[0]
            dy = world_pos[1] - d.position[1]
            dist = math.hypot(dx, dy)
            if dist < 1.0 and dist < best_dist:
                best_dist = dist
                best_id = did

        if best_id is not None:
            # If selecting a different drone while one is manual, release old one
            if (self._selected_drone_id is not None and
                    best_id != self._selected_drone_id and
                    self._manual_mode.get(self._selected_drone_id, False)):
                self._release_manual(self._selected_drone_id)
            self._selected_drone_id = best_id
            print(f"Selected drone D{best_id}")

    def _toggle_manual_mode(self):
        """M key: toggle manual control on selected drone."""
        did = self._selected_drone_id
        if did is None:
            print("No drone selected — click a drone first, then press M")
            return

        currently_manual = self._manual_mode.get(did, False)
        if currently_manual:
            self._release_manual(did)
        else:
            # Release any other drone that's in manual mode
            for other_id in list(self._manual_mode.keys()):
                if self._manual_mode.get(other_id, False):
                    self._release_manual(other_id)
            self._manual_mode[did] = True
            self._joystick_panel = JoystickPanel(
                self.window_size[0] - 250 + 5,
                self.window_size[1] - 140 - self.window_size[1] // 3)
            print(f"MANUAL MODE ON — D{did} (drag sticks to fly, M to release)")

    def _release_manual(self, drone_id: int):
        """Release manual control on a drone — resume autonomous search."""
        self._manual_mode[drone_id] = False
        self._joystick_panel = None
        self._selected_drone_id = None
        # Stop drone velocity so it doesn't drift
        if self.multi_drone_mode and self.drone_manager:
            if drone_id < len(self.drone_manager.drones):
                d = self.drone_manager.drones[drone_id]
                d.velocity[0] = 0.0
                d.velocity[1] = 0.0
                d.angular_velocity = 0.0
        else:
            self.drone.velocity[0] = 0.0
            self.drone.velocity[1] = 0.0
            self.drone.angular_velocity = 0.0
        print(f"MANUAL MODE OFF — D{drone_id} resuming autonomous")

    def _check_mission_button_click(self, mx: int, my: int) -> bool:
        """Check if click hit a mission button. Toggle mission on click."""
        for drone_id, rect in self._mission_button_rects:
            if rect.collidepoint(mx, my):
                current = self._drone_missions.get(drone_id, "map")
                new_mission = "destroy" if current == "map" else "map"
                self._drone_missions[drone_id] = new_mission
                print(f"D{drone_id} mission: {current.upper()} -> {new_mission.upper()}")
                # If switching to destroy, immediately look for known IEDs
                if new_mission == "destroy":
                    ied_pos = self._find_known_ied(drone_id)
                    if ied_pos:
                        self._drone_destroying[drone_id] = ied_pos
                        print(f"[DESTROY] D{drone_id}: targeting known IED at ({ied_pos[0]:.1f},{ied_pos[1]:.1f})")
                    else:
                        print(f"[DESTROY] D{drone_id}: no IED known yet, searching...")
                else:
                    # Switching back to map — cancel any destroy target
                    if drone_id in self._drone_destroying:
                        del self._drone_destroying[drone_id]
                        print(f"D{drone_id}: destroy cancelled, resuming MAP")
                return True
        return False

    def _find_known_ied(self, drone_id: int) -> Optional[Tuple[float, float]]:
        """Find closest known IED position from drone's map/gossip data.
        Returns None if no IED is known."""
        known_ieds = []

        if self.multi_drone_mode and self.drone_manager:
            # Multi-drone: check gossip features for IEDs
            gossip = self.drone_manager.gossip_maps[drone_id]
            for f in gossip.get_features_by_type('ied'):
                rounded = (round(f.position[0], 1), round(f.position[1], 1))
                if rounded not in self._destroyed_ieds:
                    known_ieds.append(f.position)
        else:
            # Single-drone: check minimap discovered features
            from core.stm32n6.minimap import FeatureType
            for f in self.minimap.discovered_features:
                if f.feature_type == FeatureType.IED:
                    rounded = (round(f.position[0], 1), round(f.position[1], 1))
                    if rounded not in self._destroyed_ieds:
                        known_ieds.append(f.position)

        if not known_ieds:
            return None

        # Return closest IED to the drone
        if self.multi_drone_mode and self.drone_manager:
            drone = self.drone_manager.drones[drone_id]
        else:
            drone = self.drone
        dx = float(drone.position[0])
        dy = float(drone.position[1])
        best = min(known_ieds, key=lambda p: math.hypot(p[0] - dx, p[1] - dy))
        return best

    def _apply_manual_input_to_drone(self, drone, search_algo):
        """Read joystick axes and apply velocity to drone in body frame.
        Also marks current cell + cardinal neighbors as searched (IED sensor still on)."""
        if not self._joystick_panel:
            return
        lx, ly, rx, ry = self._joystick_panel.get_axes()
        ori = drone.orientation
        max_speed = 2.0
        max_yaw = 1.0

        fwd = ly * max_speed
        strafe = lx * max_speed
        vx = fwd * math.cos(ori) - strafe * math.sin(ori)
        vy = fwd * math.sin(ori) + strafe * math.cos(ori)

        # Wall collision: check if the next position would cross a wall.
        # physics.update_drone only checks the destination, not the path,
        # so the drone can tunnel through thin walls at speed.
        px, py = float(drone.position[0]), float(drone.position[1])
        lookahead = 0.15  # seconds lookahead (~3 frames at 60fps)
        next_x = px + vx * lookahead
        next_y = py + vy * lookahead
        if not self.environment.is_path_clear((px, py), (next_x, next_y), radius=0.2):
            vx = 0.0
            vy = 0.0

        drone.velocity[0] = vx
        drone.velocity[1] = vy
        drone.angular_velocity = rx * max_yaw
        drone.current_waypoint = None

        # Mark cells as searched (IED detector still scanning during manual flight)
        # Mirrors the cell-marking logic in get_next_waypoint() of the search algorithm
        if hasattr(search_algo, '_to_grid') and hasattr(search_algo, 'searched_cells'):
            px, py = float(drone.position[0]), float(drone.position[1])
            curr_cell = search_algo._to_grid(px, py)
            if curr_cell in search_algo.free_cells:
                search_algo.searched_cells.add(curr_cell)
            for ddx, ddy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                neighbor = (curr_cell[0] + ddx, curr_cell[1] + ddy)
                if (neighbor in search_algo.free_cells and
                        neighbor not in search_algo.wall_cells and
                        curr_cell not in search_algo.wall_cells):
                    search_algo.searched_cells.add(neighbor)

    # ══════════════════════════════════════════════════════════════════
    # SYSTEM UPDATES
    # ══════════════════════════════════════════════════════════════════

    def _update_systems(self, dt: float):
        """Update all simulation systems — delegates to single or multi-drone."""
        if self.multi_drone_mode and self.drone_manager:
            self._update_multi_drone(dt)
            if self.drone_manager.drones:
                self.drone = self.drone_manager.drones[0]
            return

        # ══════════════════════════════════════════════════════════════
        # SINGLE-DRONE UPDATE
        # Shows processor boundaries: H7 does physics, N6 does search
        # ══════════════════════════════════════════════════════════════

        # ── H7: Flight controller update ─────────────────────────────
        self.drone.update(dt)
        blocked_speed = self.physics.update_drone(self.drone, dt)
        # Only count as collision if drone had real speed (>0.5 m/s) and
        # at least 2s since last collision for this drone (avoid counting
        # the same wall encounter many times).
        if blocked_speed > 0.5:
            now_col = time.time()
            if now_col - self._wall_collision_cooldown.get(0, 0) > 2.0:
                self._wall_collision_cooldown[0] = now_col
                self._wall_collisions[0] = self._wall_collisions.get(0, 0) + 1
                count = self._wall_collisions[0]
                x, y = float(self.drone.position[0]), float(self.drone.position[1])
                print(f"[COLLISION] D0: wall hit #{count} at ({x:.1f},{y:.1f}) ({blocked_speed:.1f} m/s)")

        # ── H7: Breadcrumb recording ────────────────────────────────
        if (time.time() - self._last_breadcrumb_ts) >= 0.2:
            self._breadcrumbs.append((float(self.drone.position[0]), float(self.drone.position[1])))
            if len(self._breadcrumbs) > 2000:
                self._breadcrumbs = self._breadcrumbs[-2000:]
            self._last_breadcrumb_ts = time.time()

        # ── WORLD: Sensor readings (simulated environment) ───────────
        self.lidar_data = self.environment.get_lidar_scan(self.drone.position, self.drone.orientation)
        camera_image = self.environment.get_camera_view(self.drone.position, self.drone.orientation)

        # ── N6: SLAM + minimap update ────────────────────────────────
        self.slam.update(self.drone.position[:2], self.lidar_data)
        if self.lidar_data:
            self.minimap.add_lidar_scan(self.drone.position[:2], self.lidar_data, self.drone.orientation)

        # ── N6: Periodic rotation scan (360° from 59° FoV) ──────────
        if self._inside_building:
            simulated_now = (time.time() - self._start_time) * SIM_SPEED if self._start_time else 0
            if simulated_now - self._last_rotation_scan >= self._rotation_scan_interval:
                self._last_rotation_scan = simulated_now
                base_ori = self.drone.orientation
                hfov = self.environment.lidar_hfov
                num_extra_scans = max(1, int(2 * math.pi / hfov) - 1)
                for scan_i in range(1, num_extra_scans + 1):
                    scan_angle = base_ori + scan_i * hfov
                    extra_scan = self.environment.get_lidar_scan(self.drone.position, scan_angle)
                    self.slam.update(self.drone.position[:2], extra_scan)
                    self.minimap.add_lidar_scan(self.drone.position[:2], extra_scan, scan_angle)
                    if hasattr(self.search_algorithm, 'feed_lidar_scan'):
                        self.search_algorithm.feed_lidar_scan(
                            float(self.drone.position[0]), float(self.drone.position[1]),
                            extra_scan, scan_angle)

        # ── N6: Coverage milestone logging ───────────────────────────
        if self._inside_building and hasattr(self.search_algorithm, 'get_coverage_stats'):
            cov_pct = self.search_algorithm.get_coverage_stats().get('coverage_pct', 0)
            for milestone in [25, 50, 75, 90]:
                if cov_pct >= milestone and milestone not in self._coverage_milestones_logged:
                    self._coverage_milestones_logged.add(milestone)
                    sim_elapsed = (time.time() - self._start_time) * SIM_SPEED if self._start_time else 0
                    free_n = len([c for c in self.search_algorithm.free_cells if c[1] >= 0])
                    print(f"[COVERAGE] {milestone}% at {sim_elapsed:.0f}s sim | {free_n} free cells discovered")

        # ── N6: Vision processing ────────────────────────────────────
        detected_objects = self.vision.process_frame(camera_image)
        if detected_objects:
            for detection in detected_objects:
                self.minimap.add_vision_detection(
                    detection.position,
                    detection.object_type.value if hasattr(detection.object_type, 'value') else str(detection.object_type),
                    detection.confidence)

        # ── N6: IED sensor ───────────────────────────────────────────
        ied_reading = self.ied_sensor.read(
            (float(self.drone.position[0]), float(self.drone.position[1])), self.environment)
        if ied_reading and ied_reading.confidence > 0.2:
            text = ied_reading.to_text()
            print(f"IED ALERT: {text}")
            ied_pos = ied_reading.ied_position or (float(self.drone.position[0]), float(self.drone.position[1]))
            self.minimap.add_ied_detection(ied_pos, ied_reading.confidence)
            self.comm.message_queue.put(Message(
                id=str(time.time()), priority=MessagePriority.HIGH,
                message_type="ied_alert", timestamp=time.time(),
                data={"text": text}))
            self.graphics.notify_radio_sent()

        # ── Destroy mission: look for known IEDs or check arrival ──
        if self._drone_missions.get(0, "map") == "destroy" and 0 not in self._drone_destroying:
            # Not targeting an IED yet — check map for known ones
            ied_pos = self._find_known_ied(0)
            if ied_pos:
                self._drone_destroying[0] = ied_pos
                print(f"[DESTROY] D0: targeting known IED at ({ied_pos[0]:.1f},{ied_pos[1]:.1f})")
            # else: keep searching normally, will check again next frame

        if 0 in self._drone_destroying:
            ied_target = self._drone_destroying[0]
            dist_to_ied = math.hypot(
                float(self.drone.position[0]) - ied_target[0],
                float(self.drone.position[1]) - ied_target[1])
            if dist_to_ied < 0.5:
                print(f"[DESTROY] D0: IED destroyed at ({ied_target[0]:.1f},{ied_target[1]:.1f})")
                rounded_pos = (round(ied_target[0], 1), round(ied_target[1], 1))
                self._destroyed_ieds.add(rounded_pos)
                self.environment.objects_of_interest = [
                    obj for obj in self.environment.objects_of_interest
                    if not (obj[2] == 'ied' and
                            abs(obj[0] - ied_target[0]) < 0.5 and
                            abs(obj[1] - ied_target[1]) < 0.5)]
                del self._drone_destroying[0]
                self._drone_missions[0] = "map"
                print(f"[DESTROY] D0: IED eliminated, resuming MAP mission")

        # ── Manual control bypass: pilot flies directly ────────────
        if self._manual_mode.get(0, False):
            self._apply_manual_input_to_drone(self.drone, self.search_algorithm)
            # IED sensor still runs during manual flight
            ied_reading = self.ied_sensor.read(
                (float(self.drone.position[0]), float(self.drone.position[1])), self.environment)
            if ied_reading and ied_reading.confidence > 0.2:
                text = ied_reading.to_text()
                print(f"IED ALERT: {text}")
                ied_pos = ied_reading.ied_position or (float(self.drone.position[0]), float(self.drone.position[1]))
                self.minimap.add_ied_detection(ied_pos, ied_reading.confidence)
            return  # Skip autonomous navigation

        # ── N6→H7: Navigation — entry sequence or search ────────────
        next_target_2d = None

        if not self._inside_building and self._entry_index < len(self._entry_sequence):
            target = self._entry_sequence[self._entry_index]
            dist_to_target = np.linalg.norm(np.array(target) - self.drone.position[:2])

            if not hasattr(self, '_entry_wp_time'):
                self._entry_wp_time = time.time()
            entry_wp_elapsed = time.time() - self._entry_wp_time
            tolerance = 0.8 if entry_wp_elapsed < 3.0 else 2.0

            if dist_to_target < tolerance:
                self._entry_index += 1
                self._entry_wp_time = time.time()
                if self._entry_index >= len(self._entry_sequence):
                    self._inside_building = True
                    self._start_time = time.time()
                    self._do_full_rotation_scan()
                    print(f"Entered building, beginning {self.current_search_name} search...")
            elif entry_wp_elapsed > 5.0 and self.drone.position[1] > -4.0:
                print(f"Entry waypoint stuck for {entry_wp_elapsed:.0f}s, advancing...")
                self._entry_index += 1
                self._entry_wp_time = time.time()
                if self._entry_index >= len(self._entry_sequence):
                    self._inside_building = True
                    self._start_time = time.time()
                    self._do_full_rotation_scan()
                    print("Forced entry - beginning search...")
            else:
                waypoint_3d = (target[0], target[1], float(self.drone.position[2]))
                self.drone.navigate_to(waypoint_3d)
                next_target_2d = "handled"

        # ── N6: Search algorithm decision ────────────────────────────
        if next_target_2d is None and not self._returning_home:
            simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED
            if hasattr(self.search_algorithm, 'set_simulated_time'):
                self.search_algorithm.set_simulated_time(simulated_elapsed)

            next_target_2d = self.search_algorithm.get_next_waypoint(
                self.drone.position[:2],
                self.lidar_data if hasattr(self, 'lidar_data') else [],
                self.drone.orientation)

            if next_target_2d is None:
                self._do_full_rotation_scan()
                next_target_2d = self.search_algorithm.get_next_waypoint(
                    self.drone.position[:2],
                    self.lidar_data if hasattr(self, 'lidar_data') else [],
                    self.drone.orientation)

            # Still no target — move toward most open LiDAR direction
            if next_target_2d is None and self.lidar_data:
                next_target_2d = self._lidar_fallback_target(
                    self.drone.position, self.drone.orientation, self.lidar_data)

            search_complete = False
            if hasattr(self.search_algorithm, 'is_mission_complete'):
                search_complete = self.search_algorithm.is_mission_complete()
            search_returning = getattr(self.search_algorithm, '_mode', '') == "RETURN"

            if (search_complete or search_returning) and not self._returning_home:
                self._returning_home = True
                if not self._return_msg_printed:
                    cov = self.search_algorithm.get_coverage_stats().get('coverage_pct', 0)
                    print(f"[HOME] Returning — coverage: {cov}%")
                    self._return_msg_printed = True

        elif next_target_2d is None and self._returning_home:
            dist_home_now = np.linalg.norm(np.array(self.slam.entry_point) - self.drone.position[:2])
            if (dist_home_now < 1.5 and self.drone.position[1] < 0.5) or self.drone.position[1] < -2.0:
                self.drone.emergency_stop()
                self.drone.mission_complete = True
                self.mission_active = False
                print("=" * 50)
                print("HOME! Mission complete - drone returned to start.")
                print("=" * 50)
                self._pending_drone_screenshots.append((0, "done"))
                self._finalize_video()
                print("Press SPACE to start new mission, R to reset, ESC to exit")
                return
            next_target_2d = self.slam.entry_point

        # ── Destroy mission: override target to fly to IED ──────────
        if 0 in self._drone_destroying:
            next_target_2d = self._drone_destroying[0]

        # ── H7: Execute navigation (A* pathfinding + stuck escape) ───
        if next_target_2d and next_target_2d != "handled":
            escaped = (self._inside_building and
                       self._nav_escape_if_stuck(self.drone, 0, self.search_algorithm))

            current_2d = (float(self.drone.position[0]), float(self.drone.position[1]))
            path = self.slam.path_planner.get_path_to_next_point(current_2d, next_target_2d, self.environment)

            if escaped:
                pass
            elif path and len(path) > 0:
                chosen = None
                for s in ([3, 2, 1] if self._inside_building else [2, 1]):
                    idx = min(len(path) - 1, s)
                    step = path[idx]
                    if (self.environment.is_position_valid((step[0], step[1]), radius=0.2) and
                            self.environment.is_path_clear(current_2d, (step[0], step[1]))):
                        chosen = step
                        break
                if chosen is None:
                    chosen = path[min(len(path) - 1, 1)]
                if self.environment.is_position_valid((chosen[0], chosen[1]), radius=0.2):
                    waypoint_3d = (chosen[0], chosen[1], float(self.drone.position[2]))
                    self.drone.navigate_to(waypoint_3d)
            else:
                if (hasattr(self, 'search_algorithm') and
                        hasattr(self.search_algorithm, 'mark_target_unreachable') and
                        not self._returning_home):
                    self.search_algorithm.mark_target_unreachable()
                self._fallback_navigation(self.drone, current_2d, next_target_2d)

        elif next_target_2d != "handled":
            if self._returning_home:
                dist_home = np.linalg.norm(np.array(self.slam.entry_point) - self.drone.position[:2])
                if (dist_home < 1.5 and self.drone.position[1] < 0.5) or self.drone.position[1] < -2.0:
                    self.drone.emergency_stop()
                    self.drone.mission_complete = True
                    self.mission_active = False
                    print("=" * 50)
                    print("HOME! Mission complete - drone returned to start.")
                    print("=" * 50)
                    self._finalize_video()
                    print("Press SPACE to start new mission, R to reset, ESC to exit")

        # ── WL: Communication (radio map updates) ────────────────────
        if detected_objects:
            self.comm.send_priority_data(detected_objects, self.drone.position[:2])

        progress = self.slam.get_exploration_progress()
        if getattr(self, '_last_points_visited', None) is None:
            self._last_points_visited = progress['points_visited']
        if progress['points_visited'] > self._last_points_visited:
            self._last_points_visited = progress['points_visited']
            self.slam.room_exit_events += 1
            self.comm.send_map_update(self.slam.get_map())
            self.graphics.notify_radio_sent()

    # ══════════════════════════════════════════════════════════════════
    # MULTI-DRONE UPDATE
    # ══════════════════════════════════════════════════════════════════

    def _update_multi_drone(self, dt: float):
        """Update all drones — shows H7/N6/WL processor boundaries."""
        if not self.drone_manager:
            return

        # Start ALL drone clocks on first call
        for i in range(self.drone_manager.count):
            if self.drone_manager.mission_start_times[i] is None:
                self.drone_manager.mission_start_times[i] = time.time()

        # Periodic status logging
        if not hasattr(self, '_last_status_log'):
            self._last_status_log = 0
        now = time.time()
        if now - self._last_status_log > 30:
            self._last_status_log = now
            stats = self.drone_manager.get_global_coverage_stats()
            gossip_links = self.drone_manager.get_gossip_links()
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

        # Store lidar_data for drone 0 (render compatibility)
        self.lidar_data = self.environment.get_lidar_scan(
            self.drone_manager.drones[0].position,
            self.drone_manager.drones[0].orientation)

        # ── H7 (all drones): Breadcrumbs ────────────────────────────
        self.drone_manager.update_breadcrumbs()

        # ── WL (all drones): Mesh networking ─────────────────────────
        messages = self.drone_manager.update_mesh()
        self.drone_manager.process_gossip(messages)

        # ── WL: Gossip Phase 1 — local → gossip ─────────────────────
        for i in range(self.drone_manager.count):
            self.drone_manager.sync_local_to_gossip(i)

        # ── WL: Gossip Phase 2 — gossip ↔ gossip ────────────────────
        self.drone_manager.update_positions()
        self.drone_manager.broadcast_map_updates()

        # ── Per-drone update loop (H7 + N6 + WL Phase 3) ────────────
        for i in range(self.drone_manager.count):
            drone = self.drone_manager.get_drone(i)
            search = self.drone_manager.get_search(i)
            gossip = self.drone_manager.get_gossip_map(i)

            # Skip dead/completed drones
            if drone.mission_complete:
                continue

            # ── H7: Physics update ───────────────────────────────────
            drone.update(dt)
            blocked_speed = self.physics.update_drone(drone, dt)
            if blocked_speed > 0.5:
                now_col = time.time()
                if now_col - self._wall_collision_cooldown.get(i, 0) > 2.0:
                    self._wall_collision_cooldown[i] = now_col
                    self._wall_collisions[i] = self._wall_collisions.get(i, 0) + 1
                    count = self._wall_collisions[i]
                    cx, cy = float(drone.position[0]), float(drone.position[1])
                    print(f"[COLLISION] D{i}: wall hit #{count} at ({cx:.1f},{cy:.1f}) ({blocked_speed:.1f} m/s)")

            # ── Entry sequence (H7 navigation + N6 SLAM) ────────────
            if not self.drone_manager.inside_building[i]:
                entry_seq = self.drone_manager.entry_sequences[i]
                entry_idx = self.drone_manager.entry_indices[i]

                # N6: SLAM during entry
                lidar_data = self.environment.get_lidar_scan(drone.position, drone.orientation)
                if lidar_data:
                    self.slam.update(drone.position[:2], lidar_data)
                    self.minimap.add_lidar_scan(drone.position[:2], lidar_data, drone.orientation)

                # Entry stuck timeout
                if not hasattr(self, '_entry_start_times'):
                    self._entry_start_times = {}
                if i not in self._entry_start_times:
                    self._entry_start_times[i] = time.time()

                entry_elapsed = time.time() - self._entry_start_times[i]
                if entry_elapsed > 10.0 and drone.position[1] > -2:
                    self.drone_manager.inside_building[i] = True
                    self.drone_manager.entry_indices[i] = len(entry_seq)
                    self._do_multi_drone_rotation_scan(i, drone, search)
                    print(f"Drone {i} forced entry after {entry_elapsed:.0f}s")
                    del self._entry_start_times[i]
                    continue

                if entry_idx < len(entry_seq):
                    target = entry_seq[entry_idx]
                    dist = np.linalg.norm(np.array(target) - drone.position[:2])
                    if dist < 1.5:
                        self.drone_manager.entry_indices[i] += 1
                        if self.drone_manager.entry_indices[i] >= len(entry_seq):
                            self.drone_manager.inside_building[i] = True
                            self._do_multi_drone_rotation_scan(i, drone, search)
                            print(f"Drone {i} entered building")
                            if i in self._entry_start_times:
                                del self._entry_start_times[i]
                    else:
                        path = self.slam.path_planner.get_path_to_next_point(
                            (float(drone.position[0]), float(drone.position[1])),
                            target, self.environment)
                        if path and len(path) > 0:
                            next_step = path[min(len(path)-1, 1)]
                            current_pos = (float(drone.position[0]), float(drone.position[1]))
                            if (self.environment.is_position_valid((next_step[0], next_step[1]), radius=0.2) and
                                    self.environment.is_path_clear(current_pos, (next_step[0], next_step[1]))):
                                drone.navigate_to((next_step[0], next_step[1], float(drone.position[2])))
                        else:
                            current_pos = (float(drone.position[0]), float(drone.position[1]))
                            if (self.environment.is_position_valid((target[0], target[1]), radius=0.2) and
                                    self.environment.is_path_clear(current_pos, (target[0], target[1]))):
                                drone.navigate_to((target[0], target[1], float(drone.position[2])))
                continue

            # ── N6: LiDAR + SLAM (inside building) ──────────────────
            lidar_data = self.environment.get_lidar_scan(drone.position, drone.orientation)
            self.slam.update(drone.position[:2], lidar_data)
            if lidar_data:
                self.minimap.add_lidar_scan(drone.position[:2], lidar_data, drone.orientation)

            # ── N6: Periodic rotation scan ───────────────────────────
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
                        if hasattr(search, 'feed_lidar_scan'):
                            search.feed_lidar_scan(
                                float(drone.position[0]), float(drone.position[1]),
                                extra, scan_angle)

            # ── N6: Vision processing (camera object detection) ──────
            camera_image = self.environment.get_camera_view(drone.position, drone.orientation)
            detected_objects = self.vision.process_frame(camera_image)
            if detected_objects:
                for detection in detected_objects:
                    obj_type = detection.object_type.value if hasattr(detection.object_type, 'value') else str(detection.object_type)
                    gossip.add_local_feature(obj_type, detection.position, detection.confidence)

            # ── WL→N6: Gossip Phase 3 — push knowledge to search ────
            self.drone_manager.update_search_from_gossip(i)
            claimed = self.drone_manager.get_claimed_by_others(i)
            search.set_claimed_by_others(claimed)

            # ── Manual control bypass for this drone ──────────────────
            if self._manual_mode.get(i, False):
                self._apply_manual_input_to_drone(drone, search)
                # IED sensor still runs
                ied_reading = self.ied_sensor.read(
                    (float(drone.position[0]), float(drone.position[1])), self.environment)
                if ied_reading and ied_reading.confidence > 0.2:
                    print(f"Drone {i} IED ALERT: {ied_reading.to_text()}")
                    ied_pos = ied_reading.ied_position or tuple(drone.position[:2])
                    gossip.add_local_feature("ied", ied_pos, ied_reading.confidence)
                continue  # Skip autonomous navigation for this drone

            # ── N6: Search algorithm decision ────────────────────────
            start_time = self.drone_manager.mission_start_times[i]
            if start_time:
                simulated_elapsed = (time.time() - start_time) * SIM_SPEED
                if hasattr(search, 'set_simulated_time'):
                    search.set_simulated_time(simulated_elapsed)
                search_complete = search.is_mission_complete()
                if search_complete and not self.drone_manager.returning_home[i]:
                    self.drone_manager.returning_home[i] = True
                    print(f"Drone {i} search complete, returning")

            # ── N6→H7: Navigation target ─────────────────────────────
            if self.drone_manager.returning_home[i]:
                home = self.slam.entry_point
                dist_home = np.linalg.norm(np.array(home) - drone.position[:2])
                if dist_home < 1.5 and drone.position[1] < 0.5:
                    if not drone.mission_complete:
                        drone.mission_complete = True
                        cov = search.get_coverage_stats()['coverage_pct']
                        start_t = self.drone_manager.mission_start_times[i]
                        if start_t:
                            self._drone_completion_elapsed[i] = (time.time() - start_t) * SIM_SPEED
                        print(f"Drone {i} mission complete! Coverage: {cov}%")
                        self._pending_drone_screenshots.append((i, "done"))

                target = home
                current_pos = (float(drone.position[0]), float(drone.position[1]))
                path = self.slam.path_planner.get_path_to_next_point(
                    current_pos, home, self.environment)
                if not path or len(path) <= 1:
                    dx = home[0] - current_pos[0]
                    dy = home[1] - current_pos[1]
                    angle_home = math.atan2(dy, dx)
                    for hop in [8.0, 5.0, 3.0]:
                        if hop >= dist_home:
                            continue
                        intermediate = (
                            current_pos[0] + hop * math.cos(angle_home),
                            current_pos[1] + hop * math.sin(angle_home))
                        hop_path = self.slam.path_planner.get_path_to_next_point(
                            current_pos, intermediate, self.environment)
                        if hop_path and len(hop_path) > 1:
                            target = intermediate
                            break
            else:
                target = search.get_next_waypoint(
                    drone.position[:2], lidar_data, drone.orientation)

                if target is None:
                    self._do_multi_drone_rotation_scan(i, drone, search)
                    target = search.get_next_waypoint(
                        drone.position[:2], lidar_data, drone.orientation)
                if target is None and lidar_data:
                    target = self._lidar_fallback_target(drone.position, drone.orientation, lidar_data)

                if getattr(search, '_mode', '') == "RETURN" and not self.drone_manager.returning_home[i]:
                    self.drone_manager.returning_home[i] = True
                    cov = search.get_coverage_stats().get('coverage_pct', 0)
                    st = self.drone_manager.mission_start_times[i]
                    sim_t = int((time.time() - st) * SIM_SPEED) if st else 0
                    print(f"[HOME] D{i}: RETURN at {sim_t}s sim, {cov}% coverage, exit={search.exit_point}")

                if target and search.entry_point:
                    if abs(target[0] - search.entry_point[0]) < 0.1 and abs(target[1] - search.entry_point[1]) < 0.1:
                        self.drone_manager.returning_home[i] = True

                if target and search.target_cell:
                    self.drone_manager.claim_target(i, search.target_cell)

                if target is None and lidar_data:
                    target = self._lidar_fallback_target(drone.position, drone.orientation, lidar_data)

            # ── Destroy mission: override target to fly to IED ───────
            if i in self._drone_destroying:
                target = self._drone_destroying[i]

            # ── H7: Execute navigation ───────────────────────────────
            if target:
                escaped = (self.drone_manager.inside_building[i] and
                           self._nav_escape_if_stuck(drone, i, search))

                current_pos = (float(drone.position[0]), float(drone.position[1]))
                path = self.slam.path_planner.get_path_to_next_point(
                    current_pos, target, self.environment)

                if escaped:
                    pass
                elif path and len(path) > 1:
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
                        drone.navigate_to((chosen[0], chosen[1], float(drone.position[2])))
                    else:
                        path = None

                if not escaped and (not path or len(path) <= 1):
                    if (hasattr(search, 'mark_target_unreachable') and
                            not self.drone_manager.returning_home[i]):
                        search.mark_target_unreachable()
                    self._fallback_navigation(drone, current_pos, target)

            # ── N6: IED sensor for this drone ────────────────────────
            ied_reading = self.ied_sensor.read(
                (float(drone.position[0]), float(drone.position[1])), self.environment)
            if ied_reading and ied_reading.confidence > 0.2:
                print(f"Drone {i} IED ALERT: {ied_reading.to_text()}")
                ied_pos = ied_reading.ied_position or tuple(drone.position[:2])
                gossip.add_local_feature("ied", ied_pos, ied_reading.confidence)

            # ── Destroy mission: look for known IEDs or check arrival ─
            if self._drone_missions.get(i, "map") == "destroy" and i not in self._drone_destroying:
                # Not targeting yet — search gossip map for known IEDs
                ied_pos = self._find_known_ied(i)
                if ied_pos:
                    self._drone_destroying[i] = ied_pos
                    print(f"[DESTROY] D{i}: targeting known IED at ({ied_pos[0]:.1f},{ied_pos[1]:.1f})")
                # else: keep searching normally, will check again next frame

            if i in self._drone_destroying:
                ied_target = self._drone_destroying[i]
                dist_to_ied = math.hypot(
                    float(drone.position[0]) - ied_target[0],
                    float(drone.position[1]) - ied_target[1])
                if dist_to_ied < 0.5:
                    print(f"[DESTROY] D{i}: IED destroyed at ({ied_target[0]:.1f},{ied_target[1]:.1f})")
                    rounded_pos = (round(ied_target[0], 1), round(ied_target[1], 1))
                    self._destroyed_ieds.add(rounded_pos)
                    # Remove IED from environment objects
                    self.environment.objects_of_interest = [
                        obj for obj in self.environment.objects_of_interest
                        if not (obj[2] == 'ied' and
                                abs(obj[0] - ied_target[0]) < 0.5 and
                                abs(obj[1] - ied_target[1]) < 0.5)]
                    del self._drone_destroying[i]
                    # Revert to map mission
                    self._drone_missions[i] = "map"
                    print(f"[DESTROY] D{i}: IED eliminated, resuming MAP mission")

    # ══════════════════════════════════════════════════════════════════
    # NAVIGATION HELPERS
    # ══════════════════════════════════════════════════════════════════

    def _lidar_fallback_target(self, position, orientation, lidar_data) -> Optional[Tuple[float, float]]:
        """Move toward the most open LiDAR direction when search has no target."""
        if isinstance(lidar_data, dict):
            ranges = lidar_data["ranges"]
            _sa = lidar_data["start_angle"]
            _as = lidar_data["angle_step"]
        else:
            ranges = lidar_data
            _sa = orientation
            _as = 2 * math.pi / len(ranges) if ranges else 0
        best_angle, best_dist = orientation, 0
        for idx, d in enumerate(ranges):
            if d > best_dist:
                best_dist = d
                best_angle = _sa + idx * _as
        step = min(best_dist * 0.5, 2.0)
        if step > 0.3:
            tp = (position[0] + step * math.cos(best_angle),
                  position[1] + step * math.sin(best_angle))
            if self.environment.is_position_valid(tp, radius=0.2):
                return tp
        return None

    def _fallback_navigation(self, drone, current_pos, target):
        """Multi-angle fallback when A* fails."""
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
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

    def _nav_escape_if_stuck(self, drone, drone_id: int, search_algo=None) -> bool:
        """Detect stuck drone at wall/door edge and force escape move."""
        pos = (float(drone.position[0]), float(drone.position[1]))
        now = time.time()

        last_escape = self._nav_escape_cooldown.get(drone_id, 0)
        if now - last_escape < 3.0:
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
            self._nav_stuck_pos[drone_id] = pos
            self._nav_stuck_time[drone_id] = now
            return False

        elapsed = now - ref_time
        if elapsed < 1.5:
            return False

        if search_algo and hasattr(search_algo, 'mark_target_unreachable'):
            search_algo.mark_target_unreachable()

        escape_base = drone.orientation + math.pi
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

        self._nav_stuck_pos[drone_id] = pos
        self._nav_stuck_time[drone_id] = now
        self._nav_escape_cooldown[drone_id] = now
        return moved

    # ══════════════════════════════════════════════════════════════════
    # ROTATION SCANS
    # ══════════════════════════════════════════════════════════════════

    def _do_full_rotation_scan(self):
        """360° rotation scan — single drone."""
        base_ori = self.drone.orientation
        hfov = self.environment.lidar_hfov
        num_scans = max(1, int(2 * math.pi / hfov))
        for si in range(num_scans):
            scan_angle = base_ori + si * hfov
            scan_data = self.environment.get_lidar_scan(self.drone.position, scan_angle)
            if scan_data:
                self.slam.update(self.drone.position[:2], scan_data)
                self.minimap.add_lidar_scan(self.drone.position[:2], scan_data, scan_angle)
                if hasattr(self.search_algorithm, 'feed_lidar_scan'):
                    self.search_algorithm.feed_lidar_scan(
                        float(self.drone.position[0]), float(self.drone.position[1]),
                        scan_data, scan_angle)
        self._last_rotation_scan = (time.time() - self._start_time) * SIM_SPEED if self._start_time else 0

    def _do_multi_drone_rotation_scan(self, drone_idx: int, drone, search):
        """360° rotation scan — multi-drone."""
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
                        scan_data, scan_angle)

    # ══════════════════════════════════════════════════════════════════
    # MULTI-DRONE SETUP
    # ══════════════════════════════════════════════════════════════════

    def _setup_multi_drone(self, preserve_drone_0: bool = False):
        if self.drone_count > 1:
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
                existing_states.append({
                    'position': self.drone.position.copy(),
                    'orientation': self.drone.orientation,
                    'inside_building': self._inside_building,
                    'returning_home': self._returning_home,
                    'entry_idx': self._entry_index,
                    'breadcrumbs': self._breadcrumbs.copy(),
                    'mission_start_time': self._start_time if self._inside_building else None,
                })
                if hasattr(self, 'search_algorithm'):
                    existing_search_data.append({
                        'free_cells': self.search_algorithm.free_cells.copy(),
                        'searched_cells': self.search_algorithm.searched_cells.copy(),
                        'wall_cells': self.search_algorithm.wall_cells.copy(),
                        'target': self.search_algorithm.target,
                        'target_cell': self.search_algorithm.target_cell,
                        'entry_point': self.search_algorithm.entry_point,
                        'exit_point': self.search_algorithm.exit_point,
                        'start_time': self.search_algorithm.start_time,
                    })

            self.multi_drone_mode = True
            self.drone_manager = DroneManager(
                count=self.drone_count, comm_range=self.comm_range,
                building_width=self.environment.width,
                building_height=self.environment.height)
            self.drone_manager.initialize(entry_point=(self._door_x, -3.0))

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
                        saved_start = state.get('mission_start_time')
                        self.drone_manager.mission_start_times[i] = saved_start if saved_start else time.time()

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

    # ══════════════════════════════════════════════════════════════════
    # RENDERING (Ground Station Display)
    # ══════════════════════════════════════════════════════════════════

    def _render(self):
        self.graphics.clear()
        self.graphics.draw_environment(self.environment)

        # Destroyed IED markers (crossed-out red circles)
        for died_pos in self._destroyed_ieds:
            center = self.graphics._world_to_screen(died_pos)
            pygame.draw.circle(self.graphics.screen, (255, 0, 0), center, 8, 2)
            pygame.draw.line(self.graphics.screen, (255, 0, 0),
                             (center[0] - 6, center[1] - 6),
                             (center[0] + 6, center[1] + 6), 2)
            pygame.draw.line(self.graphics.screen, (255, 0, 0),
                             (center[0] + 6, center[1] - 6),
                             (center[0] - 6, center[1] + 6), 2)

        # Laser target on door
        if not self._inside_building:
            laser_screen_pos = self.graphics._world_to_screen(self._laser_target)
            pygame.draw.circle(self.graphics.screen, (255, 0, 0), laser_screen_pos, 10, 2)
            pygame.draw.line(self.graphics.screen, (255, 0, 0),
                             (laser_screen_pos[0]-15, laser_screen_pos[1]),
                             (laser_screen_pos[0]+15, laser_screen_pos[1]), 2)
            pygame.draw.line(self.graphics.screen, (255, 0, 0),
                             (laser_screen_pos[0], laser_screen_pos[1]-15),
                             (laser_screen_pos[0], laser_screen_pos[1]+15), 2)

        # Drone rendering (unified single/multi)
        if self.multi_drone_mode and self.drone_manager:
            drone_list = [(i, self.drone_manager.drones[i], self.drone_manager.get_color(i))
                          for i in range(self.drone_manager.count)]
        else:
            drone_list = [(0, self.drone, (255, 80, 80))]

        font_drone = pygame.font.SysFont('Arial', 12, bold=True)
        for i, drone, color in drone_list:
            lidar = self.environment.get_lidar_scan(drone.position, drone.orientation)
            if lidar:
                self.graphics.draw_lidar_scan(drone, lidar, color=color)

            center = self.graphics._world_to_screen(drone.position[:2])
            drone_radius = int(0.4 * self.graphics.scale)
            pygame.draw.circle(self.graphics.screen, color, center, drone_radius)
            pygame.draw.circle(self.graphics.screen, (0, 0, 0), center, drone_radius, 2)

            arrow_len = drone_radius + 8
            arrow_end = (
                int(center[0] + arrow_len * math.cos(drone.orientation)),
                int(center[1] + arrow_len * math.sin(drone.orientation)))
            pygame.draw.line(self.graphics.screen, (0, 0, 0), center, arrow_end, 3)

            label = font_drone.render(f"D{i}", True, (255, 255, 255))
            self.graphics.screen.blit(label, (center[0] - 8, center[1] - 6))

            ied_range_px = int(2.0 * self.graphics.scale)
            pygame.draw.circle(self.graphics.screen, color, center, ied_range_px, 1)

            # Selection highlight (yellow ring)
            if i == self._selected_drone_id:
                sel_radius = drone_radius + 6
                pygame.draw.circle(self.graphics.screen, (255, 255, 0), center, sel_radius, 2)
                mode_text = "MANUAL" if self._manual_mode.get(i, False) else "SELECTED"
                mode_surf = font_drone.render(mode_text, True, (255, 255, 0))
                self.graphics.screen.blit(mode_surf, (center[0] + drone_radius + 8, center[1] - 6))

        # SLAM map
        if self._show_map:
            self.graphics.draw_slam_map(self.slam.get_map())

        # Breadcrumbs
        font = pygame.font.SysFont('Arial', 10)
        if self.multi_drone_mode and self.drone_manager:
            for drone_id in range(self.drone_manager.count):
                color = self.drone_manager.get_color(drone_id)
                breadcrumbs = self.drone_manager.breadcrumbs[drone_id] if drone_id < len(self.drone_manager.breadcrumbs) else []
                for idx, point in enumerate(breadcrumbs):
                    if idx % 10 != 0:
                        continue
                    pos = self.graphics._world_to_screen(point)
                    pygame.draw.circle(self.graphics.screen, color, pos, 3)
                    text = font.render(str(idx), True, color)
                    self.graphics.screen.blit(text, (pos[0]+4, pos[1]-4))
        elif len(self._breadcrumbs) > 0:
            for i, point in enumerate(self._breadcrumbs):
                if i % 10 != 0:
                    continue
                pos = self.graphics._world_to_screen(point)
                pygame.draw.circle(self.graphics.screen, (0, 80, 200), pos, 4)
                text = font.render(str(i), True, (0, 0, 180))
                self.graphics.screen.blit(text, (pos[0]+4, pos[1]-4))

        # Detections
        detected_objects = self.vision.get_recent_detections()
        if detected_objects:
            self.graphics.draw_detections(detected_objects[:50])

        # Minimap
        minimap_data = self.minimap.get_minimap_data()
        if minimap_data:
            minimap_data["breadcrumbs"] = self._breadcrumbs
            if self.multi_drone_mode and self.drone_manager:
                self._build_multi_drone_minimap_data(minimap_data)
            elif hasattr(self.search_algorithm, 'get_grid_data'):
                grid_data = self.search_algorithm.get_grid_data()
                minimap_data["searched_cells"] = grid_data.get('searched_cells', [])
                minimap_data["grid_size"] = grid_data.get('grid_size', 2.0)
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

        # Per-drone maps (always shown)
        if self.multi_drone_mode and self.drone_manager:
            self._render_drone_maps(minimap_data)
        else:
            self._render_single_drone_map(minimap_data)

        # Search debug info
        search_debug = {}
        try:
            if hasattr(self.search_algorithm, 'get_debug_info'):
                search_debug = self.search_algorithm.get_debug_info()
        except Exception:
            pass

        # Collect objects found from all drones' gossip maps (with counts)
        object_counts = {}  # type -> count
        if self.multi_drone_mode and self.drone_manager:
            seen = set()
            for i in range(self.drone_manager.count):
                for f in self.drone_manager.gossip_maps[i].features:
                    key = (f.feature_type, round(f.position[0], 1), round(f.position[1], 1))
                    if key not in seen:
                        seen.add(key)
                        object_counts[f.feature_type] = object_counts.get(f.feature_type, 0) + 1
        else:
            for feature in self.minimap.discovered_features:
                obj_type = feature.additional_data.get('detection_type', '') if feature.additional_data else ''
                if obj_type:
                    object_counts[obj_type] = object_counts.get(obj_type, 0) + 1
        search_debug['objects_found'] = [f"{t}: {n}" for t, n in sorted(object_counts.items())]
        search_debug['ieds_destroyed'] = len(self._destroyed_ieds)

        # Per-drone UI data (always built — single or multi)
        if self.multi_drone_mode and self.drone_manager:
            multi_drone_data = self._build_multi_drone_ui_data()
        else:
            multi_drone_data = self._build_single_drone_ui_data()

        simulated_elapsed = (time.time() - self._start_time) * SIM_SPEED if self._start_time else None
        self._mission_button_rects = self.graphics.draw_ui(
            self.drone, self.slam, self.comm, simulated_elapsed,
            self.current_search_name, self.search_options, search_debug, multi_drone_data,
            selected_drone=self._selected_drone_id, manual_mode=self._manual_mode) or []

        self.graphics.draw_search_debug(search_debug, self.drone.position[:2])

        if self.multi_drone_mode and self.drone_manager:
            self._render_multi_drone_overlay()

        # Joystick panel (manual mode)
        if self._joystick_panel and any(self._manual_mode.values()):
            did = self._selected_drone_id
            if did is not None and self._manual_mode.get(did, False):
                font_manual = pygame.font.Font(None, 20)
                label = font_manual.render(f"MANUAL: D{did}", True, (255, 255, 0))
                self.graphics.screen.blit(label,
                    (self._joystick_panel.x + 5, self._joystick_panel.y - 18))
            self._joystick_panel.draw(self.graphics.screen)

        self.graphics.present()

        if self._video_writer is not None:
            self._record_frame()

        for drone_id, tag in self._pending_drone_screenshots:
            self._save_drone_screenshot(drone_id, tag)
        self._pending_drone_screenshots.clear()

    def _build_multi_drone_minimap_data(self, minimap_data):
        """Build combined minimap data from all drones."""
        all_searched = []
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
                for cell in cells:
                    all_searched.append((cell, color))

        grid_size = 2.0
        all_frontiers = []
        for cell in combined_free:
            if cell[1] < 0 or cell in combined_searched or cell in combined_walls:
                continue
            wx = cell[0] * grid_size + grid_size / 2
            wy = cell[1] * grid_size + grid_size / 2
            all_frontiers.append((wx, wy))

        frontiers_by_drone = {}
        for i in range(self.drone_manager.count):
            search = self.drone_manager.get_search(i)
            drone_frontier = []
            for cell in search.free_cells:
                if cell[1] < 0 or cell in combined_searched or cell in combined_walls:
                    continue
                wx = cell[0] * grid_size + grid_size / 2
                wy = cell[1] * grid_size + grid_size / 2
                drone_frontier.append((wx, wy))
            frontiers_by_drone[i] = drone_frontier

        minimap_data["searched_cells_colored"] = all_searched
        minimap_data["searched_cells"] = [c[0] for c in all_searched]
        minimap_data["frontier_cells"] = all_frontiers
        minimap_data["frontiers_by_drone"] = frontiers_by_drone
        minimap_data["grid_size"] = 2.0
        minimap_data["drone_colors"] = {i: self.drone_manager.get_color(i)
                                         for i in range(self.drone_manager.count)}
        minimap_data["drone_positions"] = [
            (self.drone_manager.drones[i].position[:2], self.drone_manager.get_color(i))
            for i in range(self.drone_manager.count)]

        # Collect features (objects/IEDs) from all drones' gossip maps
        # for the combined Discovered Map, colored by discovering drone
        all_features_objects = []
        all_features_ieds = []
        seen_positions = set()  # deduplicate by position
        for i in range(self.drone_manager.count):
            gossip = self.drone_manager.gossip_maps[i]
            drone_color = self.drone_manager.get_color(i)
            for feature in gossip.features:
                # Deduplicate: skip if same type+position already added
                key = (feature.feature_type, round(feature.position[0], 1), round(feature.position[1], 1))
                if key in seen_positions:
                    continue
                seen_positions.add(key)
                discoverer_color = minimap_data["drone_colors"].get(feature.drone_id, drone_color)
                if feature.feature_type == 'ied':
                    all_features_ieds.append((feature.position, feature.confidence, 'IED', discoverer_color))
                else:
                    all_features_objects.append((feature.position, feature.confidence, feature.feature_type, discoverer_color))
        minimap_data["objects"] = all_features_objects
        minimap_data["ieds"] = all_features_ieds

    def _render_single_drone_map(self, minimap_data):
        """Render per-drone map for the single-drone case.

        Produces the same data format that ``draw_drone_maps`` expects."""
        if not hasattr(self.search_algorithm, 'get_grid_data'):
            return
        color = (255, 80, 80)
        search = self.search_algorithm
        drone_maps_data = [{
            'drone_id': 0,
            'free_cells': search.free_cells.copy(),
            'searched_cells': search.searched_cells.copy(),
            'wall_cells': search.wall_cells.copy(),
            'searched_by_drone': {0: search.searched_cells.copy()},
            'drone_colors': {0: color},
            'color': color,
            'position': self.drone.position[:2],
            'features': [],
            'coverage_pct': search.get_coverage_stats()['coverage_pct'],
        }]
        world_bounds = minimap_data.get("world_bounds", (-5, -5, 55, 45)) if minimap_data else (-5, -5, 55, 45)
        wall_segments = minimap_data.get("wall_segments", []) if minimap_data else []
        self.graphics.draw_drone_maps(drone_maps_data, world_bounds, wall_segments)

    def _render_drone_maps(self, minimap_data):
        """Render per-drone individual maps."""
        drone_maps_data = []
        drone_colors = {j: self.drone_manager.get_color(j) for j in range(self.drone_manager.count)}

        for i in range(self.drone_manager.count):
            search = self.drone_manager.get_search(i)
            drone = self.drone_manager.drones[i]
            gossip = self.drone_manager.gossip_maps[i]

            merged_free = search.free_cells.copy()
            merged_searched = search.searched_cells.copy()
            merged_walls = search.wall_cells.copy()
            merged_free.update(gossip.get_global_free())
            merged_searched.update(gossip.get_global_searched())
            merged_walls.update(gossip.get_global_walls())

            features = gossip.features if hasattr(gossip, 'features') else []
            searched_by_drone = gossip.get_searched_by_drone()
            if i not in searched_by_drone:
                searched_by_drone[i] = set()
            searched_by_drone[i].update(search.searched_cells)

            drone_maps_data.append({
                'drone_id': i,
                'free_cells': merged_free,
                'searched_cells': merged_searched,
                'wall_cells': merged_walls,
                'searched_by_drone': searched_by_drone,
                'drone_colors': drone_colors,
                'color': self.drone_manager.get_color(i),
                'position': drone.position[:2],
                'features': features,
                'coverage_pct': search.get_coverage_stats()['coverage_pct'],
            })

        world_bounds = minimap_data.get("world_bounds", (-5, -5, 55, 45)) if minimap_data else (-5, -5, 55, 45)
        wall_segments = minimap_data.get("wall_segments", []) if minimap_data else []
        self.graphics.draw_drone_maps(drone_maps_data, world_bounds, wall_segments)

    def _build_multi_drone_ui_data(self) -> dict:
        """Build multi-drone status data for UI panel."""
        drones_info = []
        now = time.time()
        for i in range(self.drone_manager.count):
            drone_i = self.drone_manager.drones[i]
            search_i = self.drone_manager.get_search(i)
            gossip_i = self.drone_manager.gossip_maps[i]
            start_t = self.drone_manager.mission_start_times[i]

            if drone_i.mission_complete and i in self._drone_completion_elapsed:
                elapsed_i = self._drone_completion_elapsed[i]
            else:
                elapsed_i = (now - start_t) * SIM_SPEED if start_t else 0

            sync_times = list(gossip_i.last_sync.values())
            gossip_synced = bool(sync_times and (now - max(sync_times)) < 2.0)
            in_range_of_any = any(
                d1 == i or d2 == i
                for d1, d2, _ in self.drone_manager.get_gossip_links())
            sync_ok = gossip_synced or in_range_of_any

            merged = max(0, len(gossip_i.get_global_searched()) - len(search_i.searched_cells))
            inside = self.drone_manager.inside_building[i]
            returning = self.drone_manager.returning_home[i]
            complete = drone_i.mission_complete
            status_str = "DONE" if complete else "RTN" if returning else "SRCH" if inside else "ENTRY"

            drones_info.append({
                'id': i, 'color': self.drone_manager.get_color(i),
                'battery': drone_i.battery_level, 'elapsed': elapsed_i,
                'status': status_str,
                'coverage_pct': search_i.get_coverage_stats()['coverage_pct'],
                'sync_ok': sync_ok, 'merged_cells': merged,
                'wall_hits': self._wall_collisions.get(i, 0),
                'mission': self._drone_missions.get(i, 'map'),
            })

        gossip_links = self.drone_manager.get_gossip_links()
        stats = self.drone_manager.get_global_coverage_stats()
        return {
            'drones': drones_info,
            'mesh_links': len(gossip_links),
            'global_coverage': stats['coverage_pct'],
            'comm_range': self.comm_range,
        }

    def _build_single_drone_ui_data(self) -> dict:
        """Build per-drone status data for the single-drone case.

        Returns the same dict format as ``_build_multi_drone_ui_data`` so the
        Per-Drone Status section in graphics renders identically."""
        elapsed = (time.time() - self._start_time) * SIM_SPEED if self._start_time else 0
        complete = self.drone.mission_complete
        returning = self._returning_home
        inside = self._inside_building
        status_str = "DONE" if complete else "RTN" if returning else "SRCH" if inside else "ENTRY"
        cov = 0
        if hasattr(self.search_algorithm, 'get_coverage_stats'):
            cov = self.search_algorithm.get_coverage_stats().get('coverage_pct', 0)
        drones_info = [{
            'id': 0, 'color': (255, 80, 80),
            'battery': self.drone.battery_level, 'elapsed': elapsed,
            'status': status_str, 'coverage_pct': cov,
            'sync_ok': False, 'merged_cells': 0,
            'wall_hits': self._wall_collisions.get(0, 0),
            'mission': self._drone_missions.get(0, 'map'),
        }]
        return {
            'drones': drones_info,
            'mesh_links': 0,
            'global_coverage': cov,
            'comm_range': self.comm_range,
        }

    def _render_multi_drone_overlay(self):
        """Render sync lines, comm range circles, distance labels, status panel."""
        if not self.drone_manager:
            return

        font_dist = pygame.font.SysFont('Arial', 11)
        now = time.time()

        # Sync visualization
        for sync_time, sender_id, receiver_id in self.drone_manager.get_recent_syncs():
            age = now - sync_time
            if age > 2.0:
                continue
            alpha = max(0.3, 1.0 - age / 2.0)
            sender_color = self.drone_manager.get_color(sender_id)
            line_color = (int(sender_color[0] * alpha),
                          int(sender_color[1] * alpha),
                          int(sender_color[2] * alpha))
            pos1 = self.graphics._world_to_screen(self.drone_manager.drones[sender_id].position[:2])
            pos2 = self.graphics._world_to_screen(self.drone_manager.drones[receiver_id].position[:2])
            self._draw_dashed_line(pos1, pos2, line_color, width=2)

        # Comm range circles
        for i in range(self.drone_manager.count):
            drone = self.drone_manager.drones[i]
            color = self.drone_manager.get_color(i)
            center = self.graphics._world_to_screen(drone.position[:2])
            radius = int(self.comm_range * self.graphics.scale)
            pygame.draw.circle(self.graphics.screen, color, center, radius, 1)

        # Distance labels between unlinked pairs
        gossip_links = self.drone_manager.get_gossip_links()
        if not gossip_links and self.drone_manager.count >= 2:
            for i in range(self.drone_manager.count):
                for j in range(i + 1, self.drone_manager.count):
                    pos_i = self.drone_manager.positions.get(i)
                    pos_j = self.drone_manager.positions.get(j)
                    if pos_i and pos_j:
                        dist = math.hypot(pos_j[0] - pos_i[0], pos_j[1] - pos_i[1])
                        sp1 = self.graphics._world_to_screen(self.drone_manager.drones[i].position[:2])
                        sp2 = self.graphics._world_to_screen(self.drone_manager.drones[j].position[:2])
                        mid = ((sp1[0] + sp2[0]) // 2, (sp1[1] + sp2[1]) // 2 - 10)
                        clr = (200, 0, 0) if dist > self.comm_range else (0, 200, 0)
                        dist_surf = font_dist.render(f"{dist:.0f}m (need <{self.comm_range:.0f}m)", True, clr)
                        self.graphics.screen.blit(dist_surf, mid)

        # Panel info now integrated into draw_ui processor sections

    def _draw_dashed_line(self, pos1, pos2, color, dash_len=8, gap_len=5, width=2):
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
        if not self.drone_manager:
            return

        panel_x = self.window_size[0] - 250
        panel_y = self.window_size[1] - 220
        panel_w = 240
        panel_h = 210

        panel_rect = pygame.Rect(panel_x, panel_y, panel_w, panel_h)
        pygame.draw.rect(self.graphics.screen, (240, 240, 255), panel_rect)
        pygame.draw.rect(self.graphics.screen, (0, 0, 0), panel_rect, 2)

        font = pygame.font.SysFont('Arial', 14)
        small_font = pygame.font.SysFont('Arial', 12)
        y = panel_y + 8

        title = font.render(f"Multi-Drone: {self.drone_count} drones", True, (0, 0, 0))
        self.graphics.screen.blit(title, (panel_x + 10, y))
        y += 18

        range_text = small_font.render(f"Comm: {self.comm_range:.0f}m (+/- adjust)", True, (0, 0, 0))
        self.graphics.screen.blit(range_text, (panel_x + 10, y))
        y += 16

        gossip_links = self.drone_manager.get_gossip_links()
        link_text = f"Links: {len(gossip_links)}"
        if gossip_links and len(gossip_links) == 1:
            link_text += f" ({gossip_links[0][2]:.0f}m)"
        mesh_text = font.render(link_text, True, (0, 200, 0) if gossip_links else (200, 0, 0))
        self.graphics.screen.blit(mesh_text, (panel_x + 10, y))
        y += 18

        stats = self.drone_manager.get_global_coverage_stats()
        cov_text = font.render(f"Global Coverage: {stats['coverage_pct']}%", True, (0, 0, 0))
        self.graphics.screen.blit(cov_text, (panel_x + 10, y))
        y += 22

        now = time.time()
        for i in range(self.drone_manager.count):
            drone = self.drone_manager.drones[i]
            color = self.drone_manager.get_color(i)
            inside = self.drone_manager.inside_building[i]
            returning = self.drone_manager.returning_home[i]
            complete = drone.mission_complete
            status = "COMPLETE" if complete else "RETURN" if returning else "SEARCH" if inside else "ENTRY"

            search = self.drone_manager.get_search(i)
            cov = search.get_coverage_stats()['coverage_pct']

            gossip = self.drone_manager.gossip_maps[i]
            last_sync_times = gossip.last_sync.values()
            if last_sync_times and (now - max(last_sync_times)) < 2.0:
                sync_str, sync_color = "OK", (0, 180, 0)
            elif not last_sync_times:
                sync_str, sync_color = "--", (150, 150, 150)
            else:
                sync_str, sync_color = "NO", (200, 0, 0)

            line = f"D{i}: {status} {cov}%"
            text_surf = small_font.render(line, True, color)
            self.graphics.screen.blit(text_surf, (panel_x + 10, y))
            sync_surf = small_font.render(f"sync:{sync_str}", True, sync_color)
            self.graphics.screen.blit(sync_surf, (panel_x + 160, y))
            y += 16

    # ══════════════════════════════════════════════════════════════════
    # VIDEO / SCREENSHOTS
    # ══════════════════════════════════════════════════════════════════

    def _start_video_recording(self):
        if self._video_writer is not None:
            self._finalize_video()
        self._video_frame_count = 0
        self._video_frames_written = 0
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
        if self._video_writer is None:
            return
        self._video_frame_count += 1
        if self._video_frame_count % 3 != 0:
            return
        surface = self.graphics.screen
        frame = pygame.surfarray.array3d(surface)
        frame = np.transpose(frame, (1, 0, 2))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self._video_writer.write(frame)
        self._video_frames_written += 1

    def _finalize_video(self):
        if self._video_writer is None:
            return
        frames = getattr(self, '_video_frames_written', 0)
        try:
            self._video_writer.release()
        except Exception as e:
            print(f"WARNING: Error releasing video writer: {e}")
        self._video_writer = None

        if frames < 20:
            try:
                if os.path.exists(self._video_filename):
                    os.remove(self._video_filename)
                    print(f"Discarded tiny video ({frames} frames)")
            except Exception:
                pass
            self._video_frame_count = 0
            self._video_frames_written = 0
            self._video_filename = ""
            return

        cov = 0
        try:
            if self.multi_drone_mode and self.drone_manager:
                cov = self.drone_manager.get_global_coverage_stats()['coverage_pct']
            elif hasattr(self.search_algorithm, 'get_coverage_stats'):
                cov = int(self.search_algorithm.get_coverage_stats().get('coverage_pct', 0))
        except Exception:
            pass

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
        self._video_frames_written = 0
        self._video_filename = ""

    def _save_screenshot(self, tag: str = ""):
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

    def _save_drone_screenshot(self, drone_id: int, tag: str = "done"):
        if self.multi_drone_mode and self.drone_manager:
            search = self.drone_manager.get_search(drone_id)
            start_t = self.drone_manager.mission_start_times[drone_id]
        else:
            search = self.search_algorithm
            start_t = self._start_time
        cov = search.get_coverage_stats()['coverage_pct']
        elapsed = int((time.time() - start_t) * SIM_SPEED) if start_t else 0
        mm = elapsed // 60
        ss = elapsed % 60
        filename = f"d{drone_id}_{tag}_{mm}m{ss:02d}s_{cov}pct.png"
        try:
            pygame.image.save(self.graphics.screen, filename)
            print(f"Saved: {filename}")
        except Exception as e:
            print(f"Error saving drone screenshot: {e}")

    # ══════════════════════════════════════════════════════════════════
    # RESET
    # ══════════════════════════════════════════════════════════════════

    def _reset_simulation(self):
        self._finalize_video()
        print("Resetting simulation...")
        self.drone.reset()
        self.slam.reset()
        self.vision.reset()
        self.comm.reset()
        self.minimap.reset()
        self.search_algorithm.reset()
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
        self._drone_mid_screenshots_taken.clear()
        self._drone_completion_elapsed.clear()
        self._pending_drone_screenshots.clear()
        self._coverage_milestones_logged.clear()
        self._selected_drone_id = None
        self._manual_mode.clear()
        self._joystick_panel = None
        self._wall_collisions.clear()
        self._wall_collision_cooldown.clear()
        self._drone_missions.clear()
        self._destroyed_ieds.clear()
        self._drone_destroying.clear()
        self._mission_button_rects.clear()

        if self.multi_drone_mode and self.drone_manager:
            self.drone_manager.reset(entry_point=(self._door_x, -3.0))

    def _reset_mission_keep_map(self):
        print("Starting new mission with existing map...")
        self.drone.reset()
        self.drone.position = np.array([self._door_x, -3, 3], dtype=np.float32)
        self.slam.current_exploration_index = 0
        self._returning_home = False
        self._breadcrumbs = []
        self._entry_index = 0
        self._inside_building = False
        self.vision.reset()
        self.search_algorithm.reset()
        self.drone.mission_complete = False
        self._time_limit_reached = False
        self._drone_mid_screenshots_taken.clear()
        self._return_msg_printed = False

    def _cleanup(self):
        self._finalize_video()
        pygame.quit()
        print("Simulation ended.")


# ══════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════════

def main():
    """Main entry point — 3-processor architecture simulation."""
    try:
        try:
            if not pygame.get_init():
                pygame.init()
            if not pygame.display.get_init():
                pygame.display.init()
        except Exception:
            pass
        simulation = DroneSimulationGUI()
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
