"""
Graphics Engine for Drone Simulation
Provides real-time visualization of drone navigation, mapping, and object detection
"""

import pygame
import numpy as np
import math
import time
from typing import Tuple, List, Dict, Optional
from core.vision import Detection, ObjectType

class Colors:
    """Color constants for visualization."""
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 0)
    ORANGE = (255, 165, 0)
    PURPLE = (128, 0, 128)
    CYAN = (0, 255, 255)
    GRAY = (128, 128, 128)
    LIGHT_GRAY = (200, 200, 200)
    DARK_GRAY = (64, 64, 64)
    
    # Object-specific colors
    PERSON = (255, 100, 100)
    EQUIPMENT = (100, 100, 255)
    HAZARD = (255, 50, 50)
    FURNITURE = (139, 69, 19)
    
    # Map colors
    FREE_SPACE = (240, 240, 240)
    OCCUPIED_SPACE = (50, 50, 50)
    UNKNOWN_SPACE = (180, 180, 180)
    EXPLORED_PATH = (100, 255, 100)

class GraphicsEngine:
    """
    Main graphics engine for rendering the drone simulation.
    Handles all visual elements including drone, environment, map, and UI.
    """
    
    def __init__(self, window_size: Tuple[int, int]):
        """Initialize graphics engine."""
        # pygame.init() should be called in main() before creating GraphicsEngine

        self.window_size = window_size
        try:
            self.screen = pygame.display.set_mode(window_size)
            pygame.display.set_caption("Deep Learning Co Drone Navigation")
            print(f"✅ Graphics engine initialized: {window_size[0]}x{window_size[1]}")
        except Exception as e:
            print(f"❌ Failed to create display: {e}")
            raise
        
        # Fonts
        self.font_small = pygame.font.Font(None, 16)
        self.font_medium = pygame.font.Font(None, 24)
        self.font_large = pygame.font.Font(None, 32)
        
        # View parameters
        self.scale = 10.0  # pixels per meter
        self.offset_x = 50
        self.offset_y = 50
        
        # View modes
        # Keep LIDAR rays visible by default
        self.show_lidar = True
        self.show_grid_map = True
        # CHANGED: hide old green waypoint rays by default (can toggle via toggle_view_mode)
        self.show_path = False
        self.show_detections = True
        
        # Animation
        self.frame_count = 0
        self.detection_animations = []  # For pulsing detection indicators
        self.overlay_color = Colors.PURPLE
        # Radio TX overlay state
        self._radio_last_ts: float = 0.0
        self._radio_flash_secs: float = 2.0
    
    def clear(self):
        """Clear the screen."""
        self.screen.fill(Colors.WHITE)
        self.frame_count += 1
    
    def present(self):
        """Present the rendered frame."""
        pygame.display.flip()
    
    def draw_environment(self, environment):
        """Draw the building environment."""
        # Draw walls
        for wall in environment.walls:
            if wall and len(wall) >= 2 and wall[0] and wall[1]:
                start_pos = self._world_to_screen(wall[0])
                end_pos = self._world_to_screen(wall[1])
                # Ensure positions are valid before drawing
                if start_pos and end_pos:
                    pygame.draw.line(self.screen, Colors.BLACK, start_pos, end_pos, 3)
        
        # Draw obstacles with labels
        obstacle_labels = ["table", "chair", "furniture", "equipment", "furniture", "obstacle"]
        for i, (obs_x, obs_y, radius) in enumerate(environment.obstacles):
            center = self._world_to_screen((obs_x, obs_y))
            screen_radius = int(radius * self.scale)
            pygame.draw.circle(self.screen, Colors.DARK_GRAY, center, screen_radius)
            pygame.draw.circle(self.screen, Colors.BLACK, center, screen_radius, 2)
            
            # Draw obstacle label
            label = obstacle_labels[i] if i < len(obstacle_labels) else "obstacle"
            text_surface = self.font_small.render(label, True, Colors.BLACK)
            label_pos = (center[0] + screen_radius + 3, center[1] - 8)
            self.screen.blit(text_surface, label_pos)
        
        # Draw objects of interest
        for obj_x, obj_y, obj_type, confidence in environment.objects_of_interest:
            center = self._world_to_screen((obj_x, obj_y))
            color = self._get_object_color(obj_type)
            
            # Draw object with fixed size (no pre-detection probabilities)
            size = 8
            pygame.draw.circle(self.screen, color, center, size)
            pygame.draw.circle(self.screen, Colors.BLACK, center, size, 1)
            
            # Draw object label - type only, no percentages until detected
            label = f"{obj_type}"
            text_surface = self.font_small.render(label, True, Colors.BLACK)
            label_pos = (center[0] + size + 5, center[1] - 8)
            self.screen.blit(text_surface, label_pos)
    
    def draw_drone(self, drone):
        """Draw the drone with orientation and status."""
        # Safety check for valid drone position
        if drone.position is None or len(drone.position) < 2:
            return
            
        center = self._world_to_screen(drone.position[:2])
        
        # Draw drone body (circle)
        drone_radius = int(0.3 * self.scale)  # 30cm radius
        if drone_radius > 0:
            pygame.draw.circle(self.screen, Colors.BLUE, center, drone_radius)
            pygame.draw.circle(self.screen, Colors.BLACK, center, drone_radius, 2)
        
        # Draw orientation arrow
        if math.isfinite(drone.orientation):
            arrow_length = drone_radius + 10
            arrow_end = (
                int(center[0] + arrow_length * math.cos(drone.orientation)),
                int(center[1] + arrow_length * math.sin(drone.orientation))
            )
            pygame.draw.line(self.screen, Colors.RED, center, arrow_end, 3)
        
        # Removed green waypoint trail to reduce visual noise. Orientation arrow already shows heading.
        # (Intentionally no residual trail rendering here.)
        
        # Removed velocity vector arrow to reduce clutter per user request
    
    def draw_slam_map(self, grid_map):
        """Draw the SLAM-generated map."""
        if not self.show_grid_map:
            return
        
        cell_size = max(1, int(grid_map.resolution * self.scale))
        
        # Throttle drawing to avoid excessive per-frame rendering
        total_cells = grid_map.grid_height * grid_map.grid_width
        max_draw_per_frame = 20000  # cap drawn cells each frame
        stride = max(1, total_cells // max_draw_per_frame)
        
        idx = 0
        for y in range(grid_map.grid_height):
            for x in range(grid_map.grid_width):
                # Sample cells to reduce draw load
                if (idx % stride) != 0:
                    idx += 1
                    continue
                idx += 1
                occupancy = grid_map.grid[y, x]
                
                # Skip unknown cells for performance
                if occupancy == 50:
                    continue
                
                # Convert grid to world coordinates
                world_x, world_y = grid_map.grid_to_world(x, y)
                screen_pos = self._world_to_screen((world_x, world_y))
                
                # Choose color based on occupancy
                if occupancy > 70:
                    color = Colors.OCCUPIED_SPACE
                elif occupancy < 30:
                    color = Colors.FREE_SPACE
                else:
                    # Blend colors for uncertain areas
                    blend_factor = (occupancy - 30) / 40
                    color = self._blend_colors(Colors.FREE_SPACE, Colors.OCCUPIED_SPACE, blend_factor)
                
                # Draw cell
                rect = pygame.Rect(int(screen_pos[0] - cell_size//2), int(screen_pos[1] - cell_size//2), 
                                 int(cell_size), int(cell_size))
                pygame.draw.rect(self.screen, color, rect)
    
    def draw_detections(self, detections: List[Detection]):
        """Draw recent object detections."""
        if not self.show_detections:
            return
        
        # User request: NO radar-range style overlays. Draw a tiny dot only.
        for detection in detections:
            pos = self._world_to_screen(detection.position)
            color = self._get_object_color(detection.object_type.value)
            pygame.draw.circle(self.screen, color, (int(pos[0]), int(pos[1])), 3)
    
    def draw_lidar_scan(self, drone, lidar_ranges, color=None):
        """Draw LIDAR scan visualization — STM 54×42 limited FoV cone.

        Args:
            drone: Drone object with position and orientation
            lidar_ranges: dict (new STM format) or list (legacy)
            color: Optional RGB tuple for ray color (default: RED)
        """
        if not self.show_lidar or not lidar_ranges:
            return

        ray_color = color if color else Colors.RED

        # Unpack new dict format or fall back to legacy list
        if isinstance(lidar_ranges, dict):
            ranges = lidar_ranges["ranges"]
            start_angle = lidar_ranges["start_angle"]
            angle_step = lidar_ranges["angle_step"]
            hfov = lidar_ranges["hfov"]
            max_range = 9.0
        else:
            ranges = lidar_ranges
            start_angle = drone.orientation
            angle_step = 2 * math.pi / len(ranges) if len(ranges) > 0 else 0
            hfov = 2 * math.pi
            max_range = 25.0

        drone_pos = self._world_to_screen(drone.position[:2])

        # Draw FoV boundary lines (faint)
        fov_color = (ray_color[0] // 3, ray_color[1] // 3, ray_color[2] // 3)
        for boundary_angle in [start_angle, start_angle + hfov]:
            bx = drone.position[0] + max_range * math.cos(boundary_angle)
            by = drone.position[1] + max_range * math.sin(boundary_angle)
            bp = self._world_to_screen((bx, by))
            if drone_pos and bp:
                pygame.draw.line(self.screen, fov_color, drone_pos, bp, 1)

        # Draw each ray
        for i, distance in enumerate(ranges):
            # Skip max-range returns
            if distance >= max_range - 0.1:
                continue

            angle = start_angle + i * angle_step
            end_x = drone.position[0] + distance * math.cos(angle)
            end_y = drone.position[1] + distance * math.sin(angle)
            end_pos = self._world_to_screen((end_x, end_y))

            # Draw LIDAR ray
            if drone_pos and end_pos:
                pygame.draw.line(self.screen, ray_color, drone_pos, end_pos, 1)

            # Draw hit point (wall hit)
            if end_pos:
                pygame.draw.circle(self.screen, ray_color, end_pos, 2)
    
    def draw_ui(self, drone, slam_system, comm_system, mission_start_time=None, search_method: Optional[str] = None, search_options: Optional[List[str]] = None, search_debug: Optional[Dict] = None, multi_drone_data: Optional[Dict] = None):
        """Draw user interface elements."""
        # Draw background panel (narrower: 250px instead of 300px)
        ui_rect = pygame.Rect(self.window_size[0] - 250, 0, 250, self.window_size[1])
        pygame.draw.rect(self.screen, Colors.LIGHT_GRAY, ui_rect)
        pygame.draw.rect(self.screen, Colors.BLACK, ui_rect, 2)
        
        y_offset = 20
        line_height = 25
        
        # Title
        title = self.font_large.render("Drone Status", True, Colors.BLACK)
        self.screen.blit(title, (ui_rect.x + 10, y_offset))
        y_offset += 40
        
        # Drone status
        status = drone.get_status()
        
        # Position
        pos_text = f"Position: ({status['position'][0]:.1f}, {status['position'][1]:.1f}, {status['position'][2]:.1f})"
        pos_surface = self.font_small.render(pos_text, True, Colors.BLACK)
        self.screen.blit(pos_surface, (ui_rect.x + 10, y_offset))
        y_offset += line_height
        
        # Direction (heading) and Speed on same row
        heading_deg = math.degrees(drone.orientation) % 360
        speed_mps = math.sqrt(drone.velocity[0]**2 + drone.velocity[1]**2)
        dir_text = f"Heading: {heading_deg:.0f}°"
        speed_text = f"Speed: {speed_mps:.1f} m/s"
        dir_surface = self.font_small.render(dir_text, True, Colors.BLACK)
        speed_surface = self.font_small.render(speed_text, True, Colors.BLACK)
        self.screen.blit(dir_surface, (ui_rect.x + 10, y_offset))
        self.screen.blit(speed_surface, (ui_rect.x + 120, y_offset))
        y_offset += line_height
        
        # Battery and Timer — per-drone grid or single drone
        if multi_drone_data and multi_drone_data.get('drones'):
            drones_info = multi_drone_data['drones']
            col_w = 57
            row_h = 26
            grid_x = ui_rect.x + 5

            for idx, di in enumerate(drones_info[:12]):
                col = idx % 4
                row = idx // 4
                cx = grid_x + col * col_w
                cy = y_offset + row * row_h

                color = di.get('color', Colors.GRAY)
                elapsed_s = di.get('elapsed', 0)
                battery = di.get('battery', 100)
                did = di.get('id', idx)

                # Timer: "D0 1:45"
                mins = int(elapsed_s // 60)
                secs = int(elapsed_s % 60)
                timer_str = f"D{did} {mins}:{secs:02d}"
                timer_surf = self.font_small.render(timer_str, True, color)
                self.screen.blit(timer_surf, (cx, cy))

                # Battery bar (thin, color-coded)
                bar_y = cy + 14
                bar_w = col_w - 6
                bar_h = 4
                pygame.draw.rect(self.screen, (200, 200, 200), (cx, bar_y, bar_w, bar_h))
                fill_w = int(bar_w * battery / 100)
                bar_color = (0, 200, 0) if battery > 50 else (255, 165, 0) if battery > 20 else (255, 0, 0)
                pygame.draw.rect(self.screen, bar_color, (cx, bar_y, fill_w, bar_h))

            num_rows = min(3, (len(drones_info) + 3) // 4)
            y_offset += num_rows * row_h + 4

            # Mesh links and global coverage
            mesh_links = multi_drone_data.get('mesh_links', 0)
            global_cov = multi_drone_data.get('global_coverage', 0)
            comm_range = multi_drone_data.get('comm_range', 0)
            mesh_color = (0, 180, 0) if mesh_links > 0 else (200, 0, 0)
            mesh_surf = self.font_small.render(f"Mesh: {mesh_links} links  Cov: {global_cov}%", True, mesh_color)
            self.screen.blit(mesh_surf, (ui_rect.x + 10, y_offset))
            y_offset += 18

            # Per-drone status line: "D0:SRCH 45% OK" compact
            for di in drones_info[:12]:
                did = di.get('id', 0)
                color = di.get('color', Colors.GRAY)
                st = di.get('status', '?')
                cov = di.get('coverage_pct', 0)
                sync_ok = di.get('sync_ok', False)
                merged = di.get('merged_cells', 0)
                sync_str = "OK" if sync_ok else "--"
                sync_color = (0, 180, 0) if sync_ok else (150, 150, 150)
                line = f"D{did}:{st} {cov}%"
                self.screen.blit(self.font_small.render(line, True, color), (ui_rect.x + 10, y_offset))
                # Sync + merge count on right
                info_str = f"+{merged} {sync_str}"
                self.screen.blit(self.font_small.render(info_str, True, sync_color), (ui_rect.x + 130, y_offset))
                y_offset += 15

            y_offset += 5
        else:
            battery_text = f"Battery: {status['battery']:.1f}%"
            battery_color = Colors.GREEN if status['battery'] > 50 else Colors.ORANGE if status['battery'] > 20 else Colors.RED
            battery_surface = self.font_small.render(battery_text, True, battery_color)
            self.screen.blit(battery_surface, (ui_rect.x + 10, y_offset))

            if mission_start_time is not None:
                elapsed = mission_start_time
                minutes = int(elapsed // 60)
                seconds = int(elapsed % 60)
                timer_text = f"Time: {minutes:02d}:{seconds:02d}"
                timer_surface = self.font_small.render(timer_text, True, Colors.GREEN)
                self.screen.blit(timer_surface, (ui_rect.x + 150, y_offset))

            y_offset += line_height
        
        # Mission status - use search debug info if available
        if search_debug:
            mode = search_debug.get('mode', 'SEARCH')
            cov = search_debug.get('coverage_pct', 0)
            uncov = search_debug.get('uncovered', 0)
            dist_home = search_debug.get('dist_to_entry', 0)
            
            # Mode with color coding
            if mode == "RETURN":
                mode_color = Colors.BLUE
            elif mode == "RUSH":
                mode_color = Colors.ORANGE
            else:
                mode_color = Colors.BLACK
            
            mode_surface = self.font_small.render(f"Mode: {mode}", True, mode_color)
            self.screen.blit(mode_surface, (ui_rect.x + 10, y_offset))
            y_offset += line_height
            
            # Coverage and uncovered
            cov_text = f"Coverage: {cov:.0f}%  ({uncov} cells left)"
            cov_surface = self.font_small.render(cov_text, True, Colors.BLACK)
            self.screen.blit(cov_surface, (ui_rect.x + 10, y_offset))
            y_offset += line_height
            
            # Distance to entry (important for return)
            dist_text = f"Dist to exit: {dist_home}m"
            dist_surface = self.font_small.render(dist_text, True, Colors.BLACK)
            self.screen.blit(dist_surface, (ui_rect.x + 10, y_offset))
            y_offset += line_height

            # Mapping stats — concise overview of exploration progress
            free_n = search_debug.get('free_cells', 0)
            wall_n = search_debug.get('wall_cells', 0)
            searched_n = search_debug.get('searched_cells', 0)
            failed_n = search_debug.get('failed_targets', 0)
            map_text = f"Map: {free_n} free, {wall_n} wall, {searched_n} searched"
            map_surface = self.font_small.render(map_text, True, Colors.DARK_GRAY)
            self.screen.blit(map_surface, (ui_rect.x + 10, y_offset))
            y_offset += 18
            if failed_n > 0:
                fail_text = f"Failed targets: {failed_n}"
                fail_surface = self.font_small.render(fail_text, True, Colors.ORANGE)
                self.screen.blit(fail_surface, (ui_rect.x + 10, y_offset))
                y_offset += 18

            # Sensor spec reminder
            lidar_text = "LiDAR: 59°/9m | IED: 2m | Grid: 2m"
            lidar_surface = self.font_small.render(lidar_text, True, Colors.DARK_GRAY)
            self.screen.blit(lidar_surface, (ui_rect.x + 10, y_offset))
            y_offset += line_height
        else:
            mission_text = f"Mission: {'Complete' if status['mission_complete'] else 'Active'}"
            mission_surface = self.font_small.render(mission_text, True, Colors.BLACK)
            self.screen.blit(mission_surface, (ui_rect.x + 10, y_offset))
            y_offset += line_height

        y_offset += 10  # Small gap
        
        # Search method
        if search_method:
            algo_surface = self.font_small.render(f"Search: {search_method}", True, Colors.BLACK)
            self.screen.blit(algo_surface, (ui_rect.x + 10, y_offset))
            y_offset += line_height + 10
        
        # Objects found section
        objects_found = search_debug.get('objects_found', []) if search_debug else []
        found_title = self.font_medium.render("Objects Found", True, Colors.BLACK)
        self.screen.blit(found_title, (ui_rect.x + 10, y_offset))
        y_offset += 25
        
        if objects_found:
            for obj_type in objects_found[:8]:  # Limit to 8 items
                obj_surface = self.font_small.render(f"- {obj_type}", True, Colors.DARK_GRAY)
                self.screen.blit(obj_surface, (ui_rect.x + 10, y_offset))
                y_offset += 18
            if len(objects_found) > 8:
                more = self.font_small.render(f"  +{len(objects_found)-8} more", True, Colors.DARK_GRAY)
                self.screen.blit(more, (ui_rect.x + 10, y_offset))
                y_offset += 18
        else:
            none_surface = self.font_small.render("(none yet)", True, Colors.DARK_GRAY)
            self.screen.blit(none_surface, (ui_rect.x + 10, y_offset))
            y_offset += 18
        
        y_offset += 20
        
        # Controls
        controls_title = self.font_medium.render("Controls", True, Colors.BLACK)
        self.screen.blit(controls_title, (ui_rect.x + 10, y_offset))
        y_offset += 25
        
        controls = [
            "SPACE: Start/Stop Mission",
            "R: Reset Simulation",
            "D: Add/Cycle Drones",
            "N: New Object Placement",
            "B: New Building Layout",
            "+/-: Comm Range",
            "P: Save Screenshot",
            "ESC: Exit"
        ]
        
        for control in controls:
            control_surface = self.font_small.render(control, True, Colors.BLACK)
            self.screen.blit(control_surface, (ui_rect.x + 10, y_offset))
            y_offset += line_height

        # Optional: numbered search methods for quick switching
        if search_options:
            y_offset += 10
            self.screen.blit(self.font_medium.render("Search Methods", True, Colors.BLACK), (ui_rect.x + 10, y_offset))
            y_offset += 28
            for idx, name in enumerate(search_options, start=1):
                line = f"{idx}: {name}"
                is_active = (search_method == name)
                color = Colors.GREEN if is_active else Colors.BLACK
                self.screen.blit(self.font_small.render(line, True, color), (ui_rect.x + 10, y_offset))
                y_offset += line_height

    def _draw_map_panel(self, panel_x, panel_y, panel_width, panel_height,
                        title, border_color, world_bounds, grid_size,
                        searched_cells=None, frontier_cells=None,
                        wall_segments=None, drone_positions=None,
                        features=None, breadcrumbs=None, doors=None):
        """Unified map panel renderer used by both draw_minimap and draw_drone_maps.

        Args:
            panel_x, panel_y: top-left corner of the panel
            panel_width, panel_height: size of the panel
            title: title string drawn above the panel
            border_color: color for the panel border
            world_bounds: (min_x, min_y, max_x, max_y) world coordinate range
            grid_size: grid cell size in world units
            searched_cells: list of ((wx,wy), fill_color, border_color) tuples
            frontier_cells: dict of drone_id -> list of (wx,wy) world positions
            wall_segments: list of ((x1,y1), (x2,y2)) line segments
            drone_positions: list of ((x,y), color) tuples
            features: list of dicts with 'type', 'position' (world), 'label'
            breadcrumbs: list of (x,y) world positions
            doors: list of (x,y) world positions
        """
        # Background and border
        rect = pygame.Rect(panel_x, panel_y, panel_width, panel_height)
        pygame.draw.rect(self.screen, Colors.WHITE, rect)
        pygame.draw.rect(self.screen, border_color, rect, 2)

        # Title above panel
        title_surf = self.font_small.render(title, True, border_color)
        self.screen.blit(title_surf, (panel_x + 2, panel_y - 14))

        # Coordinate transform setup
        min_x, min_y, max_x, max_y = world_bounds
        pad = 15

        def world_to_panel(wx, wy):
            x_ratio = (wx - min_x) / (max_x - min_x) if max_x != min_x else 0.5
            y_ratio = (wy - min_y) / (max_y - min_y) if max_y != min_y else 0.5
            px = panel_x + pad + int(x_ratio * (panel_width - 2 * pad))
            py = panel_y + pad + int(y_ratio * (panel_height - 2 * pad))
            return (px, py)

        def in_bounds(px, py):
            return (panel_x + pad < px < panel_x + panel_width - pad and
                    panel_y + pad < py < panel_y + panel_height - pad)

        # Cell pixel size
        scale_factor = min((panel_width - 2*pad) / (max_x - min_x),
                           (panel_height - 2*pad) / (max_y - min_y)) if max_x > min_x and max_y > min_y else 1
        cell_px = max(4, int(grid_size * scale_factor))

        # 1. Searched cells (colored rects with border)
        if searched_cells:
            cs = max(2, cell_px // 2)
            for cell_entry in searched_cells:
                wx, wy = cell_entry[0]
                fill = cell_entry[1]
                bdr = cell_entry[2]
                tl = world_to_panel(wx - grid_size / 2, wy - grid_size / 2)
                br = world_to_panel(wx + grid_size / 2, wy + grid_size / 2)
                w = max(br[0] - tl[0], 4)
                h = max(br[1] - tl[1], 4)
                r = pygame.Rect(tl[0], tl[1], w, h)
                pygame.draw.rect(self.screen, fill, r)
                pygame.draw.rect(self.screen, bdr, r, 1)

        # 2. Wall segments (black lines)
        if wall_segments:
            for ws, we in wall_segments:
                sp = world_to_panel(ws[0], ws[1])
                ep = world_to_panel(we[0], we[1])
                if (panel_x <= sp[0] <= panel_x + panel_width or
                        panel_x <= ep[0] <= panel_x + panel_width):
                    pygame.draw.line(self.screen, Colors.BLACK, sp, ep, 2)

        # 3. Frontier cells (yellow, small shapes — 1/3 original size)
        if frontier_cells:
            for drone_id, cells in frontier_cells.items():
                for wx, wy in cells:
                    pos = world_to_panel(wx, wy)
                    if not in_bounds(pos[0], pos[1]):
                        continue
                    x, y = pos
                    if drone_id == 0:
                        pygame.draw.circle(self.screen, (255, 200, 0), pos, 2)
                    elif drone_id == 1:
                        pts = [(x, y - 2), (x - 1, y + 1), (x + 1, y + 1)]
                        pygame.draw.polygon(self.screen, (255, 200, 0), pts)
                    elif drone_id == 2:
                        pygame.draw.rect(self.screen, (255, 200, 0), (x - 1, y - 1, 2, 2))
                    else:
                        pts = [(x, y - 2), (x + 2, y), (x, y + 2), (x - 2, y)]
                        pygame.draw.polygon(self.screen, (255, 200, 0), pts)

        # 4. Doors (green squares)
        if doors:
            for dx, dy in doors:
                dp = world_to_panel(dx, dy)
                pygame.draw.rect(self.screen, Colors.GREEN, (dp[0] - 2, dp[1] - 2, 4, 4))

        # 5. Features (IEDs red + label, objects blue + label)
        if features:
            for f in features:
                fpos = world_to_panel(f['position'][0], f['position'][1])
                if not (panel_x < fpos[0] < panel_x + panel_width and
                        panel_y < fpos[1] < panel_y + panel_height):
                    continue
                if f['type'] == 'ied':
                    pygame.draw.circle(self.screen, Colors.RED, fpos, 4)
                    pygame.draw.circle(self.screen, Colors.WHITE, fpos, 4, 1)
                    lbl = self.font_small.render("IED!", True, Colors.RED)
                    self.screen.blit(lbl, (fpos[0] + 5, fpos[1] - 6))
                else:
                    pygame.draw.circle(self.screen, Colors.BLUE, fpos, 3)
                    label = f.get('label', '')
                    if label:
                        lbl = self.font_small.render(label[:6], True, Colors.BLUE)
                        self.screen.blit(lbl, (fpos[0] + 4, fpos[1] - 6))

        # 6. Breadcrumbs (light green dots)
        if breadcrumbs:
            for i in range(0, len(breadcrumbs), 3):
                if i < len(breadcrumbs):
                    cp = world_to_panel(breadcrumbs[i][0], breadcrumbs[i][1])
                    pygame.draw.circle(self.screen, (150, 255, 150), cp, 1)

        # 7. Drone positions (colored circle with white border)
        if drone_positions:
            for pos, clr in drone_positions:
                dp = world_to_panel(pos[0], pos[1])
                pygame.draw.circle(self.screen, clr, dp, 5)
                pygame.draw.circle(self.screen, Colors.WHITE, dp, 5, 1)

    def draw_minimap(self, minimap_data: Dict, drone_pos: Tuple[float, float]):
        """Draw the Discovered Map using the unified _draw_map_panel."""
        minimap_width = 350
        minimap_height = 280
        minimap_x = 50
        minimap_y = 580

        if not minimap_data:
            # Draw empty panel with "Learning..." text
            rect = pygame.Rect(minimap_x, minimap_y, minimap_width, minimap_height)
            pygame.draw.rect(self.screen, Colors.WHITE, rect)
            pygame.draw.rect(self.screen, Colors.BLACK, rect, 2)
            title_surface = self.font_small.render("Discovered Map", True, Colors.BLACK)
            self.screen.blit(title_surface, (minimap_x, minimap_y - 15))
            learning_surface = self.font_small.render("Learning...", True, Colors.DARK_GRAY)
            text_rect = learning_surface.get_rect(center=(minimap_x + minimap_width // 2, minimap_y + minimap_height // 2))
            self.screen.blit(learning_surface, text_rect)
            return

        # Build world bounds with margin
        wb = minimap_data.get("world_bounds", (-5, -5, 55, 45))
        margin = 5
        world_bounds = (wb[0] - margin, wb[1] - margin, wb[2] + margin, wb[3] + margin)

        grid_size = minimap_data.get("grid_size", 2.0)

        # --- Convert searched cells to unified format ---
        searched_cells = []
        searched_cells_colored = minimap_data.get("searched_cells_colored", [])
        if searched_cells_colored:
            for cell_data in searched_cells_colored:
                if isinstance(cell_data, tuple) and len(cell_data) == 2:
                    (wx, wy), color = cell_data
                    fill = (min(255, color[0] + 100), min(255, color[1] + 100), min(255, color[2] + 100))
                    searched_cells.append(((wx, wy), fill, color))
                else:
                    wx, wy = cell_data
                    searched_cells.append(((wx, wy), (200, 255, 200), (100, 200, 100)))
        else:
            for wx, wy in minimap_data.get("searched_cells", []):
                searched_cells.append(((wx, wy), (200, 255, 200), (100, 200, 100)))

        # --- Convert frontiers to unified format ---
        frontiers_by_drone = minimap_data.get("frontiers_by_drone", {})
        if not frontiers_by_drone:
            frontier_list = minimap_data.get("frontier_cells", [])
            if frontier_list:
                frontiers_by_drone = {0: frontier_list}

        # --- Features ---
        features = []
        for obj_data in minimap_data.get("objects", []):
            if isinstance(obj_data, tuple) and len(obj_data) >= 2:
                label = obj_data[2] if len(obj_data) > 2 else ""
                features.append({'type': 'object', 'position': obj_data[0], 'label': label})
        for ied_data in minimap_data.get("ieds", []):
            if isinstance(ied_data, tuple) and len(ied_data) >= 2:
                features.append({'type': 'ied', 'position': ied_data[0], 'label': 'IED!'})

        # --- Drone positions ---
        dp_list = minimap_data.get("drone_positions", None)
        if dp_list:
            drone_positions = dp_list
        else:
            drone_positions = [(drone_pos, Colors.RED)]

        self._draw_map_panel(
            minimap_x, minimap_y, minimap_width, minimap_height,
            "Discovered Map", Colors.BLACK, world_bounds, grid_size,
            searched_cells=searched_cells,
            frontier_cells=frontiers_by_drone,
            wall_segments=minimap_data.get("wall_segments", []),
            drone_positions=drone_positions,
            features=features,
            breadcrumbs=minimap_data.get("breadcrumbs", []),
            doors=minimap_data.get("doors", []),
        )

    def draw_drone_maps(self, drone_maps_data: List[dict], world_bounds: Tuple[float, float, float, float], wall_segments: List = None):
        """Draw individual maps for each drone using the unified _draw_map_panel."""
        if not drone_maps_data:
            return

        if wall_segments is None:
            wall_segments = []

        map_width = 350
        map_height = 280
        start_x = self.window_size[0] - 250 - map_width - 10
        start_y = 35
        map_spacing = 35

        # Add margin to world bounds (same as minimap)
        margin = 5
        wb = (world_bounds[0] - margin, world_bounds[1] - margin,
              world_bounds[2] + margin, world_bounds[3] + margin)
        min_x, min_y, max_x, max_y = wb

        grid_size = 2.0

        # Limit to 3 maps
        display_count = min(len(drone_maps_data), 3)

        for idx in range(display_count):
            drone_data = drone_maps_data[idx]
            map_x = start_x
            map_y = start_y + idx * (map_height + map_spacing)

            drone_id = drone_data.get('drone_id', idx)
            color = drone_data.get('color', (200, 200, 200))
            light_color = (min(255, color[0] + 100), min(255, color[1] + 100), min(255, color[2] + 100))

            # --- Convert searched cells to unified format ---
            searched_cells = []
            searched_by_drone = drone_data.get('searched_by_drone', {})
            drone_colors = drone_data.get('drone_colors', {})

            if searched_by_drone:
                all_searched_cells = set()
                for cells in searched_by_drone.values():
                    all_searched_cells.update(cells)
                my_cells = searched_by_drone.get(drone_id, set())

                for cell in all_searched_cells:
                    # World position of cell center
                    wx = cell[0] * grid_size + grid_size / 2
                    wy = cell[1] * grid_size + grid_size / 2
                    if cell in my_cells:
                        searched_cells.append(((wx, wy), light_color, color))
                    else:
                        cell_fill = light_color
                        cell_bdr = color
                        for sid, scells in searched_by_drone.items():
                            if cell in scells and sid in drone_colors:
                                sc = drone_colors[sid]
                                cell_fill = (min(255, sc[0] + 100), min(255, sc[1] + 100), min(255, sc[2] + 100))
                                cell_bdr = sc
                                break
                        searched_cells.append(((wx, wy), cell_fill, cell_bdr))
            else:
                for cell in drone_data.get('searched_cells', set()):
                    wx = cell[0] * grid_size + grid_size / 2
                    wy = cell[1] * grid_size + grid_size / 2
                    searched_cells.append(((wx, wy), light_color, color))

            # --- Convert frontiers to unified format ---
            walls = drone_data.get('wall_cells', set())
            searched = drone_data.get('searched_cells', set())
            free = drone_data.get('free_cells', set())
            frontier = free - searched - walls
            frontier_world = []
            for cell in frontier:
                wx = cell[0] * grid_size + grid_size / 2
                wy = cell[1] * grid_size + grid_size / 2
                frontier_world.append((wx, wy))
            frontiers_by_drone = {drone_id: frontier_world}

            # --- Convert features to unified format ---
            features = []
            for feature in drone_data.get('features', []):
                if hasattr(feature, 'position'):
                    ftype = feature.feature_type if hasattr(feature, 'feature_type') else ''
                    features.append({
                        'type': 'ied' if ftype == 'ied' else 'object',
                        'position': feature.position,
                        'label': ftype[:6] if ftype != 'ied' else 'IED!',
                    })

            # --- Drone position ---
            dp = drone_data.get('position', (0, 0))
            drone_positions = [(dp, color)]

            # Title with coverage
            cov_pct = drone_data.get('coverage_pct', None)
            title = f"D{drone_id} Map ({cov_pct}%)" if cov_pct is not None else f"D{drone_id} Map"

            self._draw_map_panel(
                map_x, map_y, map_width, map_height,
                title, color, wb, grid_size,
                searched_cells=searched_cells,
                frontier_cells=frontiers_by_drone,
                wall_segments=wall_segments,
                drone_positions=drone_positions,
                features=features,
            )

        # Show "+N more" if more drones than displayed
        if len(drone_maps_data) > display_count:
            extra = len(drone_maps_data) - display_count
            extra_y = start_y + display_count * (map_height + map_spacing)
            extra_surf = self.font_medium.render(f"+{extra} more drone maps", True, Colors.DARK_GRAY)
            self.screen.blit(extra_surf, (start_x + 10, extra_y))
    
    def draw_search_debug(self, debug_info: dict, drone_pos: Tuple[float, float]):
        """Draw search debug: path dots, doorways, target, stats."""
        if not debug_info:
            return
        
        # Draw detected doorways as ORANGE squares
        doorways = debug_info.get('doorways', [])
        for dx, dy in doorways:
            dp = self._world_to_screen((dx, dy))
            pygame.draw.rect(self.screen, Colors.ORANGE, (int(dp[0])-5, int(dp[1])-5, 10, 10))
        
        # NOTE: Frontier cells (unknown boundaries) are now shown on the MINIMAP
        # as yellow dots, not on the main view
        
        # Draw path history as cyan dots with numbers
        # Path history visualization (optional debug)
        path_history = debug_info.get('path_history', [])
        for i, pos in enumerate(path_history):
            sp = self._world_to_screen(pos)
            pygame.draw.circle(self.screen, Colors.CYAN, (int(sp[0]), int(sp[1])), 3)
        # Note: Stats moved to main right panel to avoid duplication

    def notify_radio_sent(self):
        """Flash a small banner indicating radio transmission."""
        self._radio_last_ts = time.time()
    
    def _world_to_screen(self, world_pos: Tuple[float, float]) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates."""
        try:
            # Safety checks for valid coordinates
            if world_pos is None or len(world_pos) < 2:
                return (self.offset_x, self.offset_y)
            
            x, y = world_pos[0], world_pos[1]
            
            # Check for NaN or infinite values
            if not (math.isfinite(x) and math.isfinite(y)):
                return (self.offset_x, self.offset_y)
            
            screen_x = int(x * self.scale + self.offset_x)
            screen_y = int(y * self.scale + self.offset_y)
            
            # Clamp to reasonable screen bounds
            screen_x = max(-1000, min(2000, screen_x))
            screen_y = max(-1000, min(2000, screen_y))
            
            return (screen_x, screen_y)
        except (TypeError, ValueError, IndexError):
            # Fallback to default position if any error occurs
            return (self.offset_x, self.offset_y)
    
    def _screen_to_world(self, screen_pos: Tuple[int, int]) -> Tuple[float, float]:
        """Convert screen coordinates to world coordinates."""
        world_x = (screen_pos[0] - self.offset_x) / self.scale
        world_y = (screen_pos[1] - self.offset_y) / self.scale
        return (world_x, world_y)
    
    def _get_object_color(self, object_type: str) -> Tuple[int, int, int]:
        """Get color for object type."""
        color_map = {
            'person': Colors.PERSON,
            'equipment': Colors.EQUIPMENT,
            'hazard': Colors.HAZARD,
            'furniture': Colors.FURNITURE,
            'ied': (80, 80, 80),  # small gray dot
            'weapon': (80, 80, 80)
        }
        return color_map.get(object_type.lower(), Colors.GRAY)
    
    def _blend_colors(self, color1: Tuple[int, int, int], color2: Tuple[int, int, int], 
                     factor: float) -> Tuple[int, int, int]:
        """Blend two colors with given factor (0=color1, 1=color2)."""
        factor = max(0, min(1, factor))
        r = int(color1[0] * (1 - factor) + color2[0] * factor)
        g = int(color1[1] * (1 - factor) + color2[1] * factor)
        b = int(color1[2] * (1 - factor) + color2[2] * factor)
        return (r, g, b)
    
    def toggle_view_mode(self, mode: str):
        """Toggle visualization modes."""
        if mode == "lidar":
            self.show_lidar = not self.show_lidar
        elif mode == "map":
            self.show_grid_map = not self.show_grid_map
        elif mode == "path":
            self.show_path = not self.show_path
        elif mode == "detections":
            self.show_detections = not self.show_detections
