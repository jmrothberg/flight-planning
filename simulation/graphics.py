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
        """Draw LIDAR scan visualization - rays stop at walls.
        
        Args:
            drone: Drone object with position and orientation
            lidar_ranges: List of LIDAR distances
            color: Optional RGB tuple for ray color (default: RED)
        """
        if not self.show_lidar or not lidar_ranges:
            return
        
        ray_color = color if color else Colors.RED
        
        drone_pos = self._world_to_screen(drone.position[:2])
        angle_step = 2 * math.pi / len(lidar_ranges)
        
        # Subsample rays to reduce per-frame draw calls
        step = max(1, len(lidar_ranges) // 120)  # draw ~120 rays
        
        for i in range(0, len(lidar_ranges), step):
            distance = lidar_ranges[i]
            # Skip max-range returns to avoid lines that appear to pass through walls
            if distance >= 24.9:
                continue
            
            angle = drone.orientation + (i * angle_step)
            # LIDAR endpoint is exactly where the ray stopped (at wall or max range)
            end_x = drone.position[0] + distance * math.cos(angle)
            end_y = drone.position[1] + distance * math.sin(angle)
            end_pos = self._world_to_screen((end_x, end_y))
            
            # Draw LIDAR ray
            if drone_pos and end_pos:
                pygame.draw.line(self.screen, ray_color, drone_pos, end_pos, 1)
            
            # Draw hit point (wall hit)
            if end_pos:
                pygame.draw.circle(self.screen, ray_color, end_pos, 2)
    
    def draw_ui(self, drone, slam_system, comm_system, mission_start_time=None, search_method: Optional[str] = None, search_options: Optional[List[str]] = None, search_debug: Optional[Dict] = None):
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
        
        # Battery and Timer on same row
        battery_text = f"Battery: {status['battery']:.1f}%"
        # Color thresholds for quick glance
        battery_color = Colors.GREEN if status['battery'] > 50 else Colors.ORANGE if status['battery'] > 20 else Colors.RED
        battery_surface = self.font_small.render(battery_text, True, battery_color)
        self.screen.blit(battery_surface, (ui_rect.x + 10, y_offset))
        
        # Mission timer next to battery
        if mission_start_time is not None:
            # mission_start_time is now the simulated elapsed time in seconds
            elapsed = mission_start_time
            minutes = int(elapsed // 60)
            seconds = int(elapsed % 60)
            timer_text = f"Time: {minutes:02d}:{seconds:02d}"
            timer_surface = self.font_small.render(timer_text, True, Colors.GREEN)
            self.screen.blit(timer_surface, (ui_rect.x + 150, y_offset))  # Next to battery
        
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
            dist_text = f"Dist to entry: {dist_home}m"
            dist_surface = self.font_small.render(dist_text, True, Colors.BLACK)
            self.screen.blit(dist_surface, (ui_rect.x + 10, y_offset))
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
            "N: New Object Placement",
            "B: New Building Layout",
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

    def draw_minimap(self, minimap_data: Dict, drone_pos: Tuple[float, float]):
        """
        Draw the minimap showing discovered features.
        
        === POSITIONING CONTROLS ===
        minimap_x, minimap_y: Top-left corner of minimap frame
        minimap_width, minimap_height: Size of the minimap frame
        
        === MAPPING CONTROLS ===
        world_bounds: The world coordinates to map (min_x, min_y, max_x, max_y)
        - Decrease min_y to shift map content UP (e.g., -10 instead of -5)
        - Increase min_y to shift map content DOWN
        - Decrease min_x to shift map content RIGHT
        - Increase min_x to shift map content LEFT
        
        padding_x, padding_top, padding_bottom: Space between frame and mapped content
        - Increase to shrink the mapped area within the frame
        - Decrease to use more of the frame for mapping
        """
        # Position discovered map below the main building view (not at bottom)
        # Building view is roughly 500px tall starting at y=50
        minimap_width = 350   
        minimap_height = 280  
        minimap_x = 50        
        minimap_y = 580  # Below main map (50 + ~500 + 30 margin)
        
        # Minimap data debug removed - was not useful for improving search
        
        # Always draw minimap background and border
        minimap_rect = pygame.Rect(minimap_x, minimap_y, minimap_width, minimap_height)
        pygame.draw.rect(self.screen, Colors.WHITE, minimap_rect)  # White background
        pygame.draw.rect(self.screen, Colors.BLACK, minimap_rect, 2)  # Black border
        
        # Draw title
        title_surface = self.font_small.render("Discovered Map", True, Colors.BLACK)
        self.screen.blit(title_surface, (minimap_x, minimap_y - 15))
        
        
        if not minimap_data:
            # Show "Learning..." text
            learning_surface = self.font_small.render("Learning...", True, Colors.DARK_GRAY)
            text_rect = learning_surface.get_rect(center=(minimap_x + minimap_width//2, minimap_y + minimap_height//2))
            self.screen.blit(learning_surface, text_rect)
            return
        
        # SIMPLE - just use the data as-is and show everything
        world_bounds = minimap_data.get("world_bounds", (-5, -5, 55, 45))
        min_x, min_y, max_x, max_y = world_bounds
        
        # Extend view slightly to ensure edges aren't clipped
        margin = 5
        min_x -= margin
        max_x += margin
        min_y -= margin
        max_y += margin
        
        def world_to_minimap(world_pos):
            """Simple direct mapping from world to screen."""
            wx, wy = world_pos
            
            # Simple padding
            pad = 15
            
            # Direct linear mapping
            x_ratio = (wx - min_x) / (max_x - min_x) if max_x != min_x else 0.5
            y_ratio = (wy - min_y) / (max_y - min_y) if max_y != min_y else 0.5
            
            # Map to screen
            map_x = minimap_x + pad + int(x_ratio * (minimap_width - 2*pad))
            map_y = minimap_y + pad + int(y_ratio * (minimap_height - 2*pad))
            return (map_x, map_y)
        
        # Draw 3x3 SEARCHED GRID cells (colored squares showing IED-scanned areas)
        grid_size = minimap_data.get("grid_size", 3.0)
        
        # Check for multi-drone colored cells first
        searched_cells_colored = minimap_data.get("searched_cells_colored", [])
        if searched_cells_colored:
            # Multi-drone mode: draw each cell in drone's color (lighter shade)
            for cell_data in searched_cells_colored:
                if isinstance(cell_data, tuple) and len(cell_data) == 2:
                    (wx, wy), color = cell_data
                    # Make a lighter shade of the drone's color
                    light_color = (
                        min(255, color[0] + 100),
                        min(255, color[1] + 100),
                        min(255, color[2] + 100)
                    )
                    border_color = color
                else:
                    wx, wy = cell_data
                    light_color = (200, 255, 200)
                    border_color = (100, 200, 100)
                
                top_left = world_to_minimap((wx - grid_size/2, wy - grid_size/2))
                bot_right = world_to_minimap((wx + grid_size/2, wy + grid_size/2))
                w = max(bot_right[0] - top_left[0], 4)
                h = max(bot_right[1] - top_left[1], 4)
                rect = pygame.Rect(top_left[0], top_left[1], w, h)
                pygame.draw.rect(self.screen, light_color, rect)
                pygame.draw.rect(self.screen, border_color, rect, 1)
        else:
            # Single drone mode: all cells light green
            searched_cells = minimap_data.get("searched_cells", [])
            for wx, wy in searched_cells:
                top_left = world_to_minimap((wx - grid_size/2, wy - grid_size/2))
                bot_right = world_to_minimap((wx + grid_size/2, wy + grid_size/2))
                w = max(bot_right[0] - top_left[0], 4)
                h = max(bot_right[1] - top_left[1], 4)
                rect = pygame.Rect(top_left[0], top_left[1], w, h)
                pygame.draw.rect(self.screen, (200, 255, 200), rect)  # Light green fill
                pygame.draw.rect(self.screen, (100, 200, 100), rect, 1)  # Green border
        
        # Draw FRONTIER cells with different shapes per drone
        frontiers_by_drone = minimap_data.get("frontiers_by_drone", {})
        if frontiers_by_drone:
            # Multi-drone: different shapes per drone
            # Drone 0: circles, Drone 1: triangles, Drone 2: squares, Drone 3: diamonds
            for drone_id, frontiers in frontiers_by_drone.items():
                for wx, wy in frontiers:
                    map_pos = world_to_minimap((wx, wy))
                    x, y = map_pos
                    if drone_id == 0:
                        # Circles for drone 0
                        pygame.draw.circle(self.screen, (255, 200, 0), map_pos, 4)
                    elif drone_id == 1:
                        # Triangles for drone 1
                        points = [(x, y-5), (x-4, y+3), (x+4, y+3)]
                        pygame.draw.polygon(self.screen, (255, 200, 0), points)
                    elif drone_id == 2:
                        # Squares for drone 2
                        pygame.draw.rect(self.screen, (255, 200, 0), (x-3, y-3, 6, 6))
                    else:
                        # Diamonds for drone 3+
                        points = [(x, y-4), (x+4, y), (x, y+4), (x-4, y)]
                        pygame.draw.polygon(self.screen, (255, 200, 0), points)
        else:
            # Single drone: all circles
            frontier_cells = minimap_data.get("frontier_cells", [])
            for wx, wy in frontier_cells:
                map_pos = world_to_minimap((wx, wy))
                pygame.draw.circle(self.screen, (255, 200, 0), map_pos, 4)  # Yellow dot
        
        # Draw wall segments (black lines) - on top of grid
        wall_segments = minimap_data.get("wall_segments", [])
        for wall_start, wall_end in wall_segments:
            start_pos = world_to_minimap(wall_start)
            end_pos = world_to_minimap(wall_end)
            pygame.draw.line(self.screen, Colors.BLACK, start_pos, end_pos, 2)
        
        # Draw discovered doors (green squares)
        doors = minimap_data.get("doors", [])
        for door_pos in doors:
            map_pos = world_to_minimap(door_pos)
            pygame.draw.rect(self.screen, Colors.GREEN, (map_pos[0]-2, map_pos[1]-2, 4, 4))
        
        # Draw discovered objects with LABELS (blue circles)
        objects = minimap_data.get("objects", [])
        for obj_data in objects:
            if isinstance(obj_data, tuple) and len(obj_data) >= 2:
                obj_pos = obj_data[0]
                # Check if there's label info
                label = obj_data[2] if len(obj_data) > 2 else ""
            else:
                continue
            map_pos = world_to_minimap(obj_pos)
            pygame.draw.circle(self.screen, Colors.BLUE, map_pos, 3)
            # Draw label if available
            if label:
                lbl = self.font_small.render(label[:6], True, Colors.BLUE)
                self.screen.blit(lbl, (map_pos[0]+4, map_pos[1]-6))
        
        # Draw discovered IEDs with LABELS (red circles with warning)
        ieds = minimap_data.get("ieds", [])
        for ied_data in ieds:
            if isinstance(ied_data, tuple) and len(ied_data) >= 2:
                ied_pos = ied_data[0]
            else:
                continue
            map_pos = world_to_minimap(ied_pos)
            pygame.draw.circle(self.screen, Colors.RED, map_pos, 4)
            pygame.draw.circle(self.screen, Colors.WHITE, map_pos, 4, 1)
            # Always label IEDs
            lbl = self.font_small.render("IED!", True, Colors.RED)
            self.screen.blit(lbl, (map_pos[0]+5, map_pos[1]-6))
        
        # Draw breadcrumb trail (light green dots showing where drone has been)
        breadcrumbs = minimap_data.get("breadcrumbs", [])
        if breadcrumbs:
            # Sample breadcrumbs to avoid too many dots
            for i in range(0, len(breadcrumbs), 3):  # Every 3rd breadcrumb
                if i < len(breadcrumbs):
                    crumb_pos = world_to_minimap(breadcrumbs[i])
                    pygame.draw.circle(self.screen, (150, 255, 150), crumb_pos, 1)  # Light green dots
        
        # Draw drone position(s) using same coordinate system
        drone_positions = minimap_data.get("drone_positions", None)
        if drone_positions:
            # Multi-drone: draw all drones with their colors
            for pos, color in drone_positions:
                dm_pos = world_to_minimap(pos)
                pygame.draw.circle(self.screen, color, dm_pos, 5)
                pygame.draw.circle(self.screen, Colors.WHITE, dm_pos, 5, 1)
        else:
            # Single drone: red dot
            drone_minimap_pos = world_to_minimap(drone_pos)
            pygame.draw.circle(self.screen, Colors.RED, drone_minimap_pos, 5)
            pygame.draw.circle(self.screen, Colors.WHITE, drone_minimap_pos, 5, 1)
        
        
    
    def draw_drone_maps(self, drone_maps_data: List[dict], world_bounds: Tuple[float, float, float, float], wall_segments: List = None):
        """Draw individual maps for each drone showing their local knowledge.
        
        Args:
            drone_maps_data: List of dicts, one per drone, containing:
                - 'free_cells': set of (x,y) grid cells
                - 'searched_cells': set of (x,y) grid cells  
                - 'wall_cells': set of (x,y) grid cells
                - 'searched_by_drone': dict of drone_id -> set of cells (for color-coded display)
                - 'color': drone color
                - 'drone_id': drone index
                - 'position': current position
            world_bounds: (min_x, min_y, max_x, max_y) for coordinate mapping
            wall_segments: List of wall segments from environment for building outline
        """
        if not drone_maps_data:
            return
        
        if wall_segments is None:
            wall_segments = []
        
        # Vertical column between main map and status panel
        # Same scale as Discovered Map (350x280) for uniform appearance
        map_width = 350
        map_height = 280
        start_x = self.window_size[0] - 250 - map_width - 10  # Left of status panel
        start_y = 35  # Reset to original position
        map_spacing = 35  # More space between maps
        
        min_x, min_y, max_x, max_y = world_bounds
        grid_size = 3.0
        
        for idx, drone_data in enumerate(drone_maps_data):
            # Position for this drone's map (vertical stack with more spacing)
            map_x = start_x
            map_y = start_y + idx * (map_height + map_spacing)
            
            # Background
            map_rect = pygame.Rect(map_x, map_y, map_width, map_height)
            pygame.draw.rect(self.screen, Colors.WHITE, map_rect)
            pygame.draw.rect(self.screen, drone_data.get('color', Colors.BLACK), map_rect, 2)
            
            # Title with drone ID
            drone_id = drone_data.get('drone_id', idx)
            title = f"D{drone_id} Map"
            title_surf = self.font_small.render(title, True, drone_data.get('color', Colors.BLACK))
            self.screen.blit(title_surf, (map_x + 2, map_y - 14))
            
            # Coordinate transforms - use RATIO-BASED mapping like Discovered Map
            # This ensures content is properly centered and scaled within the frame
            pad = 15  # Same padding as Discovered Map
            
            # Cell size in pixels (estimate for drawing)
            scale_factor = min((map_width - 2*pad) / (max_x - min_x), 
                              (map_height - 2*pad) / (max_y - min_y)) if max_x > min_x and max_y > min_y else 1
            cell_px = max(4, int(grid_size * scale_factor))
            
            # Vertical offset to shift content UP within box (fixes content falling out of bottom)
            content_y_offset = -25  # Reduced from -50 to keep content inside box
            
            def to_map(cell):
                """Convert grid cell to screen position - ratio-based like Discovered Map"""
                cx, cy = cell
                # Add grid_size/2 to get cell CENTER (not corner) - fixes 1/2 grid offset bug
                wx = cx * grid_size + grid_size / 2
                wy = cy * grid_size + grid_size / 2
                # Use ratio-based mapping for proper centering
                x_ratio = (wx - min_x) / (max_x - min_x) if max_x != min_x else 0.5
                y_ratio = (wy - min_y) / (max_y - min_y) if max_y != min_y else 0.5
                px = map_x + pad + int(x_ratio * (map_width - 2*pad))
                py = map_y + pad + int(y_ratio * (map_height - 2*pad)) + content_y_offset
                return (px, py)
            
            def world_to_map(world_pos):
                """Convert world position to screen position - ratio-based like Discovered Map"""
                wx, wy = world_pos
                x_ratio = (wx - min_x) / (max_x - min_x) if max_x != min_x else 0.5
                y_ratio = (wy - min_y) / (max_y - min_y) if max_y != min_y else 0.5
                px = map_x + pad + int(x_ratio * (map_width - 2*pad))
                py = map_y + pad + int(y_ratio * (map_height - 2*pad)) + content_y_offset
                return (px, py)
            
            # Use pad instead of padding for bounds checking
            padding = pad
            
            # 0. BUILDING OUTLINE (BLACK lines like Discovered Map) - draws first as background
            for wall_start, wall_end in wall_segments:
                start_pos = world_to_map(wall_start)
                end_pos = world_to_map(wall_end)
                # Clip to map bounds
                if (map_x <= start_pos[0] <= map_x + map_width or map_x <= end_pos[0] <= map_x + map_width):
                    pygame.draw.line(self.screen, Colors.BLACK, start_pos, end_pos, 2)  # Black, thickness 2
            
            # Get data
            color = drone_data.get('color', (200, 200, 200))
            light_color = (min(255, color[0] + 100), min(255, color[1] + 100), min(255, color[2] + 100))
            walls = drone_data.get('wall_cells', set())
            searched = drone_data.get('searched_cells', set())
            free = drone_data.get('free_cells', set())
            features = drone_data.get('features', [])
            # For color-coded display: which drone searched each cell
            searched_by_drone = drone_data.get('searched_by_drone', {})
            drone_colors = drone_data.get('drone_colors', {})
            
            # Cell drawing size (half for offset, full for rect)
            cs = max(2, cell_px // 2)
            
            # 1. SEARCHED CELLS FIRST - draw in color of drone that searched them
            # RULE: Maps become IDENTICAL after communication sync, EXCEPT:
            #       If THIS drone searched a cell, show it in THIS drone's color
            #       (even if another drone also searched the same cell)
            # Style: match Discovered Map with fill + border
            if searched_by_drone:
                # Collect all searched cells from all drones
                all_searched_cells = set()
                for cells in searched_by_drone.values():
                    all_searched_cells.update(cells)
                
                # Get cells this drone searched (for priority coloring)
                my_cells = searched_by_drone.get(drone_id, set())
                
                # Draw each cell once, with correct color
                for cell in all_searched_cells:
                    # Determine color: if THIS drone searched it, use THIS drone's color
                    # Otherwise use the color of whoever searched it
                    if cell in my_cells:
                        # This drone searched it - use this drone's color
                        cell_color = light_color
                        border_color = color
                    else:
                        # Find which drone searched it and use their color
                        cell_color = light_color  # Default
                        border_color = color
                        for search_drone_id, cells in searched_by_drone.items():
                            if cell in cells:
                                if search_drone_id in drone_colors:
                                    search_color = drone_colors[search_drone_id]
                                    cell_color = (min(255, search_color[0] + 100), 
                                                  min(255, search_color[1] + 100), 
                                                  min(255, search_color[2] + 100))
                                    border_color = search_color
                                break
                    
                    pos = to_map(cell)
                    if map_x + padding < pos[0] < map_x + map_width - padding and map_y + padding < pos[1] < map_y + map_height - padding:
                        rect = pygame.Rect(pos[0]-cs, pos[1]-cs, cell_px, cell_px)
                        pygame.draw.rect(self.screen, cell_color, rect)  # Fill
                        pygame.draw.rect(self.screen, border_color, rect, 1)  # Border
            else:
                # Fallback: draw all searched cells in this drone's color (old behavior)
                for cell in searched:
                    pos = to_map(cell)
                    if map_x + padding < pos[0] < map_x + map_width - padding and map_y + padding < pos[1] < map_y + map_height - padding:
                        rect = pygame.Rect(pos[0]-cs, pos[1]-cs, cell_px, cell_px)
                        pygame.draw.rect(self.screen, light_color, rect)  # Fill
                        pygame.draw.rect(self.screen, color, rect, 1)  # Border
            
            # 2. FRONTIERS (yellow shapes) - same size as Discovered Map
            # Frontiers = free cells that haven't been searched yet (excluding wall cells)
            frontier = free - searched - walls
            # Use fixed sizes matching Discovered Map (4 for circles, 4-5 for triangles/shapes)
            for cell in frontier:
                pos = to_map(cell)
                if map_x + padding < pos[0] < map_x + map_width - padding and map_y + padding < pos[1] < map_y + map_height - padding:
                    x, y = pos
                    if drone_id == 0:
                        # Circles for drone 0 - radius 4 like Discovered Map
                        pygame.draw.circle(self.screen, (255, 200, 0), pos, 4)
                    elif drone_id == 1:
                        # Triangles for drone 1 - same size as Discovered Map
                        points = [(x, y-5), (x-4, y+3), (x+4, y+3)]
                        pygame.draw.polygon(self.screen, (255, 200, 0), points)
                    elif drone_id == 2:
                        # Squares for drone 2 - 6x6 like Discovered Map
                        pygame.draw.rect(self.screen, (255, 200, 0), (x-3, y-3, 6, 6))
                    else:
                        # Diamonds for drone 3+ - size 4 like Discovered Map
                        points = [(x, y-4), (x+4, y), (x, y+4), (x-4, y)]
                        pygame.draw.polygon(self.screen, (255, 200, 0), points)
            
            # NOTE: Walls are shown via building outline (BLACK lines) drawn earlier
            # DO NOT draw wall_cells as filled black rectangles - that's the bug!
            
            # 3. FEATURES (IEDs red, objects blue) ON TOP - with labels like Discovered Map
            for feature in features:
                if hasattr(feature, 'position'):
                    fx, fy = feature.position
                    fcell = (int(fx / grid_size), int(fy / grid_size))
                    fpos = to_map(fcell)
                    if map_x < fpos[0] < map_x + map_width and map_y < fpos[1] < map_y + map_height:
                        ftype = feature.feature_type if hasattr(feature, 'feature_type') else ''
                        if ftype == 'ied':
                            # Red circle with white border for IED + label
                            pygame.draw.circle(self.screen, Colors.RED, fpos, 4)
                            pygame.draw.circle(self.screen, Colors.WHITE, fpos, 4, 1)
                            # Add "IED!" label like Discovered Map
                            lbl = self.font_small.render("IED!", True, Colors.RED)
                            self.screen.blit(lbl, (fpos[0]+5, fpos[1]-6))
                        else:
                            # Blue circle for other objects + label
                            pygame.draw.circle(self.screen, Colors.BLUE, fpos, 3)
                            # Add type label if available
                            if ftype:
                                lbl = self.font_small.render(ftype[:6], True, Colors.BLUE)
                                self.screen.blit(lbl, (fpos[0]+4, fpos[1]-6))
            
            # 5. DRONE POSITION (on very top)
            drone_pos = drone_data.get('position', (0, 0))
            drone_cell = (int(drone_pos[0] / grid_size), int(drone_pos[1] / grid_size))
            dp = to_map(drone_cell)
            if map_x < dp[0] < map_x + map_width and map_y < dp[1] < map_y + map_height:
                pygame.draw.circle(self.screen, color, dp, 5)
                pygame.draw.circle(self.screen, Colors.WHITE, dp, 5, 1)
    
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
