"""
Navigation algorithms for autonomous drone flight
Includes A* pathfinding and RRT exploration algorithms
Designed for STM32 portability with minimal memory usage
"""

import numpy as np
import math
import random
from typing import List, Tuple, Optional, Set
import heapq
from dataclasses import dataclass
# from core.frontier_exploration import FrontierExplorer  # Not used with lawn mower pattern

@dataclass
class Node:
    """Node for pathfinding algorithms."""
    x: float
    y: float
    g_cost: float = float('inf')  # Cost from start
    h_cost: float = 0.0           # Heuristic cost to goal
    parent: Optional['Node'] = None
    
    @property
    def f_cost(self) -> float:
        """Total cost (g + h)."""
        return self.g_cost + self.h_cost
    
    def __lt__(self, other):
        """For priority queue comparison."""
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        """Equality check for nodes."""
        if not isinstance(other, Node):
            return False
        return abs(self.x - other.x) < 0.1 and abs(self.y - other.y) < 0.1

class AStarPathfinder:
    """
    A* pathfinding algorithm for optimal path planning.
    Memory efficient implementation suitable for embedded systems.
    """
    
    def __init__(self, grid_resolution: float = 0.5):
        """Initialize A* pathfinder with grid resolution in meters."""
        self.grid_resolution = grid_resolution
        self.max_iterations = 3000  # Prevent infinite loops (more robust indoors)
    
    def find_path(self, start: Tuple[float, float], 
                  goal: Tuple[float, float], 
                  environment) -> List[Tuple[float, float]]:
        """
        Find optimal path from start to goal using A*.
        Returns list of waypoints or empty list if no path found.
        """
        # Convert to grid coordinates
        start_node = Node(start[0], start[1])
        goal_node = Node(goal[0], goal[1])
        
        # Initialize open (min-heap) and closed sets
        open_set: List[Node] = []
        closed_set = set()
        
        start_node.g_cost = 0
        start_node.h_cost = self._heuristic(start_node, goal_node)
        heapq.heappush(open_set, start_node)
        
        iterations = 0
        while open_set and iterations < self.max_iterations:
            iterations += 1
            
            # Get node with lowest f_cost
            current = heapq.heappop(open_set)
            
            # Check if we reached the goal
            if self._distance(current, goal_node) < self.grid_resolution:
                return self._reconstruct_path(current)
            
            # Add to closed set
            closed_set.add((round(current.x/self.grid_resolution), 
                          round(current.y/self.grid_resolution)))
            
            # Check neighbors
            for neighbor_pos in self._get_neighbors(current.x, current.y):
                # Skip if position is invalid
                # KNOWN FIX: inflate collision radius slightly to avoid wall grazing
                if not environment.is_position_valid(neighbor_pos):
                    continue
                
                neighbor = Node(neighbor_pos[0], neighbor_pos[1])
                neighbor_grid = (round(neighbor.x/self.grid_resolution),
                               round(neighbor.y/self.grid_resolution))
                
                # Skip if already processed
                if neighbor_grid in closed_set:
                    continue
                
                # Calculate costs
                tentative_g = current.g_cost + self._distance(current, neighbor)
                
                # Check if this is a better path
                existing_node = None
                for node in open_set:
                    if self._distance(node, neighbor) < self.grid_resolution:
                        existing_node = node
                        break
                
                if existing_node is None:
                    # New node
                    neighbor.g_cost = tentative_g
                    neighbor.h_cost = self._heuristic(neighbor, goal_node)
                    neighbor.parent = current
                    heapq.heappush(open_set, neighbor)
                elif tentative_g < existing_node.g_cost:
                    # Better path found
                    existing_node.g_cost = tentative_g
                    existing_node.parent = current
                    heapq.heapify(open_set)  # Re-heapify since costs changed
        
        # No path found
        return []
    
    def _get_neighbors(self, x: float, y: float) -> List[Tuple[float, float]]:
        """Get 8-connected neighbors for a grid position."""
        neighbors = []
        for dx in [-self.grid_resolution, 0, self.grid_resolution]:
            for dy in [-self.grid_resolution, 0, self.grid_resolution]:
                if dx == 0 and dy == 0:
                    continue
                neighbors.append((x + dx, y + dy))
        return neighbors
    
    def _heuristic(self, node1: Node, node2: Node) -> float:
        """Calculate heuristic distance (Euclidean)."""
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def _distance(self, node1: Node, node2: Node) -> float:
        """Calculate actual distance between nodes."""
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def _reconstruct_path(self, node: Node) -> List[Tuple[float, float]]:
        """Reconstruct path from goal node back to start."""
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.reverse()
        return path

class RRTExplorer:
    """
    Rapidly-exploring Random Tree for exploration and path planning.
    Efficient for unknown environments and exploration tasks.
    """
    
    def __init__(self, step_size: float = 1.0, max_iterations: int = 500):
        """Initialize RRT explorer."""
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.nodes = []
    
    def explore_area(self, start: Tuple[float, float], 
                    environment, 
                    exploration_bounds: Tuple[float, float, float, float]) -> List[Tuple[float, float]]:
        """
        Explore area using RRT to find interesting locations.
        Returns list of exploration waypoints.
        """
        min_x, min_y, max_x, max_y = exploration_bounds
        
        # Initialize with start node
        self.nodes = [Node(start[0], start[1])]
        exploration_points = []
        
        for _ in range(self.max_iterations):
            # Generate random point
            rand_x = random.uniform(min_x, max_x)
            rand_y = random.uniform(min_y, max_y)
            rand_point = (rand_x, rand_y)
            
            # Find nearest node in tree
            nearest_node = self._find_nearest_node(rand_point)
            
            # Generate new point towards random point
            new_point = self._steer(nearest_node, rand_point)
            
            # Check if new point is valid with extra margin for walls
            if environment.is_position_valid(new_point, radius=0.5):
                # Check if path to new point is clear
                if self._is_path_clear(nearest_node, new_point, environment):
                    new_node = Node(new_point[0], new_point[1])
                    new_node.parent = nearest_node
                    self.nodes.append(new_node)
                
                # Add as exploration point if it's interesting
                if self._is_interesting_location(new_point, exploration_points):
                    exploration_points.append(new_point)
        
        return exploration_points
    
    def find_path_rrt(self, start: Tuple[float, float], 
                     goal: Tuple[float, float], 
                     environment) -> List[Tuple[float, float]]:
        """Find path using RRT algorithm."""
        # Initialize with start node
        self.nodes = [Node(start[0], start[1])]
        
        for _ in range(self.max_iterations):
            # Bias towards goal 10% of the time
            if random.random() < 0.1:
                rand_point = goal
            else:
                # Generate random point in environment bounds
                rand_x = random.uniform(0, environment.width)
                rand_y = random.uniform(0, environment.height)
                rand_point = (rand_x, rand_y)
            
            # Find nearest node and steer towards random point
            nearest_node = self._find_nearest_node(rand_point)
            new_point = self._steer(nearest_node, rand_point)
            
            # Check validity
            if (environment.is_position_valid(new_point) and 
                self._is_path_clear(nearest_node, new_point, environment)):
                
                new_node = Node(new_point[0], new_point[1])
                new_node.parent = nearest_node
                self.nodes.append(new_node)
                
                # Check if we reached the goal
                if self._distance_to_point(new_point, goal) < self.step_size:
                    goal_node = Node(goal[0], goal[1])
                    goal_node.parent = new_node
                    return self._reconstruct_path_rrt(goal_node)
        
        return []  # No path found
    
    def _find_nearest_node(self, point: Tuple[float, float]) -> Node:
        """Find nearest node in tree to given point."""
        min_distance = float('inf')
        nearest_node = self.nodes[0]
        
        for node in self.nodes:
            distance = self._distance_to_point((node.x, node.y), point)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        
        return nearest_node
    
    def _steer(self, from_node: Node, to_point: Tuple[float, float]) -> Tuple[float, float]:
        """Steer from node towards point with step size limit."""
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance <= self.step_size:
            return to_point
        
        # Normalize and scale to step size
        ratio = self.step_size / distance
        new_x = from_node.x + dx * ratio
        new_y = from_node.y + dy * ratio
        
        return (new_x, new_y)
    
    def _is_path_clear(self, from_node: Node, to_point: Tuple[float, float], 
                      environment) -> bool:
        """Check if path between two points is collision-free."""
        steps = int(self._distance_to_point((from_node.x, from_node.y), to_point) / 0.2)
        steps = max(1, steps)
        
        for i in range(steps + 1):
            t = i / steps
            check_x = from_node.x + t * (to_point[0] - from_node.x)
            check_y = from_node.y + t * (to_point[1] - from_node.y)
            
            if not environment.is_position_valid((check_x, check_y)):
                return False
        
        return True
    
    def _is_interesting_location(self, point: Tuple[float, float], 
                               existing_points: List[Tuple[float, float]]) -> bool:
        """Check if location is interesting for exploration."""
        # Ensure minimum distance from existing exploration points
        min_distance = 3.0
        for existing in existing_points:
            if self._distance_to_point(point, existing) < min_distance:
                return False
        return True
    
    def _distance_to_point(self, point1: Tuple[float, float], 
                          point2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def _reconstruct_path_rrt(self, node: Node) -> List[Tuple[float, float]]:
        """Reconstruct path from goal node back to start."""
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.reverse()
        return path

class PathPlanner:
    """
    High-level path planning coordinator.
    Combines different algorithms for optimal mission execution.
    """
    
    def __init__(self):
        """Initialize path planner with algorithm instances."""
        self.astar = AStarPathfinder()  # Still used for obstacle avoidance
        # Revert: disable RRT fallback to restore previous stable behavior
        self.rrt = None
        # self.frontier_explorer = FrontierExplorer()  # Not used with lawn mower pattern
        self.current_path = []
        self.exploration_points = []
        self._cached_door_points: List[Tuple[float, float]] = []
    
    def plan_exploration_mission(self, start: Tuple[float, float], 
                                 environment) -> List[Tuple[float, float]]:
        """Plan exploration - now handled by wall follower, this is just a fallback."""
        # Wall follower handles exploration, this is just a fallback
        # Return some basic waypoints if needed
        self.exploration_points = []
        return self.exploration_points
    
    def get_path_to_next_point(self, current: Tuple[float, float], 
                              target: Tuple[float, float], 
                              environment) -> List[Tuple[float, float]]:
        """Get optimal path to target using A*."""
        return self.astar.find_path(current, target, environment)
    
    def _sort_points_by_distance(self, start: Tuple[float, float], 
                                points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Sort points by distance from start for efficient traversal."""
        def distance_key(point):
            return math.sqrt((point[0] - start[0])**2 + (point[1] - start[1])**2)
        
        return sorted(points, key=distance_key)
    
    # Removed _order_points_nearest_neighbor - not used with lawn mower pattern

    def get_door_points(self, environment) -> List[Tuple[float, float]]:
        """Public method to get likely door gap points."""
        # If we have a cache from last planning, reuse; otherwise compute fresh
        if self._cached_door_points:
            return self._cached_door_points
        self._cached_door_points = self._detect_door_gaps(environment)
        return self._cached_door_points

    def _detect_door_gaps(self, environment, min_gap: float = 0.8, max_gap: float = 5.0) -> List[Tuple[float, float]]:
        """Detect likely door openings from wall segments and return mid-gap waypoints.
        Heuristic: identify gaps between colinear wall segments and place a waypoint in the gap.
        """
        horizontal_by_y = {}
        vertical_by_x = {}
        for (x1, y1), (x2, y2) in environment.walls:
            if abs(y1 - y2) < 1e-6:
                y = y1
                x_start, x_end = (x1, x2) if x1 <= x2 else (x2, x1)
                horizontal_by_y.setdefault(y, []).append((x_start, x_end))
            elif abs(x1 - x2) < 1e-6:
                x = x1
                y_start, y_end = (y1, y2) if y1 <= y2 else (y2, y1)
                vertical_by_x.setdefault(x, []).append((y_start, y_end))
        
        gap_points: List[Tuple[float, float]] = []
        # Process horizontal walls to find x-gaps at fixed y
        for y, spans in horizontal_by_y.items():
            spans.sort()
            for i in range(len(spans) - 1):
                end_a = spans[i][1]
                start_b = spans[i+1][0]
                gap = start_b - end_a
                if min_gap <= gap <= max_gap:
                    mid_x = (end_a + start_b) / 2.0
                    # Try slight offset to ensure validity
                    candidates = [(mid_x, y - 0.5), (mid_x, y + 0.5), (mid_x, y)]
                    for cx, cy in candidates:
                        if environment.is_position_valid((cx, cy)):
                            gap_points.append((cx, cy))
                            break
        # Process vertical walls to find y-gaps at fixed x
        for x, spans in vertical_by_x.items():
            spans.sort()
            for i in range(len(spans) - 1):
                end_a = spans[i][1]
                start_b = spans[i+1][0]
                gap = start_b - end_a
                if min_gap <= gap <= max_gap:
                    mid_y = (end_a + start_b) / 2.0
                    candidates = [(x - 0.5, mid_y), (x + 0.5, mid_y), (x, mid_y)]
                    for cx, cy in candidates:
                        if environment.is_position_valid((cx, cy)):
                            gap_points.append((cx, cy))
                            break
        return gap_points
