import numpy as np
from collections import deque
import heapq

class PathPlanner:
    def __init__(self):
        self.path = []
        
    def plan_path(self, start_state, goal, occupancy_grid):
        """Plan path from start to goal using A*"""
        start = (int((start_state[0] - occupancy_grid.origin_x) / occupancy_grid.resolution),
                int((start_state[1] - occupancy_grid.origin_y) / occupancy_grid.resolution))
        
        goal_grid = (int((goal[0] - occupancy_grid.origin_x) / occupancy_grid.resolution),
                    int((goal[1] - occupancy_grid.origin_y) / occupancy_grid.resolution))
        
        # Ensure start and goal are within bounds
        if not self._is_valid_point(start, occupancy_grid):
            print(f"Start point {start} is out of bounds")
            return None
            
        if not self._is_valid_point(goal_grid, occupancy_grid):
            print(f"Goal point {goal_grid} is out of bounds")
            return None
        
        path_grid = self._astar(start, goal_grid, occupancy_grid)
        
        if path_grid is None:
            print("No path found by A*")
            return None
            
        # Convert grid path to world coordinates
        path_world = []
        for grid_point in path_grid:
            world_x = grid_point[0] * occupancy_grid.resolution + occupancy_grid.origin_x
            world_y = grid_point[1] * occupancy_grid.resolution + occupancy_grid.origin_y
            path_world.append([world_x, world_y])
            
        return np.array(path_world)
    
    def _is_valid_point(self, point, occupancy_grid):
        """Check if point is within grid bounds"""
        return (0 <= point[0] < occupancy_grid.width and 
                0 <= point[1] < occupancy_grid.height)
        
    def _astar(self, start, goal, occupancy_grid):
        """A* pathfinding algorithm"""
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                return self._reconstruct_path(came_from, current)
                
            for neighbor in self._get_neighbors(current, occupancy_grid):
                tentative_g_score = g_score[current] + self._distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal)
                    
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        
        return None
        
    def _get_neighbors(self, node, occupancy_grid):
        """Get valid neighbors of a node"""
        neighbors = []
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        for dx, dy in directions:
            neighbor = (node[0] + dx, node[1] + dy)
            
            if (self._is_valid_point(neighbor, occupancy_grid) and
                not occupancy_grid.grid[neighbor[1], neighbor[0]]):
                neighbors.append(neighbor)
                
        return neighbors
        
    def _heuristic(self, a, b):
        """Euclidean distance heuristic"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
    def _distance(self, a, b):
        """Distance between two nodes"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]