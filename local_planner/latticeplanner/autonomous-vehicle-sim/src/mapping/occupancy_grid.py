import numpy as np
from typing import Dict, Any, List, Tuple

class OccupancyGrid:
    """Optimized Occupancy Grid for complex environments"""
    
    def __init__(self, config: Dict[str, Any]):
        self.width = config['width']
        self.height = config['height']
        self.resolution = config['resolution']
        self.origin_x = config['origin_x']
        self.origin_y = config['origin_y']
        
        # Initialize grid
        self.grid = np.zeros((self.height, self.width), dtype=np.float32)
        
        # Cache for performance
        self._obstacle_cache = []
        
        print(f"✅ OccupancyGrid created: {self.width}x{self.height} @ {self.resolution}m/cell")
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices"""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates"""
        x = self.origin_x + grid_x * self.resolution
        y = self.origin_y + grid_y * self.resolution
        return x, y
    
    def is_valid_grid_pos(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid position is valid"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def add_rectangle_obstacle(self, center_x: float, center_y: float, 
                             width: float, height: float, value: float = 1.0):
        """Add rectangular obstacle to grid"""
        # Convert to grid coordinates
        grid_center_x, grid_center_y = self.world_to_grid(center_x, center_y)
        
        # Calculate grid dimensions
        grid_width = int(width / self.resolution) + 1
        grid_height = int(height / self.resolution) + 1
        
        # Calculate bounds
        x_min = max(0, grid_center_x - grid_width // 2)
        x_max = min(self.width, grid_center_x + grid_width // 2 + 1)
        y_min = max(0, grid_center_y - grid_height // 2)
        y_max = min(self.height, grid_center_y + grid_height // 2 + 1)
        
        # Set obstacle
        self.grid[y_min:y_max, x_min:x_max] = value
        
        # Cache obstacle for later use
        self._obstacle_cache.append({
            'center_x': center_x, 'center_y': center_y,
            'width': width, 'height': height, 'value': value
        })
    
    def add_circle_obstacle(self, center_x: float, center_y: float, 
                          radius: float, value: float = 1.0):
        """Add circular obstacle to grid"""
        grid_center_x, grid_center_y = self.world_to_grid(center_x, center_y)
        grid_radius = int(radius / self.resolution) + 1
        
        # Create circular mask
        y_indices, x_indices = np.ogrid[:self.height, :self.width]
        mask = ((x_indices - grid_center_x) ** 2 + 
                (y_indices - grid_center_y) ** 2) <= grid_radius ** 2
        
        self.grid[mask] = value
    
    def add_road_boundaries(self, road_data: Dict[str, Any]):
        """Add road boundaries to grid"""
        if road_data['type'].startswith('horizontal'):
            self._add_horizontal_road(road_data)
        else:
            self._add_vertical_road(road_data)
    
    def _add_horizontal_road(self, road: Dict[str, Any]):
        """Add horizontal road boundaries"""
        y = road['y']
        width = road['width']
        x_start = road['x_start']
        x_end = road['x_end']
        
        # Road boundaries
        boundary_thickness = 0.5
        
        # North boundary
        for x in np.arange(x_start, x_end, self.resolution * 2):
            self.add_rectangle_obstacle(x, y + width/2 + boundary_thickness, 
                                      self.resolution * 3, boundary_thickness)
        
        # South boundary
        for x in np.arange(x_start, x_end, self.resolution * 2):
            self.add_rectangle_obstacle(x, y - width/2 - boundary_thickness, 
                                      self.resolution * 3, boundary_thickness)
        
        # Lane markings (optional, for visualization)
        num_lanes = road.get('lanes', 2)
        if num_lanes > 1:
            for lane in range(1, num_lanes):
                lane_y = y - width/2 + (lane * width / num_lanes)
                for x in np.arange(x_start, x_end, 3.0):  # Dashed lines
                    self.add_rectangle_obstacle(x, lane_y, 1.0, 0.1, 0.3)  # Low occupancy
    
    def _add_vertical_road(self, road: Dict[str, Any]):
        """Add vertical road boundaries"""
        x = road['x']
        width = road['width']
        y_start = road['y_start']
        y_end = road['y_end']
        
        # Road boundaries
        boundary_thickness = 0.5
        
        # West boundary
        for y in np.arange(y_start, y_end, self.resolution * 2):
            self.add_rectangle_obstacle(x - width/2 - boundary_thickness, y, 
                                      boundary_thickness, self.resolution * 3)
        
        # East boundary
        for y in np.arange(y_start, y_end, self.resolution * 2):
            self.add_rectangle_obstacle(x + width/2 + boundary_thickness, y, 
                                      boundary_thickness, self.resolution * 3)
        
        # Lane markings
        num_lanes = road.get('lanes', 2)
        if num_lanes > 1:
            for lane in range(1, num_lanes):
                lane_x = x - width/2 + (lane * width / num_lanes)
                for y in np.arange(y_start, y_end, 3.0):
                    self.add_rectangle_obstacle(lane_x, y, 0.1, 1.0, 0.3)
    
    def populate_from_scenario(self, city_data: Dict[str, Any]):
        """Populate grid from city scenario data"""
        print("🗺️ Populating occupancy grid from scenario...")
        
        # Clear existing grid
        self.grid.fill(0.0)
        self._obstacle_cache.clear()
        
        # Add roads (boundaries only)
        for road in city_data['roads']:
            self.add_road_boundaries(road)
        
        # Add buildings - FIX: Use object attributes instead of dictionary access
        for building in city_data['buildings']:
            self.add_rectangle_obstacle(
                building.x, building.y, building.width, building.height, 1.0
            )
        
        # Add static obstacles - FIX: Use object attributes instead of dictionary access
        for obstacle in city_data['obstacles']:
            # Determine obstacle value based on type
            if obstacle.obstacle_type == 'building':
                value = 1.0
            elif obstacle.obstacle_type == 'parked_car':
                value = 0.9
            elif obstacle.obstacle_type == 'barrier':
                value = 0.8
            elif obstacle.obstacle_type == 'tree':
                value = 0.6
            elif obstacle.obstacle_type == 'construction':
                value = 0.85
            elif obstacle.obstacle_type == 'pole':
                value = 0.7
            else:
                value = 0.7
            
            self.add_rectangle_obstacle(
                obstacle.x, obstacle.y, obstacle.width, obstacle.height, value
            )
        
        print(f"✅ Grid populated with {len(self._obstacle_cache)} obstacles")
    
    def update_dynamic_obstacles(self, dynamic_obstacles: List[Dict[str, Any]]):
        """Update dynamic obstacles in grid"""
        # This is computationally expensive, so we'll keep it simple
        # In a real implementation, you'd want to track and update only changed areas
        pass
    
    def get_occupancy_at_world(self, x: float, y: float) -> float:
        """Get occupancy value at world coordinates"""
        grid_x, grid_y = self.world_to_grid(x, y)
        
        if self.is_valid_grid_pos(grid_x, grid_y):
            return self.grid[grid_y, grid_x]
        else:
            return 1.0  # Treat out-of-bounds as occupied
    
    def get_occupancy_at_grid(self, grid_x: int, grid_y: int) -> float:
        """Get occupancy value at grid coordinates"""
        if self.is_valid_grid_pos(grid_x, grid_y):
            return self.grid[grid_y, grid_x]
        else:
            return 1.0
    
    def is_free_world(self, x: float, y: float, threshold: float = 0.5) -> bool:
        """Check if world position is free"""
        return self.get_occupancy_at_world(x, y) < threshold
    
    def is_free_grid(self, grid_x: int, grid_y: int, threshold: float = 0.5) -> bool:
        """Check if grid position is free"""
        return self.get_occupancy_at_grid(grid_x, grid_y) < threshold
    
    def get_data(self) -> np.ndarray:
        """Get grid data for compatibility"""
        return self.grid
    
    def clear(self):
        """Clear the grid"""
        self.grid.fill(0.0)
        self._obstacle_cache.clear()
    
    def get_free_space_around(self, x: float, y: float, radius: float) -> List[Tuple[float, float]]:
        """Get free space positions around a point"""
        free_positions = []
        grid_x, grid_y = self.world_to_grid(x, y)
        grid_radius = int(radius / self.resolution)
        
        for dx in range(-grid_radius, grid_radius + 1):
            for dy in range(-grid_radius, grid_radius + 1):
                check_x, check_y = grid_x + dx, grid_y + dy
                
                if (self.is_valid_grid_pos(check_x, check_y) and
                    self.is_free_grid(check_x, check_y)):
                    world_x, world_y = self.grid_to_world(check_x, check_y)
                    free_positions.append((world_x, world_y))
        
        return free_positions