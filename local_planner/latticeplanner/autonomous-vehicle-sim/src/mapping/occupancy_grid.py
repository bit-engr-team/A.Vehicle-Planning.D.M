import numpy as np

class OccupancyGrid:
    def __init__(self, config=None, width=200, height=200, resolution=0.5, origin_x=-50.0, origin_y=-50.0):
        """Initialize occupancy grid with flexible parameter input"""
        
        if config is not None:
            # Initialize from config dictionary
            self.width = config.get('width', width)
            self.height = config.get('height', height)
            self.resolution = config.get('resolution', resolution)
            self.origin_x = config.get('origin_x', origin_x)
            self.origin_y = config.get('origin_y', origin_y)
        else:
            # Initialize from individual parameters
            self.width = width
            self.height = height
            self.resolution = resolution
            self.origin_x = origin_x
            self.origin_y = origin_y
        
        # Initialize grid (0 = free, 1 = occupied, 0.5 = unknown)
        self.grid = np.zeros((self.height, self.width), dtype=np.float32)
        
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices"""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y
        
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates"""
        world_x = grid_x * self.resolution + self.origin_x
        world_y = grid_y * self.resolution + self.origin_y
        return world_x, world_y
        
    def is_valid_grid(self, grid_x, grid_y):
        """Check if grid coordinates are valid"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
        
    def is_occupied(self, x, y):
        """Check if world position is occupied"""
        grid_x, grid_y = self.world_to_grid(x, y)
        
        if not self.is_valid_grid(grid_x, grid_y):
            return True  # Consider out-of-bounds as occupied
            
        return self.grid[grid_y, grid_x] > 0.5
        
    def set_occupied(self, x, y):
        """Set world position as occupied"""
        grid_x, grid_y = self.world_to_grid(x, y)
        
        if self.is_valid_grid(grid_x, grid_y):
            self.grid[grid_y, grid_x] = 1.0
            
    def set_free(self, x, y):
        """Set world position as free"""
        grid_x, grid_y = self.world_to_grid(x, y)
        
        if self.is_valid_grid(grid_x, grid_y):
            self.grid[grid_y, grid_x] = 0.0
            
    def add_rectangle_obstacle(self, center_x, center_y, width, height):
        """Add rectangular obstacle"""
        half_width = width / 2.0
        half_height = height / 2.0
        
        # Get grid bounds
        min_x = center_x - half_width
        max_x = center_x + half_width
        min_y = center_y - half_height
        max_y = center_y + half_height
        
        # Convert to grid coordinates
        min_grid_x, min_grid_y = self.world_to_grid(min_x, min_y)
        max_grid_x, max_grid_y = self.world_to_grid(max_x, max_y)
        
        # Set occupied cells
        for grid_y in range(max(0, min_grid_y), min(self.height, max_grid_y + 1)):
            for grid_x in range(max(0, min_grid_x), min(self.width, max_grid_x + 1)):
                self.grid[grid_y, grid_x] = 1.0
                
    def add_circle_obstacle(self, center_x, center_y, radius):
        """Add circular obstacle"""
        # Get grid bounds
        min_grid_x, min_grid_y = self.world_to_grid(center_x - radius, center_y - radius)
        max_grid_x, max_grid_y = self.world_to_grid(center_x + radius, center_y + radius)
        
        # Set occupied cells within radius
        for grid_y in range(max(0, min_grid_y), min(self.height, max_grid_y + 1)):
            for grid_x in range(max(0, min_grid_x), min(self.width, max_grid_x + 1)):
                world_x, world_y = self.grid_to_world(grid_x, grid_y)
                distance = np.sqrt((world_x - center_x)**2 + (world_y - center_y)**2)
                
                if distance <= radius:
                    self.grid[grid_y, grid_x] = 1.0

    def get_grid_coordinates(self):
        """Get coordinate arrays for visualization"""
        x = np.arange(self.width) * self.resolution + self.origin_x
        y = np.arange(self.height) * self.resolution + self.origin_y
        return np.meshgrid(x, y)
        
    def get_heights(self):
        """Get height array for 2.5D visualization"""
        return self.grid * 2.0  # Scale height for better visualization

# Test the class
if __name__ == "__main__":
    print("Testing OccupancyGrid class...")
    
    # Test with config dictionary
    config = {
        'width': 100,
        'height': 100,
        'resolution': 0.5,
        'origin_x': -25.0,
        'origin_y': -25.0
    }
    
    grid = OccupancyGrid(config)
    print(f"✓ OccupancyGrid created with config: {grid.width}x{grid.height}")
    
    # Test with individual parameters
    grid2 = OccupancyGrid(width=50, height=50, resolution=1.0)
    print(f"✓ OccupancyGrid created with parameters: {grid2.width}x{grid2.height}")
    
    # Test adding obstacles
    grid.add_rectangle_obstacle(0, 0, 5, 5)
    grid.add_circle_obstacle(10, 10, 3)
    print("✓ Obstacles added successfully")
    
    print("OccupancyGrid test completed successfully!")