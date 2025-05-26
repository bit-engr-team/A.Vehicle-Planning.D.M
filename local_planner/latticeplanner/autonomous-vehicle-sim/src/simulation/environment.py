import numpy as np


class Environment:
    def __init__(self, occupancy_grid):
        """Initialize environment with occupancy grid"""
        self.occupancy_grid = occupancy_grid
        self.dynamic_obstacles = []

    def add_dynamic_obstacle(self, obstacle):
        """Add a dynamic obstacle to the environment"""
        self.dynamic_obstacles.append(obstacle)

    def remove_dynamic_obstacle(self, obstacle):
        """Remove a dynamic obstacle from the environment"""
        if obstacle in self.dynamic_obstacles:
            self.dynamic_obstacles.remove(obstacle)

    def update(self, dt):
        """Update dynamic obstacles"""
        for obstacle in self.dynamic_obstacles:
            if hasattr(obstacle, "update"):
                obstacle.update(dt)

    def get_occupancy_at(self, x, y):
        """Get occupancy value at world coordinates"""
        return self.occupancy_grid.is_occupied(x, y)

    def is_collision_free(self, trajectory):
        """Check if trajectory is collision free"""
        for point in trajectory:
            if self.occupancy_grid.is_occupied(point[0], point[1]):
                return False
        return True

    def get_nearest_obstacle(self, x, y, search_radius=10.0):
        """Find nearest obstacle within search radius"""
        min_distance = float("inf")
        nearest_obstacle = None

        # Search in grid around position
        grid_x, grid_y = self.occupancy_grid.world_to_grid(x, y)
        search_grid_radius = int(search_radius / self.occupancy_grid.resolution)

        for dy in range(-search_grid_radius, search_grid_radius + 1):
            for dx in range(-search_grid_radius, search_grid_radius + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy

                if self.occupancy_grid.is_valid_grid(check_x, check_y):
                    if self.occupancy_grid.grid[check_y, check_x] > 0.5:
                        world_x, world_y = self.occupancy_grid.grid_to_world(
                            check_x, check_y
                        )
                        distance = np.sqrt((world_x - x) ** 2 + (world_y - y) ** 2)

                        if distance < min_distance:
                            min_distance = distance
                            nearest_obstacle = (world_x, world_y)

        return nearest_obstacle, min_distance if nearest_obstacle else None


# Test the class
if __name__ == "__main__":
    print("Testing Environment class...")

    # Create a simple mock occupancy grid
    class MockGrid:
        def __init__(self):
            self.resolution = 0.5
            self.origin_x = -50
            self.origin_y = -50
            self.width = 100
            self.height = 100

        def is_occupied(self, x, y):
            return False

        def world_to_grid(self, x, y):
            return (
                int((x - self.origin_x) / self.resolution),
                int((y - self.origin_y) / self.resolution),
            )

        def is_valid_grid(self, x, y):
            return 0 <= x < self.width and 0 <= y < self.height

    mock_grid = MockGrid()
    env = Environment(mock_grid)
    print("✓ Environment instance created")

    print("Environment test completed!")