import numpy as np
import random
from typing import List, Dict, Any, Tuple
import time

class DynamicObstacle:
    """Enhanced dynamic obstacle with realistic behavior"""
    
    def __init__(self, obstacle_data: Dict[str, Any]):
        # Basic properties
        self.x = obstacle_data['x']
        self.y = obstacle_data['y']
        self.width = obstacle_data['width']
        self.height = obstacle_data['height']
        self.vx = obstacle_data['vx']
        self.vy = obstacle_data['vy']
        self.obstacle_type = obstacle_data['type']
        self.behavior = obstacle_data['behavior']
        self.speed = obstacle_data['speed']
        
        # Initial position for boundary management
        self.initial_pos = obstacle_data['initial_pos']
        self.max_distance_from_initial = 20.0
        
        # Behavior state
        self.behavior_timer = 0.0
        self.behavior_state = "moving"  # "moving", "stopped", "turning"
        self.stop_duration = 0.0
        self.target_direction = None
        
        # Collision properties
        self.radius = max(self.width, self.height) / 2.0
        
        # Path following (for cyclists and vehicles)
        self.path_points = []
        self.current_path_index = 0
        
        # Random behavior parameters
        self.direction_change_probability = 0.02  # Per update
        self.stop_probability = 0.01 if self.obstacle_type != 'pedestrian' else 0.05
        
        # Initialize behavior-specific parameters
        self._initialize_behavior()
        
        print(f"🚶 Created {self.obstacle_type} at ({self.x:.1f}, {self.y:.1f})")
    
    def _initialize_behavior(self):
        """Initialize behavior-specific parameters"""
        if self.behavior == "random_walk":
            # Pedestrians - random walk
            self.direction_change_probability = 0.03
            self.stop_probability = 0.02
            
        elif self.behavior == "follow_path":
            # Cyclists - follow predefined paths
            self._generate_path()
            
        elif self.behavior == "road_following":
            # Vehicles - follow road lanes
            self.direction_change_probability = 0.005
            self.stop_probability = 0.001
            
        elif self.behavior == "stop_and_go":
            # Service vehicles - stop periodically
            self.stop_probability = 0.02
            self.direction_change_probability = 0.01
    
    def _generate_path(self):
        """Generate path for path-following behavior"""
        if self.behavior != "follow_path":
            return
        
        # Generate a simple path (could be more sophisticated)
        num_points = random.randint(3, 8)
        path_radius = random.uniform(10, 25)
        
        for i in range(num_points):
            angle = (2 * np.pi * i) / num_points + random.uniform(-0.5, 0.5)
            px = self.initial_pos[0] + path_radius * np.cos(angle)
            py = self.initial_pos[1] + path_radius * np.sin(angle)
            self.path_points.append((px, py))
    
    def update(self, dt: float):
        """Update obstacle position and behavior"""
        self.behavior_timer += dt
        
        # Update behavior state
        self._update_behavior_state(dt)
        
        # Update velocity based on behavior
        self._update_velocity(dt)
        
        # Update position
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # Handle boundaries
        self._handle_boundaries()
    
    def _update_behavior_state(self, dt: float):
        """Update behavior state machine"""
        
        if self.behavior_state == "stopped":
            self.stop_duration += dt
            if self.stop_duration > random.uniform(1.0, 4.0):
                self.behavior_state = "moving"
                self.stop_duration = 0.0
                # Choose new direction when resuming
                self._choose_new_direction()
            return
        
        # Random stopping
        if random.random() < self.stop_probability * dt:
            self.behavior_state = "stopped"
            self.vx = 0.0
            self.vy = 0.0
            return
        
        # Random direction changes
        if random.random() < self.direction_change_probability * dt:
            self.behavior_state = "turning"
            self._choose_new_direction()
    
    def _update_velocity(self, dt: float):
        """Update velocity based on behavior and state"""
        
        if self.behavior_state == "stopped":
            self.vx = 0.0
            self.vy = 0.0
            return
        
        if self.behavior == "random_walk":
            self._update_random_walk_velocity()
            
        elif self.behavior == "follow_path":
            self._update_path_following_velocity()
            
        elif self.behavior == "road_following":
            self._update_road_following_velocity()
            
        elif self.behavior == "stop_and_go":
            self._update_stop_and_go_velocity()
    
    def _update_random_walk_velocity(self):
        """Update velocity for random walk behavior"""
        # Gradually change direction
        if random.random() < 0.1:  # 10% chance per update
            angle_change = random.uniform(-0.3, 0.3)
            current_angle = np.arctan2(self.vy, self.vx)
            new_angle = current_angle + angle_change
            
            speed = np.linalg.norm([self.vx, self.vy])
            if speed < 0.1:
                speed = self.speed * random.uniform(0.5, 1.0)
            
            self.vx = speed * np.cos(new_angle)
            self.vy = speed * np.sin(new_angle)
    
    def _update_path_following_velocity(self):
        """Update velocity for path following behavior"""
        if not self.path_points:
            self._update_random_walk_velocity()
            return
        
        # Move towards current target point
        target = self.path_points[self.current_path_index]
        target_x, target_y = target
        
        # Vector to target
        dx = target_x - self.x
        dy = target_y - self.y
        distance_to_target = np.linalg.norm([dx, dy])
        
        # If close to target, move to next point
        if distance_to_target < 2.0:
            self.current_path_index = (self.current_path_index + 1) % len(self.path_points)
            target = self.path_points[self.current_path_index]
            target_x, target_y = target
            dx = target_x - self.x
            dy = target_y - self.y
            distance_to_target = np.linalg.norm([dx, dy])
        
        # Update velocity towards target
        if distance_to_target > 0.1:
            self.vx = self.speed * dx / distance_to_target
            self.vy = self.speed * dy / distance_to_target
    
    def _update_road_following_velocity(self):
        """Update velocity for road following behavior"""
        # Maintain current direction mostly, with small random variations
        if random.random() < 0.02:  # 2% chance of small direction change
            current_angle = np.arctan2(self.vy, self.vx)
            angle_change = random.uniform(-0.1, 0.1)
            new_angle = current_angle + angle_change
            
            self.vx = self.speed * np.cos(new_angle)
            self.vy = self.speed * np.sin(new_angle)
    
    def _update_stop_and_go_velocity(self):
        """Update velocity for stop-and-go behavior"""
        # Service vehicles that stop periodically
        if self.behavior_state == "moving":
            # Move in current direction
            current_speed = np.linalg.norm([self.vx, self.vy])
            if current_speed < 0.1:
                self._choose_new_direction()
        
        # Occasionally change direction when moving
        if random.random() < 0.005:  # 0.5% chance
            self._choose_new_direction()
    
    def _choose_new_direction(self):
        """Choose a new random direction"""
        angle = random.uniform(0, 2 * np.pi)
        speed_factor = random.uniform(0.5, 1.2)
        actual_speed = self.speed * speed_factor
        
        self.vx = actual_speed * np.cos(angle)
        self.vy = actual_speed * np.sin(angle)
    
    def _handle_boundaries(self):
        """Handle obstacle boundaries and containment"""
        # Distance from initial position
        distance_from_initial = np.linalg.norm([
            self.x - self.initial_pos[0],
            self.y - self.initial_pos[1]
        ])
        
        # If too far from initial position, turn back
        if distance_from_initial > self.max_distance_from_initial:
            # Vector back to initial position
            back_x = self.initial_pos[0] - self.x
            back_y = self.initial_pos[1] - self.y
            back_distance = np.linalg.norm([back_x, back_y])
            
            if back_distance > 0.1:
                # Set velocity towards initial position
                self.vx = self.speed * back_x / back_distance
                self.vy = self.speed * back_y / back_distance
    
    def get_position(self) -> Tuple[float, float]:
        """Get current position"""
        return self.x, self.y
    
    def get_velocity(self) -> Tuple[float, float]:
        """Get current velocity"""
        return self.vx, self.vy
    
    def get_bounds(self) -> Dict[str, float]:
        """Get bounding box"""
        return {
            'x_min': self.x - self.width / 2,
            'x_max': self.x + self.width / 2,
            'y_min': self.y - self.height / 2,
            'y_max': self.y + self.height / 2
        }
    
    def is_collision_with_point(self, point: Tuple[float, float], safety_margin: float = 0.5) -> bool:
        """Check collision with a point"""
        distance = np.linalg.norm([point[0] - self.x, point[1] - self.y])
        return distance < (self.radius + safety_margin)
    
    def predict_position(self, time_horizon: float) -> Tuple[float, float]:
        """Predict future position"""
        future_x = self.x + self.vx * time_horizon
        future_y = self.y + self.vy * time_horizon
        return future_x, future_y
    
    def get_render_data(self) -> Dict[str, Any]:
        """Get data for rendering"""
        return {
            'x': self.x,
            'y': self.y,
            'width': self.width,
            'height': self.height,
            'vx': self.vx,
            'vy': self.vy,
            'type': self.obstacle_type,
            'radius': self.radius,
            'behavior_state': self.behavior_state
        }

class DynamicObstacleManager:
    """Manager for all dynamic obstacles"""
    
    def __init__(self):
        self.obstacles: List[DynamicObstacle] = []
        self.collision_grid = {}  # Simple spatial hashing for performance
        self.grid_size = 5.0  # Grid cell size for spatial hashing
        
    def add_obstacles_from_scenario(self, dynamic_obstacle_data: List[Dict[str, Any]]):
        """Add obstacles from scenario data"""
        self.obstacles.clear()
        
        for obs_data in dynamic_obstacle_data:
            obstacle = DynamicObstacle(obs_data)
            self.obstacles.append(obstacle)
        
        print(f"✅ Added {len(self.obstacles)} dynamic obstacles")
    
    def update_all(self, dt: float):
        """Update all dynamic obstacles"""
        for obstacle in self.obstacles:
            obstacle.update(dt)
        
        # Update spatial hash
        self._update_spatial_hash()
    
    def _update_spatial_hash(self):
        """Update spatial hash for collision detection"""
        self.collision_grid.clear()
        
        for i, obstacle in enumerate(self.obstacles):
            grid_x = int(obstacle.x / self.grid_size)
            grid_y = int(obstacle.y / self.grid_size)
            
            if (grid_x, grid_y) not in self.collision_grid:
                self.collision_grid[(grid_x, grid_y)] = []
            
            self.collision_grid[(grid_x, grid_y)].append(i)
    
    def get_obstacles_near(self, x: float, y: float, radius: float) -> List[DynamicObstacle]:
        """Get obstacles near a position"""
        nearby_obstacles = []
        
        # Simple radius check (could use spatial hash for better performance)
        for obstacle in self.obstacles:
            distance = np.linalg.norm([obstacle.x - x, obstacle.y - y])
            if distance < radius:
                nearby_obstacles.append(obstacle)
        
        return nearby_obstacles
    
    def check_collision_with_trajectory(self, trajectory: np.ndarray, 
                                      safety_margin: float = 2.0) -> bool:
        """Check if trajectory collides with any dynamic obstacle"""
        if len(trajectory) == 0:
            return False
        
        # Check collision for each trajectory point
        for point in trajectory:
            x, y = point[0], point[1]
            
            for obstacle in self.obstacles:
                if obstacle.is_collision_with_point((x, y), safety_margin):
                    return True
        
        return False
    
    def get_all_positions(self) -> List[Tuple[float, float]]:
        """Get positions of all obstacles"""
        return [obs.get_position() for obs in self.obstacles]
    
    def get_render_data(self) -> List[Dict[str, Any]]:
        """Get render data for all obstacles"""
        return [obs.get_render_data() for obs in self.obstacles]
    
    def clear(self):
        """Clear all obstacles"""
        self.obstacles.clear()
        self.collision_grid.clear()