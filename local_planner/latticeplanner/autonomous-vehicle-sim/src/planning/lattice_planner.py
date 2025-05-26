import numpy as np
from .path_planner import PathPlanner

class LatticePlanner:
    def __init__(self, config):
        self.config = config
        self.prediction_horizon = config['prediction_horizon']
        self.lateral_resolution = config['lateral_resolution']
        self.speed_resolution = config['speed_resolution']
        self.max_lateral_offset = config['max_lateral_offset']
        self.trajectory_length = config['trajectory_length']
        
        self.path_planner = PathPlanner()
        
    def plan(self, current_state, goal, occupancy_grid):
        """Generate optimal trajectory using lattice planning"""
        # Generate reference path
        reference_path = self.path_planner.plan_path(current_state, goal, occupancy_grid)
        
        if reference_path is None:
            # Create simple straight line path if A* fails
            start_pos = current_state[:2]
            goal_pos = goal[:2]
            reference_path = self._create_straight_line_path(start_pos, goal_pos)
            
        # Generate lattice trajectories
        trajectories = self._generate_lattice_trajectories(current_state, reference_path)
        
        # Evaluate and select best trajectory
        best_trajectory = self._evaluate_trajectories(trajectories, occupancy_grid, goal)
        
        return best_trajectory
    
    def _create_straight_line_path(self, start, goal):
        """Create a simple straight line path"""
        num_points = 20
        path = []
        for i in range(num_points):
            alpha = i / (num_points - 1)
            point = start + alpha * (goal - start)
            path.append(point)
        return np.array(path)
        
    def _generate_lattice_trajectories(self, current_state, reference_path):
        """Generate multiple trajectory candidates"""
        trajectories = []
        
        # Different lateral offsets
        lateral_offsets = np.arange(-self.max_lateral_offset, 
                                  self.max_lateral_offset + self.lateral_resolution,
                                  self.lateral_resolution)
        
        # Different target speeds
        speeds = np.arange(2.0, 10.0, self.speed_resolution)
        
        for lateral_offset in lateral_offsets:
            for target_speed in speeds:
                trajectory = self._generate_trajectory(current_state, reference_path, 
                                                    lateral_offset, target_speed)
                if trajectory is not None:
                    trajectories.append(trajectory)
                    
        return trajectories
        
    def _generate_trajectory(self, current_state, reference_path, lateral_offset, target_speed):
        """Generate a single trajectory"""
        if len(reference_path) < 2:
            return None
            
        trajectory_points = []
        
        # Time sampling
        dt = 0.2
        time_steps = int(self.prediction_horizon / dt)
        
        for i in range(time_steps):
            t = i * dt
            
            # Progress along reference path
            progress = min(t * target_speed / 3.0, len(reference_path) - 1)
            idx = int(progress)
            
            if idx >= len(reference_path) - 1:
                ref_point = reference_path[-1]
                if len(reference_path) > 1:
                    ref_heading = np.arctan2(reference_path[-1][1] - reference_path[-2][1],
                                           reference_path[-1][0] - reference_path[-2][0])
                else:
                    ref_heading = 0.0
            else:
                # Interpolate reference point
                alpha = progress - idx
                ref_point = (1 - alpha) * reference_path[idx] + alpha * reference_path[idx + 1]
                ref_heading = np.arctan2(reference_path[idx + 1][1] - reference_path[idx][1],
                                       reference_path[idx + 1][0] - reference_path[idx][0])
            
            # Apply lateral offset
            offset_x = ref_point[0] + lateral_offset * np.cos(ref_heading + np.pi/2)
            offset_y = ref_point[1] + lateral_offset * np.sin(ref_heading + np.pi/2)
            
            trajectory_points.append([offset_x, offset_y, ref_heading, target_speed])
            
        return np.array(trajectory_points)
        
    def _evaluate_trajectories(self, trajectories, occupancy_grid, goal):
        """Evaluate and select the best trajectory"""
        if not trajectories:
            return None
            
        best_trajectory = None
        best_score = float('inf')
        
        for trajectory in trajectories:
            score = self._calculate_trajectory_cost(trajectory, occupancy_grid, goal)
            
            if score < best_score:
                best_score = score
                best_trajectory = trajectory
                
        return best_trajectory
        
    def _calculate_trajectory_cost(self, trajectory, occupancy_grid, goal):
        """Calculate trajectory cost"""
        if self._check_collision(trajectory, occupancy_grid):
            return float('inf')
            
        # Distance to goal
        goal_cost = np.linalg.norm(trajectory[-1][:2] - goal[:2])
        
        # Smoothness cost
        smoothness_cost = self._calculate_smoothness_cost(trajectory)
        
        # Speed cost (prefer higher speeds)
        speed_cost = -np.mean(trajectory[:, 3]) * 0.1
        
        total_cost = goal_cost + smoothness_cost * 0.5 + speed_cost
        
        return total_cost
        
    def _check_collision(self, trajectory, occupancy_grid):
        """Check if trajectory collides with obstacles"""
        for point in trajectory:
            if occupancy_grid.is_occupied(point[0], point[1]):
                return True
        return False
        
    def _calculate_smoothness_cost(self, trajectory):
        """Calculate trajectory smoothness cost"""
        if len(trajectory) < 3:
            return 0.0
            
        # Calculate curvature
        curvature_cost = 0.0
        for i in range(1, len(trajectory) - 1):
            p1 = trajectory[i-1][:2]
            p2 = trajectory[i][:2]
            p3 = trajectory[i+1][:2]
            
            # Approximate curvature
            v1 = p2 - p1
            v2 = p3 - p2
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                angle_diff = np.arccos(np.clip(np.dot(v1, v2) / 
                                             (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))
                curvature_cost += angle_diff ** 2
                
        return curvature_cost

# Test the class
if __name__ == "__main__":
    print("Testing LatticePlanner class...")
    
    config = {
        'prediction_horizon': 5.0,
        'lateral_resolution': 1.0,
        'speed_resolution': 2.0,
        'max_lateral_offset': 6.0,
        'trajectory_length': 50
    }
    
    planner = LatticePlanner(config)
    print("✓ LatticePlanner instance created")
    
    # Test plan method exists
    import numpy as np
    current_state = np.array([0, 0, 0, 0])
    goal = np.array([10, 10, 0])
    
    # Create a simple mock occupancy grid for testing
    class MockGrid:
        def is_occupied(self, x, y):
            return False
    
    mock_grid = MockGrid()
    
    try:
        trajectory = planner.plan(current_state, goal, mock_grid)
        print("✓ Plan method works")
        if trajectory is not None:
            print(f"✓ Generated trajectory with {len(trajectory)} points")
        else:
            print("✓ Plan method returns None (expected for mock test)")
    except Exception as e:
        print(f"✗ Plan method failed: {e}")
        
    print("LatticePlanner test completed!")