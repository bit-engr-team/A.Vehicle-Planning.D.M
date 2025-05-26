import numpy as np
import matplotlib.pyplot as plt
from .path_planner import PathPlanner

class AdvancedLatticePlanner:
    def __init__(self, config):
        self.config = config
        self.prediction_horizon = config['prediction_horizon']
        self.lateral_resolution = config['lateral_resolution']
        self.speed_resolution = config['speed_resolution']
        self.max_lateral_offset = config['max_lateral_offset']
        self.trajectory_length = config['trajectory_length']
        
        self.path_planner = PathPlanner()
        self.all_trajectories = []  # Store all generated trajectories for visualization
        
    def plan(self, current_state, goal, occupancy_grid, dynamic_obstacles=None):
        """Generate optimal trajectory using advanced lattice planning"""
        # Generate reference path
        reference_path = self.path_planner.plan_path(current_state, goal, occupancy_grid)
        
        if reference_path is None:
            # Create road-following path
            reference_path = self._create_road_path(current_state, goal)
            
        # Generate lattice trajectories with branches
        self.all_trajectories = self._generate_advanced_lattice(current_state, reference_path)
        
        # Evaluate trajectories considering dynamic obstacles
        best_trajectory = self._evaluate_trajectories_dynamic(
            self.all_trajectories, occupancy_grid, goal, dynamic_obstacles)
        
        return best_trajectory
    
    def _create_road_path(self, current_state, goal):
        """Create a road-like path with curves"""
        start_pos = current_state[:2]
        goal_pos = goal[:2]
        
        # Create a curved path that simulates a road
        total_distance = np.linalg.norm(goal_pos - start_pos)
        num_points = max(20, int(total_distance / 2))
        
        path_points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # Main direction
            main_point = start_pos + t * (goal_pos - start_pos)
            
            # Add some curves to simulate a real road
            curve_amplitude = 5.0
            direction_angle = np.arctan2(goal_pos[1]-start_pos[1], goal_pos[0]-start_pos[0])
            curve1 = curve_amplitude * np.sin(t * np.pi * 2) * np.array([-np.sin(direction_angle), np.cos(direction_angle)])
            curve2 = curve_amplitude * 0.5 * np.sin(t * np.pi * 4) * np.array([np.cos(direction_angle), np.sin(direction_angle)])
            
            curved_point = main_point + curve1 + curve2
            path_points.append(curved_point)
            
        return np.array(path_points)
    
    def _generate_advanced_lattice(self, current_state, reference_path):
        """Generate lattice trajectories with branching structure"""
        all_trajectories = []
        
        # Multiple branching levels
        branch_levels = [0.0, 1.0, 2.0, 3.0]  # Time points for branching
        
        for branch_time in branch_levels:
            trajectories_at_level = self._generate_branch_trajectories(
                current_state, reference_path, branch_time)
            all_trajectories.extend(trajectories_at_level)
            
        return all_trajectories
    
    def _generate_branch_trajectories(self, current_state, reference_path, branch_start_time):
        """Generate trajectories that branch at a specific time"""
        trajectories = []
        
        # Different lateral offsets for branching
        lateral_offsets = np.arange(-self.max_lateral_offset, 
                                  self.max_lateral_offset + self.lateral_resolution,
                                  self.lateral_resolution)
        
        # Different speeds
        speeds = np.arange(3.0, 12.0, self.speed_resolution)
        
        for lateral_offset in lateral_offsets:
            for target_speed in speeds:
                # Generate lane change maneuvers
                for lane_change_type in ['maintain', 'left_change', 'right_change']:
                    trajectory = self._generate_branched_trajectory(
                        current_state, reference_path, lateral_offset, 
                        target_speed, branch_start_time, lane_change_type)
                    
                    if trajectory is not None:
                        trajectories.append({
                            'trajectory': trajectory,
                            'lateral_offset': lateral_offset,
                            'speed': target_speed,
                            'branch_time': branch_start_time,
                            'maneuver': lane_change_type
                        })
                        
        return trajectories
    
    def _generate_branched_trajectory(self, current_state, reference_path, lateral_offset, 
                                    target_speed, branch_start_time, maneuver_type):
        """Generate a single branched trajectory"""
        if len(reference_path) < 2:
            return None
            
        trajectory_points = []
        dt = 0.1
        time_steps = int(self.prediction_horizon / dt)
        
        for i in range(time_steps):
            t = i * dt
            
            # Progress along reference path
            progress = min(t * target_speed / 4.0, len(reference_path) - 1)
            idx = int(progress)
            
            if idx >= len(reference_path) - 1:
                ref_point = reference_path[-1]
                if len(reference_path) > 1:
                    ref_heading = np.arctan2(reference_path[-1][1] - reference_path[-2][1],
                                           reference_path[-1][0] - reference_path[-2][0])
                else:
                    ref_heading = 0.0
            else:
                alpha = progress - idx
                ref_point = (1 - alpha) * reference_path[idx] + alpha * reference_path[idx + 1]
                ref_heading = np.arctan2(reference_path[idx + 1][1] - reference_path[idx][1],
                                       reference_path[idx + 1][0] - reference_path[idx][0])
            
            # Apply dynamic lateral offset based on maneuver
            dynamic_lateral_offset = self._calculate_dynamic_offset(
                t, lateral_offset, branch_start_time, maneuver_type)
            
            # Calculate position with offset
            offset_x = ref_point[0] + dynamic_lateral_offset * np.cos(ref_heading + np.pi/2)
            offset_y = ref_point[1] + dynamic_lateral_offset * np.sin(ref_heading + np.pi/2)
            
            trajectory_points.append([offset_x, offset_y, ref_heading, target_speed])
            
        return np.array(trajectory_points)
    
    def _calculate_dynamic_offset(self, t, base_offset, branch_time, maneuver_type):
        """Calculate dynamic lateral offset for different maneuvers"""
        if maneuver_type == 'maintain':
            return base_offset
        elif maneuver_type == 'left_change':
            if t < branch_time:
                return base_offset
            else:
                # Smooth transition to left lane
                transition_progress = min((t - branch_time) / 2.0, 1.0)
                return base_offset + 3.5 * transition_progress  # 3.5m lane width
        elif maneuver_type == 'right_change':
            if t < branch_time:
                return base_offset
            else:
                # Smooth transition to right lane
                transition_progress = min((t - branch_time) / 2.0, 1.0)
                return base_offset - 3.5 * transition_progress
        
        return base_offset
    
    def _evaluate_trajectories_dynamic(self, trajectory_data, occupancy_grid, goal, dynamic_obstacles):
        """Evaluate trajectories considering dynamic obstacles"""
        if not trajectory_data:
            return None
            
        best_trajectory = None
        best_score = float('inf')
        
        for traj_data in trajectory_data:
            trajectory = traj_data['trajectory']
            score = self._calculate_dynamic_cost(trajectory, occupancy_grid, goal, 
                                               dynamic_obstacles, traj_data)
            
            if score < best_score:
                best_score = score
                best_trajectory = trajectory
                
        return best_trajectory
    
    def _calculate_dynamic_cost(self, trajectory, occupancy_grid, goal, dynamic_obstacles, traj_data):
        """Calculate trajectory cost with dynamic obstacles"""
        # Static collision check
        if self._check_static_collision(trajectory, occupancy_grid):
            return float('inf')
        
        # Dynamic collision check
        if dynamic_obstacles and self._check_dynamic_collision(trajectory, dynamic_obstacles):
            return float('inf') * 0.9  # Slightly less penalty to allow for prediction uncertainty
        
        # Goal distance cost
        goal_cost = np.linalg.norm(trajectory[-1][:2] - goal[:2])
        
        # Smoothness cost
        smoothness_cost = self._calculate_smoothness_cost(trajectory)
        
        # Speed cost (prefer higher speeds)
        speed_cost = -np.mean(trajectory[:, 3]) * 0.1
        
        # Maneuver cost (prefer maintaining lane)
        maneuver_cost = 0.0
        if traj_data['maneuver'] == 'left_change':
            maneuver_cost = 5.0
        elif traj_data['maneuver'] == 'right_change':
            maneuver_cost = 5.0
        
        # Comfort cost (penalize large lateral offsets)
        comfort_cost = abs(traj_data['lateral_offset']) * 2.0
        
        total_cost = (goal_cost + smoothness_cost * 0.3 + speed_cost + 
                     maneuver_cost + comfort_cost)
        
        return total_cost
    
    def _check_static_collision(self, trajectory, occupancy_grid):
        """Check collision with static obstacles"""
        for point in trajectory:
            if occupancy_grid.is_occupied(point[0], point[1]):
                return True
        return False
    
    def _check_dynamic_collision(self, trajectory, dynamic_obstacles):
        """Check collision with dynamic obstacles"""
        dt = 0.1
        
        for i, point in enumerate(trajectory):
            t = i * dt
            
            for obstacle in dynamic_obstacles:
                # Predict obstacle position at time t
                future_x = obstacle.x + obstacle.vx * t
                future_y = obstacle.y + obstacle.vy * t
                
                # Check collision
                distance = np.sqrt((point[0] - future_x)**2 + (point[1] - future_y)**2)
                if distance < (obstacle.radius + 2.0):  # 2m safety margin
                    return True
                    
        return False
    
    def _calculate_smoothness_cost(self, trajectory):
        """Calculate trajectory smoothness cost"""
        if len(trajectory) < 3:
            return 0.0
            
        curvature_cost = 0.0
        for i in range(1, len(trajectory) - 1):
            p1 = trajectory[i-1][:2]
            p2 = trajectory[i][:2]
            p3 = trajectory[i+1][:2]
            
            v1 = p2 - p1
            v2 = p3 - p2
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                angle_diff = np.arccos(np.clip(np.dot(v1, v2) / 
                                             (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))
                curvature_cost += angle_diff ** 2
                
        return curvature_cost
    
    def get_all_trajectories(self):
        """Get all generated trajectories for visualization"""
        return self.all_trajectories