import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, Dict, Any
import math
import random

class LatticePlanner:
    """Improved Lattice-based path planner"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.prediction_horizon = config.get('prediction_horizon', 4.0)
        self.lateral_resolution = config.get('lateral_resolution', 1.5)  # Daha küçük (2.5 -> 1.5)
        self.speed_resolution = config.get('speed_resolution', 1.5)      # Daha küçük (2.0 -> 1.5)
        self.max_lateral_offset = config.get('max_lateral_offset', 4.0)  # Daha küçük (5.0 -> 4.0)
        self.trajectory_length = config.get('trajectory_length', 25)     # Daha uzun (20 -> 25)
        
        # Planning parameters - IMPROVED
        self.max_curvature = 0.3      # Daha yumuşak virajlar (0.5 -> 0.3)
        self.max_acceleration = 2.0   # Daha yumuşak ivmelenme (3.0 -> 2.0)
        self.safety_margin = 3.5      # Daha büyük güvenlik marjı (2.0 -> 3.5)
        
        self.all_trajectories = []
        self.selected_trajectory = None
        
        print(f"🧠 Improved LatticePlanner initialized:")
        print(f"   📏 Lateral resolution: {self.lateral_resolution}m")
        print(f"   🎯 Max lateral offset: {self.max_lateral_offset}m")
        print(f"   📐 Max curvature: {self.max_curvature}")
    
    def plan(self, current_state: np.ndarray, goal: np.ndarray, 
             occupancy_grid: Any) -> Optional[np.ndarray]:
        """Plan trajectory using improved lattice method"""
        try:
            # Current vehicle position and heading
            x, y, theta, v, _, _ = current_state
            goal_x, goal_y = goal[0], goal[1]
            
            # Distance to goal
            distance_to_goal = np.sqrt((goal_x - x)**2 + (goal_y - y)**2)
            
            # If very close to goal, plan simple approach
            if distance_to_goal < 5.0:
                return self._plan_goal_approach(current_state, goal)
            
            # Generate improved lattice trajectories
            trajectories = self._generate_improved_lattice_trajectories(
                current_state, goal, distance_to_goal
            )
            
            # Evaluate trajectories
            best_trajectory = self._evaluate_trajectories(
                trajectories, occupancy_grid, goal, current_state
            )
            
            # Store for visualization
            self.all_trajectories = trajectories[:8]  # Limit for performance
            self.selected_trajectory = best_trajectory
            
            return best_trajectory
            
        except Exception as e:
            print(f"⚠️ Planning error: {e}")
            return self._emergency_trajectory(current_state)
    
    def _generate_improved_lattice_trajectories(self, current_state: np.ndarray, 
                                              goal: np.ndarray, distance_to_goal: float) -> List[np.ndarray]:
        """Generate improved lattice trajectories"""
        x, y, theta, v, _, _ = current_state
        goal_x, goal_y = goal[0], goal[1]
        
        trajectories = []
        
        # Adaptive parameters based on distance
        if distance_to_goal > 30:
            # Long distance - wider search
            lateral_offsets = np.linspace(-self.max_lateral_offset, self.max_lateral_offset, 7)
            speed_targets = [max(3.0, v-1), v, min(12.0, v+2)]
        else:
            # Short distance - narrower search
            lateral_offsets = np.linspace(-self.max_lateral_offset*0.7, self.max_lateral_offset*0.7, 5)
            speed_targets = [max(2.0, v-0.5), v, min(8.0, v+1)]
        
        # Generate trajectories
        for lateral_offset in lateral_offsets:
            for target_speed in speed_targets:
                trajectory = self._generate_single_trajectory(
                    current_state, goal, lateral_offset, target_speed, distance_to_goal
                )
                if trajectory is not None and len(trajectory) > 3:
                    trajectories.append(trajectory)
        
        # Add direct trajectory to goal
        direct_trajectory = self._generate_direct_trajectory(current_state, goal)
        if direct_trajectory is not None:
            trajectories.append(direct_trajectory)
        
        return trajectories
    
    def _generate_single_trajectory(self, current_state: np.ndarray, goal: np.ndarray,
                                   lateral_offset: float, target_speed: float, 
                                   distance_to_goal: float) -> Optional[np.ndarray]:
        """Generate a single improved trajectory"""
        try:
            x, y, theta, v, _, _ = current_state
            goal_x, goal_y = goal[0], goal[1]
            
            # Calculate trajectory points
            trajectory_points = []
            
            # Adaptive step size based on distance
            if distance_to_goal > 50:
                dt = 0.3
                num_steps = min(self.trajectory_length, int(distance_to_goal / 2))
            else:
                dt = 0.2
                num_steps = self.trajectory_length
            
            # Initial state
            curr_x, curr_y, curr_theta = x, y, theta
            curr_speed = v
            
            for i in range(num_steps):
                # Progress ratio
                progress = i / num_steps
                
                # Target point with lateral offset
                target_progress = min(1.0, progress + 0.1)
                intermediate_x = x + target_progress * (goal_x - x)
                intermediate_y = y + target_progress * (goal_y - y)
                
                # Apply lateral offset perpendicular to goal direction
                goal_direction = np.arctan2(goal_y - y, goal_x - x)
                offset_x = intermediate_x + lateral_offset * np.sin(goal_direction)
                offset_y = intermediate_y - lateral_offset * np.cos(goal_direction)
                
                # Smooth approach to target
                desired_heading = np.arctan2(offset_y - curr_y, offset_x - curr_x)
                
                # Smooth heading change
                heading_diff = self._normalize_angle(desired_heading - curr_theta)
                max_heading_change = self.max_curvature * dt
                
                if abs(heading_diff) > max_heading_change:
                    heading_diff = np.sign(heading_diff) * max_heading_change
                
                curr_theta = self._normalize_angle(curr_theta + heading_diff)
                
                # Smooth speed change
                speed_diff = target_speed - curr_speed
                max_speed_change = self.max_acceleration * dt
                
                if abs(speed_diff) > max_speed_change:
                    speed_diff = np.sign(speed_diff) * max_speed_change
                
                curr_speed = max(1.0, curr_speed + speed_diff)
                
                # Update position
                curr_x += curr_speed * np.cos(curr_theta) * dt
                curr_y += curr_speed * np.sin(curr_theta) * dt
                
                # Add point to trajectory
                trajectory_points.append([curr_x, curr_y, curr_theta, curr_speed])
                
                # Early termination if very close to goal
                if np.sqrt((curr_x - goal_x)**2 + (curr_y - goal_y)**2) < 2.0:
                    break
            
            if len(trajectory_points) < 3:
                return None
            
            return np.array(trajectory_points)
            
        except Exception as e:
            return None
    
    def _generate_direct_trajectory(self, current_state: np.ndarray, 
                                   goal: np.ndarray) -> Optional[np.ndarray]:
        """Generate direct trajectory to goal"""
        try:
            x, y, theta, v, _, _ = current_state
            goal_x, goal_y = goal[0], goal[1]
            
            trajectory_points = []
            dt = 0.2
            
            curr_x, curr_y = x, y
            distance = np.sqrt((goal_x - x)**2 + (goal_y - y)**2)
            
            if distance < 2.0:
                return None
            
            # Direct path with smooth speed control
            num_steps = min(self.trajectory_length, int(distance / 2))
            
            for i in range(num_steps):
                progress = (i + 1) / num_steps
                
                # Interpolate position
                curr_x = x + progress * (goal_x - x)
                curr_y = y + progress * (goal_y - y)
                
                # Calculate heading towards goal
                if i < num_steps - 1:
                    next_progress = (i + 2) / num_steps
                    next_x = x + next_progress * (goal_x - x)
                    next_y = y + next_progress * (goal_y - y)
                    curr_theta = np.arctan2(next_y - curr_y, next_x - curr_x)
                else:
                    curr_theta = np.arctan2(goal_y - curr_y, goal_x - curr_x)
                
                # Adaptive speed - slow down near goal
                remaining_distance = distance * (1 - progress)
                if remaining_distance < 10.0:
                    curr_speed = max(2.0, v * 0.5)
                else:
                    curr_speed = min(v + 1, 8.0)
                
                trajectory_points.append([curr_x, curr_y, curr_theta, curr_speed])
            
            return np.array(trajectory_points) if len(trajectory_points) > 0 else None
            
        except Exception as e:
            return None
    
    def _plan_goal_approach(self, current_state: np.ndarray, 
                           goal: np.ndarray) -> Optional[np.ndarray]:
        """Plan final approach to goal"""
        try:
            x, y, theta, v, _, _ = current_state
            goal_x, goal_y = goal[0], goal[1]
            
            trajectory_points = []
            dt = 0.15
            
            # Slow approach
            num_steps = 15
            curr_x, curr_y = x, y
            
            for i in range(num_steps):
                progress = (i + 1) / num_steps
                
                # Smooth approach to goal
                curr_x = x + progress * (goal_x - x)
                curr_y = y + progress * (goal_y - y)
                curr_theta = np.arctan2(goal_y - curr_y, goal_x - curr_x)
                
                # Slow down as approaching
                curr_speed = max(1.0, v * (1 - progress * 0.7))
                
                trajectory_points.append([curr_x, curr_y, curr_theta, curr_speed])
                
                # Stop when very close
                if np.sqrt((curr_x - goal_x)**2 + (curr_y - goal_y)**2) < 1.0:
                    break
            
            return np.array(trajectory_points) if len(trajectory_points) > 0 else None
            
        except Exception as e:
            return None
    
    def _evaluate_trajectories(self, trajectories: List[np.ndarray], 
                              occupancy_grid: Any, goal: np.ndarray,
                              current_state: np.ndarray) -> Optional[np.ndarray]:
        """Evaluate and select best trajectory"""
        if not trajectories:
            return None
        
        best_trajectory = None
        best_score = -float('inf')
        
        goal_x, goal_y = goal[0], goal[1]
        x, y = current_state[0], current_state[1]
        
        for trajectory in trajectories:
            try:
                # Check collision
                if self._check_trajectory_collision(trajectory, occupancy_grid):
                    continue
                
                # Calculate score
                score = self._calculate_trajectory_score(
                    trajectory, goal, current_state
                )
                
                if score > best_score:
                    best_score = score
                    best_trajectory = trajectory
                    
            except Exception as e:
                continue
        
        return best_trajectory
    
    def _calculate_trajectory_score(self, trajectory: np.ndarray, 
                                   goal: np.ndarray, current_state: np.ndarray) -> float:
        """Calculate trajectory score"""
        try:
            if len(trajectory) == 0:
                return -1000
            
            goal_x, goal_y = goal[0], goal[1];
            
            # Distance to goal (lower is better)
            final_point = trajectory[-1];
            distance_to_goal = np.sqrt((final_point[0] - goal_x)**2 + (final_point[1] - goal_y)**2);
            goal_score = -distance_to_goal * 2.0;
            
            # Progress towards goal (higher is better)
            start_distance = np.sqrt((current_state[0] - goal_x)**2 + (current_state[1] - goal_y)**2);
            progress = max(0, start_distance - distance_to_goal);
            progress_score = progress * 3.0;
            
            # Smoothness (penalize sharp turns)
            smoothness_score = 0;
            if len(trajectory) > 2:
                for i in range(1, len(trajectory) - 1):
                    angle_change = abs(self._normalize_angle(
                        trajectory[i+1][2] - trajectory[i][2]
                    ));
                    smoothness_score -= angle_change * 10.0;
            
            # Speed consistency
            speed_score = 0;
            if len(trajectory) > 1:
                speeds = trajectory[:, 3];
                speed_variance = np.var(speeds);
                speed_score = -speed_variance * 2.0;
            
            # Length bonus (prefer longer trajectories that make progress)
            length_score = len(trajectory) * 0.5;
            
            total_score = goal_score + progress_score + smoothness_score + speed_score + length_score;
            
            return total_score;
            
        except Exception as e:
            return -1000
    
    def _check_trajectory_collision(self, trajectory: np.ndarray, 
                                   occupancy_grid: Any) -> bool:
        """Check if trajectory collides with obstacles"""
        try:
            if not hasattr(occupancy_grid, 'is_occupied'):
                return False
            
            for point in trajectory[::2]:  # Check every 2nd point for performance
                x, y = point[0], point[1]
                
                # Check multiple points around vehicle
                check_points = [
                    (x, y),
                    (x + self.safety_margin, y),
                    (x - self.safety_margin, y),
                    (x, y + self.safety_margin),
                    (x, y - self.safety_margin)
                ]
                
                for check_x, check_y in check_points:
                    if occupancy_grid.is_occupied(check_x, check_y):
                        return True
            
            return False
            
        except Exception as e:
            return True  # Assume collision if can't check
    
    def _emergency_trajectory(self, current_state: np.ndarray) -> Optional[np.ndarray]:
        """Generate emergency stop trajectory"""
        try:
            x, y, theta, v, _, _ = current_state
            
            trajectory_points = []
            dt = 0.1
            
            # Emergency deceleration
            curr_speed = v
            decel_rate = 3.0
            
            for i in range(10):
                curr_speed = max(0, curr_speed - decel_rate * dt)
                
                # Move forward slightly
                x += curr_speed * np.cos(theta) * dt
                y += curr_speed * np.sin(theta) * dt
                
                trajectory_points.append([x, y, theta, curr_speed])
                
                if curr_speed <= 0:
                    break
            
            return np.array(trajectory_points) if len(trajectory_points) > 0 else None
            
        except Exception as e:
            return None
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle