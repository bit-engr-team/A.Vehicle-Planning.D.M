import numpy as np
from typing import Optional, Dict, Any, Tuple
import time

class AutonomousVehicle:
    """Advanced Autonomous Vehicle with improved dynamics and control"""
    
    def __init__(self, config: Dict[str, Any], planner=None):
        # Vehicle configuration
        self.config = config
        self.length = config['length']
        self.width = config['width']
        self.wheelbase = config['wheelbase']
        self.max_speed = config['max_speed']
        self.max_acceleration = config['max_acceleration']
        self.max_deceleration = config['max_deceleration']
        self.max_steering_angle = config['max_steering_angle']
        
        # Vehicle state: [x, y, heading, speed, acceleration, steering_angle]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Goal and trajectory
        self.goal = None
        self.current_trajectory = None
        self.trajectory_index = 0
        
        # Planner
        self.planner = planner
        
        # Control system
        self.controller = VehicleController(config)
        
        # Performance monitoring
        self.total_distance = 0.0
        self.start_time = time.time()
        self.last_position = None
        
        # Parking system
        self.parking_state = "driving"  # "driving", "approaching_parking", "parking", "parked"
        self.parking_zone = None
        self.parking_precision = 1.5  # meters
        
        print(f"✅ AutonomousVehicle initialized: {self.length}x{self.width}m")
    
    def set_goal(self, goal: np.ndarray, parking_zone=None):
        """Set navigation goal with optional parking zone"""
        self.goal = goal.copy()
        self.parking_zone = parking_zone
        self.parking_state = "driving"
        
        if parking_zone:
            print(f"🎯 Goal set: ({goal[0]:.1f}, {goal[1]:.1f}) with parking at ({parking_zone.x:.1f}, {parking_zone.y:.1f})")
        else:
            print(f"🎯 Goal set: ({goal[0]:.1f}, {goal[1]:.1f})")
    
    def update_dynamics(self, dt: float):
        """Update vehicle dynamics using bicycle model"""
        if len(self.state) < 6:
            # Extend state if needed
            self.state = np.array([self.state[0], self.state[1], self.state[2], 
                                 self.state[3] if len(self.state) > 3 else 0.0, 
                                 0.0, 0.0])
        
        x, y, heading, speed, acceleration, steering_angle = self.state
        
        # Bicycle model dynamics
        # Front wheel position
        front_x = x + self.wheelbase * np.cos(heading)
        front_y = y + self.wheelbase * np.sin(heading)
        
        # Update speed
        speed += acceleration * dt
        speed = np.clip(speed, 0.0, self.max_speed)
        
        # Update position using bicycle model
        if abs(steering_angle) > 0.001:  # Avoid division by zero
            # Turning radius
            R = self.wheelbase / np.tan(steering_angle)
            
            # Angular velocity
            omega = speed / R
            
            # Update heading
            heading += omega * dt
            
            # Update position (curved motion)
            dx = speed * np.cos(heading) * dt
            dy = speed * np.sin(heading) * dt
        else:
            # Straight motion
            dx = speed * np.cos(heading) * dt
            dy = speed * np.sin(heading) * dt
        
        x += dx
        y += dy
        
        # Normalize heading
        while heading > np.pi:
            heading -= 2 * np.pi
        while heading < -np.pi:
            heading += 2 * np.pi
        
        # Update state
        self.state = np.array([x, y, heading, speed, acceleration, steering_angle])
        
        # Update distance tracking
        if self.last_position is not None:
            distance_increment = np.linalg.norm([x - self.last_position[0], y - self.last_position[1]])
            self.total_distance += distance_increment
        self.last_position = np.array([x, y])
        
        # Update parking state
        self._update_parking_state()
    
    def control(self, trajectory: Optional[np.ndarray]):
        """Control the vehicle using the given trajectory"""
        if trajectory is None or len(trajectory) == 0:
            # Emergency stop
            self.state[4] = self.max_deceleration  # acceleration
            self.state[5] = 0.0  # steering
            return
        
        # Update current trajectory
        self.current_trajectory = trajectory
        
        # Use controller to compute control inputs
        acceleration, steering = self.controller.compute_control(self.state, trajectory, self.goal)
        
        # Apply control limits
        acceleration = np.clip(acceleration, self.max_deceleration, self.max_acceleration)
        steering = np.clip(steering, -self.max_steering_angle, self.max_steering_angle)
        
        # Update control inputs in state
        self.state[4] = acceleration
        self.state[5] = steering
    
    def _update_parking_state(self):
        """Update parking state machine"""
        if self.parking_zone is None:
            return
        
        # Distance to parking zone
        parking_distance = np.linalg.norm([
            self.state[0] - self.parking_zone.x,
            self.state[1] - self.parking_zone.y
        ])
        
        # Distance to goal
        goal_distance = float('inf')
        if self.goal is not None:
            goal_distance = np.linalg.norm(self.state[:2] - self.goal[:2])
        
        # State machine
        if self.parking_state == "driving":
            if goal_distance < 8.0:  # Approaching goal
                self.parking_state = "approaching_parking"
                
        elif self.parking_state == "approaching_parking":
            if parking_distance < 4.0:  # Close to parking zone
                self.parking_state = "parking"
                
        elif self.parking_state == "parking":
            if (parking_distance < self.parking_precision and 
                self.state[3] < 0.5):  # Stopped in parking zone
                self.parking_state = "parked"
                print("🅿️ Vehicle successfully parked!")
    
    def is_parked(self) -> bool:
        """Check if vehicle is parked"""
        return self.parking_state == "parked"
    
    def get_position(self) -> np.ndarray:
        """Get current position"""
        return self.state[:2]
    
    def get_heading(self) -> float:
        """Get current heading"""
        return self.state[2]
    
    def get_speed(self) -> float:
        """Get current speed"""
        return self.state[3]
    
    def get_performance_metrics(self) -> Dict[str, float]:
        """Get performance metrics"""
        elapsed_time = time.time() - self.start_time
        avg_speed = self.total_distance / elapsed_time if elapsed_time > 0 else 0.0
        
        return {
            'total_distance': self.total_distance,
            'elapsed_time': elapsed_time,
            'average_speed': avg_speed,
            'current_speed': self.get_speed(),
            'parking_state': self.parking_state
        }
    
    def get_vehicle_corners(self) -> np.ndarray:
        """Get vehicle corner positions for collision detection"""
        x, y, heading = self.state[0], self.state[1], self.state[2]
        
        # Vehicle corners in local frame
        corners_local = np.array([
            [-self.length/2, -self.width/2],
            [self.length/2, -self.width/2],
            [self.length/2, self.width/2],
            [-self.length/2, self.width/2]
        ])
        
        # Rotation matrix
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        rotation_matrix = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        
        # Transform to world frame
        corners_world = corners_local @ rotation_matrix.T
        corners_world[:, 0] += x
        corners_world[:, 1] += y
        
        return corners_world

class VehicleController:
    """Advanced vehicle controller using multiple control strategies"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # Control parameters
        self.look_ahead_distance_base = 4.0
        self.look_ahead_speed_factor = 0.3
        self.max_look_ahead = 12.0
        self.min_look_ahead = 2.0
        
        # PID controllers
        self.speed_pid = PIDController(kp=2.0, ki=0.1, kd=0.05)
        self.steering_pid = PIDController(kp=1.5, ki=0.0, kd=0.1)
        
        # Stanley controller parameters
        self.stanley_gain = 1.0
        self.cross_track_gain = 2.0
        
        # Smoothing filters
        self.steering_filter = 0.3  # Reduced for responsiveness
        self.throttle_filter = 0.4
        self.prev_steering = 0.0
        self.prev_throttle = 0.0
        
        print("✅ VehicleController initialized with PID and Stanley control")
    
    def compute_control(self, vehicle_state: np.ndarray, trajectory: np.ndarray, 
                       goal: Optional[np.ndarray]) -> Tuple[float, float]:
        """Compute control inputs (acceleration, steering)"""
        
        if len(trajectory) == 0:
            return 0.0, 0.0
        
        # Adaptive look-ahead distance
        current_speed = vehicle_state[3]
        look_ahead_distance = np.clip(
            self.look_ahead_distance_base + current_speed * self.look_ahead_speed_factor,
            self.min_look_ahead, self.max_look_ahead
        )
        
        # Find look-ahead point
        target_point = self._find_look_ahead_point(vehicle_state[:2], trajectory, look_ahead_distance)
        
        if target_point is None:
            target_point = trajectory[-1]  # Use last point as fallback
        
        # Stanley controller for steering
        steering = self._stanley_control(vehicle_state, target_point, trajectory)
        
        # Speed control
        target_speed = self._determine_target_speed(vehicle_state, trajectory, goal)
        acceleration = self._speed_control(vehicle_state[3], target_speed)
        
        # Apply smoothing
        steering = self._apply_smoothing(steering, self.prev_steering, self.steering_filter)
        acceleration = self._apply_smoothing(acceleration, self.prev_throttle, self.throttle_filter)
        
        # Store previous values
        self.prev_steering = steering
        self.prev_throttle = acceleration
        
        return acceleration, steering
    
    def _find_look_ahead_point(self, vehicle_pos: np.ndarray, trajectory: np.ndarray, 
                              look_ahead_distance: float) -> Optional[np.ndarray]:
        """Find look-ahead point on trajectory"""
        
        if len(trajectory) == 0:
            return None
        
        # Find closest point on trajectory
        distances = np.linalg.norm(trajectory[:, :2] - vehicle_pos, axis=1)
        closest_idx = np.argmin(distances)
        
        # Search forward from closest point for look-ahead distance
        for i in range(closest_idx, len(trajectory)):
            point_distance = np.linalg.norm(trajectory[i, :2] - vehicle_pos)
            if point_distance >= look_ahead_distance:
                return trajectory[i]
        
        # If no point found at look-ahead distance, return last point
        return trajectory[-1]
    
    def _stanley_control(self, vehicle_state: np.ndarray, target_point: np.ndarray, 
                        trajectory: np.ndarray) -> float:
        """Stanley controller for lateral control"""
        
        vehicle_x, vehicle_y, vehicle_heading = vehicle_state[0], vehicle_state[1], vehicle_state[2]
        target_x, target_y = target_point[0], target_point[1]
        
        # 1. Heading error
        target_heading = np.arctan2(target_y - vehicle_y, target_x - vehicle_x)
        heading_error = target_heading - vehicle_heading
        
        # Normalize heading error
        while heading_error > np.pi:
            heading_error -= 2 * np.pi
        while heading_error < -np.pi:
            heading_error += 2 * np.pi
        
        # 2. Cross-track error
        cross_track_error = self._calculate_cross_track_error(vehicle_state, trajectory)
        
        # 3. Stanley control law
        vehicle_speed = max(0.1, vehicle_state[3])  # Avoid division by zero
        cross_track_term = np.arctan2(self.cross_track_gain * cross_track_error, vehicle_speed)
        
        steering = self.stanley_gain * heading_error + cross_track_term
        
        return steering
    
    def _calculate_cross_track_error(self, vehicle_state: np.ndarray, 
                                   trajectory: np.ndarray) -> float:
        """Calculate cross-track error from trajectory"""
        
        vehicle_pos = vehicle_state[:2]
        
        if len(trajectory) < 2:
            return 0.0
        
        # Find closest segment on trajectory
        min_distance = float('inf')
        cross_track_error = 0.0
        
        for i in range(len(trajectory) - 1):
            p1 = trajectory[i, :2]
            p2 = trajectory[i + 1, :2]
            
            # Vector from p1 to p2
            segment_vec = p2 - p1
            segment_length = np.linalg.norm(segment_vec)
            
            if segment_length < 1e-6:
                continue
            
            # Vector from p1 to vehicle
            vehicle_vec = vehicle_pos - p1
            
            # Project vehicle onto segment
            projection_length = np.dot(vehicle_vec, segment_vec) / segment_length
            projection_length = np.clip(projection_length, 0, segment_length)
            
            # Closest point on segment
            closest_point = p1 + (projection_length / segment_length) * segment_vec
            
            # Distance to closest point
            distance = np.linalg.norm(vehicle_pos - closest_point)
            
            if distance < min_distance:
                min_distance = distance
                
                # Cross-track error (signed)
                cross_vec = vehicle_pos - closest_point
                segment_normal = np.array([-segment_vec[1], segment_vec[0]])
                if np.linalg.norm(segment_normal) > 1e-6:
                    segment_normal = segment_normal / np.linalg.norm(segment_normal)
                    cross_track_error = np.dot(cross_vec, segment_normal)
        
        return cross_track_error
    
    def _determine_target_speed(self, vehicle_state: np.ndarray, trajectory: np.ndarray, 
                               goal: Optional[np.ndarray]) -> float:
        """Determine target speed based on situation"""
        
        # Base speed from trajectory
        if len(trajectory) > 0 and len(trajectory[0]) > 3:
            trajectory_speed = trajectory[0][3]
        else:
            trajectory_speed = 8.0
        
        # Distance to goal
        if goal is not None:
            goal_distance = np.linalg.norm(vehicle_state[:2] - goal[:2])
            
            # Slow down when approaching goal
            if goal_distance < 10.0:
                approach_factor = max(0.3, goal_distance / 10.0)
                trajectory_speed *= approach_factor
            
            # Very slow when very close to goal
            if goal_distance < 3.0:
                trajectory_speed = min(trajectory_speed, 2.0)
        
        # Curvature-based speed adjustment
        curvature = self._estimate_path_curvature(trajectory)
        curvature_factor = max(0.5, 1.0 - abs(curvature) * 5.0)
        trajectory_speed *= curvature_factor
        
        return np.clip(trajectory_speed, 1.0, self.config['max_speed'])
    
    def _estimate_path_curvature(self, trajectory: np.ndarray) -> float:
        """Estimate path curvature from trajectory"""
        if len(trajectory) < 3:
            return 0.0
        
        # Use first few points to estimate curvature
        points = trajectory[:min(5, len(trajectory)), :2]
        
        if len(points) < 3:
            return 0.0
        
        # Calculate curvature using three points
        p1, p2, p3 = points[0], points[len(points)//2], points[-1]
        
        # Calculate triangle area
        area = 0.5 * abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1]))
        
        # Calculate side lengths
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)
        
        # Avoid division by zero
        if a * b * c < 1e-6:
            return 0.0
        
        # Curvature = 4 * Area / (a * b * c)
        curvature = 4 * area / (a * b * c)
        
        return curvature
    
    def _speed_control(self, current_speed: float, target_speed: float) -> float:
        """PID speed control"""
        speed_error = target_speed - current_speed
        acceleration = self.speed_pid.update(speed_error)
        
        return np.clip(acceleration, self.config['max_deceleration'], self.config['max_acceleration'])
    
    def _apply_smoothing(self, new_value: float, prev_value: float, alpha: float) -> float:
        """Apply exponential smoothing"""
        return alpha * new_value + (1 - alpha) * prev_value

class PIDController:
    """PID Controller implementation"""
    
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
    
    def update(self, error: float) -> float:
        """Update PID controller"""
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            dt = 0.01  # Prevent division by zero
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        # Prevent integral windup
        self.integral = np.clip(self.integral, -10.0, 10.0)
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.prev_error) / dt
        
        # Update for next iteration
        self.prev_error = error
        self.prev_time = current_time
        
        return proportional + integral + derivative
    
    def reset(self):
        """Reset PID controller"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()