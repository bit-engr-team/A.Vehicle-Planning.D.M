import numpy as np

class AutonomousVehicle:
    def __init__(self, config, planner):
        self.config = config
        self.planner = planner
        
        # Vehicle state [x, y, theta, v]
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.goal = None
        
        # Vehicle parameters
        self.length = config['length']
        self.width = config['width']
        self.wheelbase = config['wheelbase']
        self.max_speed = config['max_speed']
        self.max_acceleration = config['max_acceleration']
        self.max_deceleration = config['max_deceleration']
        self.max_steering_angle = config['max_steering_angle']
        
        # Control inputs
        self.steering_angle = 0.0
        self.acceleration = 0.0
        
    def set_goal(self, goal):
        """Set the goal position [x, y, theta]"""
        self.goal = goal
        
    def plan_trajectory(self, occupancy_grid):
        """Plan trajectory using lattice planner"""
        if self.goal is None:
            return None
            
        return self.planner.plan(self.state, self.goal, occupancy_grid)
        
    def control(self, trajectory):
        """Execute trajectory tracking control"""
        if trajectory is None or len(trajectory) < 2:
            self.acceleration = 0.0
            self.steering_angle = 0.0
            return
            
        # Simple pure pursuit controller
        target_point = trajectory[1]  # Next waypoint
        
        # Calculate steering angle
        dx = target_point[0] - self.state[0]
        dy = target_point[1] - self.state[1]
        target_heading = np.arctan2(dy, dx)
        heading_error = target_heading - self.state[2]
        
        # Normalize angle
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # Simple proportional control
        self.steering_angle = np.clip(2.0 * heading_error, 
                                    -self.max_steering_angle, 
                                    self.max_steering_angle)
        
        # Speed control
        target_speed = target_point[3] if len(target_point) > 3 else 5.0
        speed_error = target_speed - self.state[3]
        self.acceleration = np.clip(speed_error, 
                                  self.max_deceleration, 
                                  self.max_acceleration)
        
    def update_dynamics(self, dt):
        """Update vehicle dynamics using bicycle model"""
        x, y, theta, v = self.state
        
        # Bicycle model
        beta = np.arctan(0.5 * np.tan(self.steering_angle))
        
        dx = v * np.cos(theta + beta)
        dy = v * np.sin(theta + beta)
        dtheta = (v / self.wheelbase) * np.sin(beta)
        dv = self.acceleration
        
        # Update state
        self.state[0] += dx * dt
        self.state[1] += dy * dt
        self.state[2] += dtheta * dt
        self.state[3] = np.clip(self.state[3] + dv * dt, 0, self.max_speed)
        
        # Normalize heading
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
    def get_footprint(self):
        """Get vehicle footprint corners"""
        x, y, theta = self.state[:3]
        
        # Vehicle corners relative to center
        corners = np.array([
            [self.length/2, self.width/2],
            [self.length/2, -self.width/2],
            [-self.length/2, -self.width/2],
            [-self.length/2, self.width/2]
        ])
        
        # Rotation matrix
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        
        # Transform corners
        rotated_corners = corners @ R.T
        footprint = rotated_corners + np.array([x, y])
        
        return footprint