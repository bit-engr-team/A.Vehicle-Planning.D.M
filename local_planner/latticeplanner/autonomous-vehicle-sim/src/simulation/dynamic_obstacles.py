import numpy as np
import time

class DynamicObstacle:
    def __init__(self, x, y, radius, vx, vy, obstacle_type="circle"):
        self.x = x
        self.y = y
        self.radius = radius
        self.vx = vx  # velocity in x direction
        self.vy = vy  # velocity in y direction
        self.obstacle_type = obstacle_type
        self.creation_time = time.time()
        
    def update(self, dt):
        """Update obstacle position"""
        self.x += self.vx * dt
        self.y += self.vy * dt
        
    def get_position(self):
        return (self.x, self.y)
        
    def is_collision(self, x, y, safety_margin=0.5):
        """Check if point (x,y) collides with this obstacle"""
        distance = np.sqrt((x - self.x)**2 + (y - self.y)**2)
        return distance < (self.radius + safety_margin)

class MovingVehicle(DynamicObstacle):
    def __init__(self, x, y, length, width, vx, vy, heading=0):
        super().__init__(x, y, max(length, width)/2, vx, vy, "vehicle")
        self.length = length
        self.width = width
        self.heading = heading
        
    def update(self, dt):
        """Update vehicle position and heading"""
        super().update(dt)
        # Simple heading update based on velocity
        if abs(self.vx) > 0.1 or abs(self.vy) > 0.1:
            self.heading = np.arctan2(self.vy, self.vx)
            
    def get_corners(self):
        """Get vehicle corner positions"""
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        
        corners = [
            [self.length/2, self.width/2],
            [self.length/2, -self.width/2],
            [-self.length/2, -self.width/2],
            [-self.length/2, self.width/2]
        ]
        
        rotated_corners = []
        for corner in corners:
            x_rot = corner[0] * cos_h - corner[1] * sin_h + self.x
            y_rot = corner[0] * sin_h + corner[1] * cos_h + self.y
            rotated_corners.append([x_rot, y_rot])
            
        return np.array(rotated_corners)