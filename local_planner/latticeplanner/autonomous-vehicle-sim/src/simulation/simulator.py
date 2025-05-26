import numpy as np
import time
from .environment import Environment

class Simulator:
    def __init__(self, config, vehicle, occupancy_grid, renderer=None):
        """Initialize the simulator with flexible parameter handling"""
        self.config = config
        self.vehicle = vehicle
        self.occupancy_grid = occupancy_grid
        self.renderer = renderer
        
        self.environment = Environment(occupancy_grid)
        
        self.dt = config.get('dt', 0.1)
        self.max_time = config.get('max_time', 100.0)
        self.current_time = 0.0
        
        self.running = False
        self.trajectory_history = []
        
    def add_obstacles(self):
        """Add predefined obstacles to the environment"""
        # Static obstacles
        self.occupancy_grid.add_rectangle_obstacle(20, 10, 4, 2)
        self.occupancy_grid.add_rectangle_obstacle(-10, 15, 3, 6)
        self.occupancy_grid.add_circle_obstacle(30, -5, 3)
        self.occupancy_grid.add_rectangle_obstacle(5, -20, 8, 3)
        self.occupancy_grid.add_circle_obstacle(-15, -10, 2.5)
        
        # Boundary walls
        for x in range(-50, 51, 5):
            self.occupancy_grid.add_rectangle_obstacle(x, 49, 2, 2)
            self.occupancy_grid.add_rectangle_obstacle(x, -49, 2, 2)
            
        for y in range(-50, 51, 5):
            self.occupancy_grid.add_rectangle_obstacle(49, y, 2, 2)
            self.occupancy_grid.add_rectangle_obstacle(-49, y, 2, 2)
        
    def step(self):
        """Single simulation step"""
        # Plan trajectory
        trajectory = self.vehicle.plan_trajectory(self.occupancy_grid)
        
        # Execute control
        self.vehicle.control(trajectory)
        
        # Update vehicle dynamics
        self.vehicle.update_dynamics(self.dt)
        
        # Store trajectory for visualization
        if trajectory is not None:
            self.trajectory_history.append(trajectory.copy())
            # Keep only recent trajectories
            if len(self.trajectory_history) > 10:
                self.trajectory_history.pop(0)
                
        # Update time
        self.current_time += self.dt
        
        # Render if renderer is available
        if self.renderer is not None:
            self.renderer.render(self.vehicle, self.occupancy_grid, trajectory, self.current_time)
        
        return trajectory
        
    def run(self):
        """Run the complete simulation"""
        self.running = True
        
        print("Starting autonomous vehicle simulation...")
        print(f"Goal: {self.vehicle.goal}")
        
        step_count = 0
        
        while self.running and self.current_time < self.max_time:
            step_start_time = time.time()
            
            # Simulation step
            trajectory = self.step()
            
            # Check if goal is reached
            if self.vehicle.goal is not None:
                distance_to_goal = np.linalg.norm(self.vehicle.state[:2] - self.vehicle.goal[:2])
                if distance_to_goal < 2.0:
                    print(f"Goal reached at time {self.current_time:.1f}s!")
                    break
                    
            # Print progress
            if step_count % 50 == 0:
                print(f"Time: {self.current_time:.1f}s, "
                      f"Position: ({self.vehicle.state[0]:.1f}, {self.vehicle.state[1]:.1f}), "
                      f"Speed: {self.vehicle.state[3]:.1f} m/s")
                      
            step_count += 1
            
            # Real-time simulation timing
            step_time = time.time() - step_start_time
            sleep_time = max(0, self.dt - step_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
        print("Simulation completed!")
        
        # Keep visualization open
        if self.renderer is not None:
            input("Press Enter to close...")
            self.renderer.close()
        
    def stop(self):
        """Stop the simulation"""
        self.running = False
        
    def reset(self):
        """Reset simulation to initial state"""
        self.current_time = 0.0
        self.vehicle.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.trajectory_history = []

# Test the class
if __name__ == "__main__":
    print("Testing Simulator class...")
    
    # Create a simple test config
    config = {
        'dt': 0.1,
        'max_time': 10.0
    }
    
    print("✓ Simulator class defined successfully")
    print("Note: Full test requires vehicle, grid, and renderer instances")