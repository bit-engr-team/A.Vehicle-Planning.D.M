import numpy as np
import time

class Simulator:
    def __init__(self, config, vehicle, occupancy_grid, renderer):
        """Initialize the simulator with flexible parameter handling"""
        self.config = config
        self.vehicle = vehicle
        self.occupancy_grid = occupancy_grid
        self.renderer = renderer
        
        self.dt = config.get('dt', 0.1)
        self.max_time = config.get('max_time', 100.0)
        self.current_time = 0.0
        
        self.running = True
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
        if not self.running or self.current_time >= self.max_time:
            return False
        
        try:
            # Plan trajectory
            trajectory = self.vehicle.planner.plan(
                self.vehicle.state, 
                self.vehicle.goal, 
                self.occupancy_grid
            )
            
            # Execute control
            self.vehicle.control(trajectory)
            
            # Update vehicle dynamics
            self.vehicle.update_dynamics(self.dt)
            
            # Render the scene
            if hasattr(self, 'renderer') and self.renderer:
                self.renderer.render(
                    self.vehicle, 
                    self.occupancy_grid, 
                    trajectory, 
                    self.current_time
                )
            
            # Update time
            self.current_time += self.dt
            
            return True
            
        except Exception as e:
            print(f"Simulation step error: {e}")
            return False
    
    def run(self):
        """Run the complete simulation"""
        print("Running simulation...")
        
        step_count = 0
        max_steps = int(self.max_time / self.dt)
        
        while step_count < max_steps and self.step():
            step_count += 1
            
            # Check if goal reached
            if self.vehicle.goal is not None:
                distance = np.linalg.norm(self.vehicle.state[:2] - self.vehicle.goal[:2])
                if distance < 5.0:
                    print(f"Goal reached after {step_count * self.dt:.1f} seconds!")
                    break
            
            # Progress update every 100 steps
            if step_count % 100 == 0:
                print(f"Step {step_count}, Time: {step_count * self.dt:.1f}s")
        
        print("Simulation completed.")
        return step_count * self.dt

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