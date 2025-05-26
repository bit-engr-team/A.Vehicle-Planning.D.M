import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import time

# Add the src directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

class DynamicObstacle:
    """Simple dynamic obstacle for advanced simulation"""
    def __init__(self, x, y, width, height, vx, vy=0):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.vx = vx  # velocity in x direction
        self.vy = vy  # velocity in y direction
        self.radius = max(width, height) / 2.0
        
    def update(self, dt):
        """Update obstacle position"""
        self.x += self.vx * dt
        self.y += self.vy * dt
        
    def get_position(self):
        return np.array([self.x, self.y])
        
    def get_bounds(self):
        return {
            'x_min': self.x - self.width/2,
            'x_max': self.x + self.width/2,
            'y_min': self.y - self.height/2,
            'y_max': self.y + self.height/2
        }

def test_advanced_imports():
    """Test imports for advanced simulation"""
    try:
        from simulation.simulator import Simulator
        print("✓ Simulator imported successfully")
    except ImportError as e:
        print(f"✗ Simulator import failed: {e}")
        return False
        
    try:
        from vehicle.autonomous_vehicle import AutonomousVehicle
        print("✓ AutonomousVehicle imported successfully")
    except ImportError as e:
        print(f"✗ AutonomousVehicle import failed: {e}")
        return False
        
    try:
        from planning.lattice_planner import LatticePlanner
        print("✓ LatticePlanner imported successfully")
    except ImportError as e:
        print(f"✗ LatticePlanner import failed: {e}")
        return False
        
    try:
        from mapping.occupancy_grid import OccupancyGrid
        print("✓ OccupancyGrid imported successfully")
    except ImportError as e:
        print(f"✗ OccupancyGrid import failed: {e}")
        return False
        
    try:
        from visualization.matplotlib_viz import MatplotlibRenderer
        print("✓ MatplotlibRenderer imported successfully")
    except ImportError as e:
        print(f"✗ MatplotlibRenderer import failed: {e}")
        return False
        
    return True

def get_clean_config():
    """Clean simulation configuration"""
    return {
        'simulation': {
            'dt': 0.1,
            'max_time': 60.0
        },
        'vehicle': {
            'length': 4.5,
            'width': 2.0,
            'wheelbase': 2.7,
            'max_speed': 15.0,
            'max_acceleration': 3.0,
            'max_deceleration': -4.0,
            'max_steering_angle': 0.5
        },
        'lattice_planner': {
            'prediction_horizon': 3.0,    # Clean lattice view
            'lateral_resolution': 2.5,     # Less dense
            'speed_resolution': 2.0,
            'max_lateral_offset': 4.0,     # Small offset
            'trajectory_length': 20        # Short trajectories
        },
        'occupancy_grid': {
            'width': 400,
            'height': 300,
            'resolution': 0.4,
            'origin_x': -80.0,
            'origin_y': -60.0
        }
    }

def create_clean_street_map(occupancy_grid):
    """Create clean street network"""
    print("Creating clean street network...")
    
    # Simple street parameters
    main_street_width = 8.0
    side_street_width = 6.0
    
    # 1. Main horizontal street
    print("  - Creating main street...")
    for x in range(-70, 81, 4):
        # North boundary
        occupancy_grid.add_rectangle_obstacle(x, main_street_width/2 + 1, 2, 1)
        # South boundary
        occupancy_grid.add_rectangle_obstacle(x, -main_street_width/2 - 1, 2, 1)
    
    # 2. One side street at x=20
    print("  - Creating side street...")
    for y in range(-40, 41, 4):
        # West boundary
        occupancy_grid.add_rectangle_obstacle(20 - side_street_width/2 - 1, y, 1, 2)
        # East boundary
        occupancy_grid.add_rectangle_obstacle(20 + side_street_width/2 + 1, y, 1, 2)
    
    # 3. Simple lane markings
    print("  - Adding lane markings...")
    for x in range(-60, 71, 20):
        occupancy_grid.add_rectangle_obstacle(x, 0, 0.5, 0.2)  # Center line
    
    # 4. Few static obstacles
    print("  - Adding static obstacles...")
    static_obstacles = [
        (10, 5),     # Parked car north side
        (40, -5),    # Parked car south side
        (60, 2),     # Car in left lane
        (17, 15),    # Side street parking
        (23, -20),   # Side street parking
    ]
    
    for x, y in static_obstacles:
        occupancy_grid.add_rectangle_obstacle(x, y, 4.0, 1.8)
    
    # 5. Few buildings (far away)
    print("  - Adding buildings...")
    buildings = [
        (-30, 20), (0, -25), (30, 25), (50, -25), (70, 20)
    ]
    
    for x, y in buildings:
        occupancy_grid.add_rectangle_obstacle(x, y, 8, 6)
    
    print("  ✓ Clean street network completed")

def create_simple_dynamic_obstacles():
    """Create simple dynamic obstacles"""
    obstacles = []
    
    # One pedestrian
    obstacles.append(DynamicObstacle(25, -10, 0.6, 0.6, 0, 0.8))  # Pedestrian crossing
    
    # One slow vehicle
    obstacles.append(DynamicObstacle(-50, 2, 4.0, 1.8, 5.0, 0))  # Slow car
    
    return obstacles

def clean_render(vehicle, occupancy_grid, planner, trajectory, dynamic_obstacles, current_time):
    """Clean rendering function - fixed for OccupancyGrid"""
    try:
        plt.clf()
        
        # Get grid properties
        try:
            # Try different possible attributes for grid data
            if hasattr(occupancy_grid, 'grid'):
                grid_data = occupancy_grid.grid
            elif hasattr(occupancy_grid, 'data'):
                grid_data = occupancy_grid.data
            elif hasattr(occupancy_grid, 'occupancy_map'):
                grid_data = occupancy_grid.occupancy_map
            else:
                # Skip grid rendering if can't access data
                grid_data = None
                
            # Get grid properties
            origin_x = getattr(occupancy_grid, 'origin_x', -80.0)
            origin_y = getattr(occupancy_grid, 'origin_y', -60.0)
            resolution = getattr(occupancy_grid, 'resolution', 0.4)
            
            # Show occupancy grid if available
            if grid_data is not None:
                extent = [origin_x, origin_x + grid_data.shape[1] * resolution,
                         origin_y, origin_y + grid_data.shape[0] * resolution]
                plt.imshow(grid_data, extent=extent, origin='lower', cmap='Greys', alpha=0.6)
            else:
                # Draw a simple background grid
                x_range = np.arange(-75, 85, 5)
                y_range = np.arange(-55, 55, 5)
                for x in x_range:
                    plt.axvline(x, color='lightgray', alpha=0.3, linewidth=0.5)
                for y in y_range:
                    plt.axhline(y, color='lightgray', alpha=0.3, linewidth=0.5)
                    
        except Exception as e:
            print(f"Grid rendering error: {e}")
            # Draw simple background
            plt.gca().set_facecolor('lightgray')
        
        # Add vehicle
        vehicle_x, vehicle_y = vehicle.state[0], vehicle.state[1]
        vehicle_heading = vehicle.state[2]
        
        # Vehicle as rectangle
        vehicle_length, vehicle_width = 4.5, 2.0
        vehicle_corners = np.array([
            [-vehicle_length/2, -vehicle_width/2],
            [vehicle_length/2, -vehicle_width/2],
            [vehicle_length/2, vehicle_width/2],
            [-vehicle_length/2, vehicle_width/2],
            [-vehicle_length/2, -vehicle_width/2]
        ])
        
        # Rotate and translate
        cos_h, sin_h = np.cos(vehicle_heading), np.sin(vehicle_heading)
        rotation_matrix = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        rotated_corners = vehicle_corners @ rotation_matrix.T
        rotated_corners[:, 0] += vehicle_x
        rotated_corners[:, 1] += vehicle_y
        
        plt.plot(rotated_corners[:, 0], rotated_corners[:, 1], 'b-', linewidth=3, label='Vehicle')
        plt.plot(vehicle_x, vehicle_y, 'bo', markersize=6)
        
        # Add goal
        if hasattr(vehicle, 'goal') and vehicle.goal is not None:
            plt.plot(vehicle.goal[0], vehicle.goal[1], 'g*', markersize=15, label='Goal')
        
        # Add dynamic obstacles
        for i, obstacle in enumerate(dynamic_obstacles):
            circle = plt.Circle((obstacle.x, obstacle.y), obstacle.radius, 
                              color='red', alpha=0.7, zorder=5, 
                              label='Dynamic Obstacle' if i == 0 else "")
            plt.gca().add_patch(circle)
            
            # Velocity arrow
            if abs(obstacle.vx) > 0.1 or abs(obstacle.vy) > 0.1:
                plt.arrow(obstacle.x, obstacle.y, obstacle.vx*1.5, obstacle.vy*1.5, 
                         head_width=0.5, head_length=0.3, fc='red', ec='red', alpha=0.8)
        
        # Clean lattice trajectories (only a few)
        if hasattr(planner, 'all_trajectories') and hasattr(planner, 'all_trajectories') and planner.all_trajectories:
            for i, traj in enumerate(planner.all_trajectories[:8]):  # Only 8 trajectories
                if len(traj) > 1:
                    x_coords = [point[0] for point in traj]
                    y_coords = [point[1] for point in traj]
                    plt.plot(x_coords, y_coords, 'c-', alpha=0.4, linewidth=1)
        
        # Best trajectory
        if trajectory is not None and len(trajectory) > 1:
            x_coords = [point[0] for point in trajectory]
            y_coords = [point[1] for point in trajectory]
            plt.plot(x_coords, y_coords, 'g-', linewidth=3, label='Selected Path')
        
        # Draw street boundaries manually (since grid might not show)
        street_boundaries = [
            # Main street boundaries
            [(-70, 5), (80, 5)],    # North boundary
            [(-70, -5), (80, -5)],  # South boundary
            # Side street boundaries
            [(17, -40), (17, 40)],  # West boundary
            [(23, -40), (23, 40)],  # East boundary
        ]
        
        for boundary in street_boundaries:
            plt.plot([boundary[0][0], boundary[1][0]], 
                    [boundary[0][1], boundary[1][1]], 'k-', linewidth=2, alpha=0.8)
        
        # Draw some obstacles manually
        static_obstacles = [
            (10, 5), (40, -5), (60, 2), (17, 15), (23, -20)
        ]
        
        for x, y in static_obstacles:
            rect = plt.Rectangle((x-2, y-0.9), 4, 1.8, 
                               facecolor='gray', alpha=0.8, zorder=3)
            plt.gca().add_patch(rect)
        
        # Clean view
        plt.xlim(-75, 85)
        plt.ylim(-55, 55)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.legend(loc='upper right')
        plt.title(f'Clean Advanced Vehicle Simulation - Time: {current_time:.1f}s', fontsize=16)
        
        plt.pause(0.05)
        
    except Exception as e:
        print(f"Rendering error: {e}")

def simple_render_fallback(vehicle, dynamic_obstacles, current_time):
    """Simple fallback rendering if all else fails"""
    try:
        plt.clf()
        
        # Simple background
        plt.gca().set_facecolor('lightblue')
        
        # Vehicle
        vehicle_x, vehicle_y = vehicle.state[0], vehicle.state[1]
        plt.plot(vehicle_x, vehicle_y, 'bo', markersize=10, label='Vehicle')
        
        # Goal
        if hasattr(vehicle, 'goal') and vehicle.goal is not None:
            plt.plot(vehicle.goal[0], vehicle.goal[1], 'g*', markersize=15, label='Goal')
        
        # Dynamic obstacles
        for obstacle in dynamic_obstacles:
            plt.plot(obstacle.x, obstacle.y, 'ro', markersize=8, label='Obstacle')
        
        # Simple road
        plt.plot([-70, 80], [5, 5], 'k-', linewidth=3)    # North boundary
        plt.plot([-70, 80], [-5, -5], 'k-', linewidth=3)  # South boundary
        plt.plot([0, 0], [-2, 2], 'y--', linewidth=2)     # Center line
        
        plt.xlim(-75, 85)
        plt.ylim(-55, 55)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.legend()
        plt.title(f'Simple Vehicle Simulation - Time: {current_time:.1f}s', fontsize=16)
        
        plt.pause(0.05)
        
    except Exception as e:
        print(f"Even fallback rendering failed: {e}")

def run_clean_simulation(vehicle, occupancy_grid, planner, dynamic_obstacles, sim_config):
    """Run clean simulation with improved error handling"""
    dt = sim_config['dt']
    max_time = sim_config['max_time']
    current_time = 0.0
    
    print("Clean simulation running... Press Ctrl+C to stop")
    
    # Large figure
    plt.figure(figsize=(20, 14))
    
    try:
        while current_time < max_time:
            step_start_time = time.time()
            
            # Update dynamic obstacles
            for obstacle in dynamic_obstacles:
                obstacle.update(dt)
                
                # Reset if out of bounds
                if obstacle.vy != 0 and abs(obstacle.y) > 12:
                    obstacle.y = -10 if obstacle.y > 0 else 10
                if obstacle.vx != 0 and obstacle.x > 75:
                    obstacle.x = -50
            
            # Vehicle bounds check
            vehicle_x, vehicle_y = vehicle.state[0], vehicle.state[1]
            if vehicle_x < -70 or vehicle_x > 75 or abs(vehicle_y) > 10:
                print(f"Vehicle out of bounds: ({vehicle_x:.1f}, {vehicle_y:.1f})")
                break
            
            # Planning
            trajectory = None
            try:
                trajectory = planner.plan(vehicle.state, vehicle.goal, occupancy_grid)
            except Exception as e:
                print(f"Planning error: {e}")
            
            # Control
            if hasattr(vehicle, 'control') and trajectory is not None:
                vehicle.control(trajectory)
            
            # Update vehicle
            vehicle.update_dynamics(dt)
            
            # Render with fallback
            try:
                clean_render(vehicle, occupancy_grid, planner, trajectory, dynamic_obstacles, current_time)
            except Exception as render_error:
                print(f"Main render failed: {render_error}, using fallback")
                simple_render_fallback(vehicle, dynamic_obstacles, current_time)
            
            # Goal check
            if vehicle.goal is not None:
                distance_to_goal = np.linalg.norm(vehicle.state[:2] - vehicle.goal[:2])
                if distance_to_goal < 3.0:
                    print(f"Goal reached at {current_time:.1f}s!")
                    time.sleep(2)
                    break
            
            # Progress
            if int(current_time * 10) % 30 == 0:
                print(f"T:{current_time:.1f}s Pos:({vehicle.state[0]:.1f},{vehicle.state[1]:.1f}) Speed:{vehicle.state[3]:.1f}")
            
            current_time += dt
            
            # Real-time
            step_time = time.time() - step_start_time
            sleep_time = max(0, dt - step_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    
    print("Clean simulation completed!")
    input("Press Enter to close...")

def main():
    print("Clean Advanced Vehicle Simulation")
    print("=" * 40)
    
    print("Testing imports...")
    if not test_advanced_imports():
        print("Import test failed.")
        return
    
    # Import modules
    from vehicle.autonomous_vehicle import AutonomousVehicle
    from planning.lattice_planner import LatticePlanner
    from mapping.occupancy_grid import OccupancyGrid
    
    # Get config
    config = get_clean_config()
    
    try:
        print("Initializing clean simulation...")
        
        # Components
        occupancy_grid = OccupancyGrid(config['occupancy_grid'])
        print("✓ Occupancy grid created")
        
        # Create map
        create_clean_street_map(occupancy_grid)
        print("✓ Clean street map created")
        
        planner = LatticePlanner(config['lattice_planner'])
        print("✓ Lattice planner created")
        
        vehicle = AutonomousVehicle(config['vehicle'], planner)
        vehicle.state = np.array([-40.0, -2.0, 0.0, 8.0])
        print("✓ Vehicle created")
        
        # Dynamic obstacles
        dynamic_obstacles = create_simple_dynamic_obstacles()
        print(f"✓ {len(dynamic_obstacles)} dynamic obstacles created")
        
        # Goal
        goal = np.array([60.0, -2.0, 0.0])
        vehicle.set_goal(goal)
        print("✓ Goal set")
        
        print(f"\nVehicle start: {vehicle.state[:2]}")
        print(f"Goal: {goal[:2]}")
        
        # Run simulation
        run_clean_simulation(vehicle, occupancy_grid, planner, dynamic_obstacles, config['simulation'])
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()