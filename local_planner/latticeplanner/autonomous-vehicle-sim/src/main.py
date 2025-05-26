import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Add the src directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_imports():
    """Test all imports before main execution"""
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
        
    try:
        from utils.config import load_config
        print("✓ Config loader imported successfully")
    except ImportError as e:
        print(f"✗ Config loader import failed: {e}")
        return False
        
    return True

def get_default_config():
    """Get default configuration dictionary"""
    return {
        'simulation': {
            'dt': 0.1,
            'max_time': 100.0,
            'grid_resolution': 0.5
        },
        'vehicle': {
            'length': 4.5,
            'width': 2.0,
            'wheelbase': 2.7,
            'max_speed': 15.0,
            'max_acceleration': 3.0,
            'max_deceleration': -5.0,
            'max_steering_angle': 0.6
        },
        'lattice_planner': {
            'prediction_horizon': 5.0,
            'lateral_resolution': 1.0,
            'speed_resolution': 2.0,
            'max_lateral_offset': 6.0,
            'trajectory_length': 50
        },
        'occupancy_grid': {
            'width': 200,
            'height': 200,
            'resolution': 0.5,
            'origin_x': -50.0,
            'origin_y': -50.0
        }
    }

def main():
    print("Testing imports...")
    if not test_imports():
        print("Import test failed. Please check the error messages above.")
        return
    
    # Import after testing
    from simulation.simulator import Simulator
    from vehicle.autonomous_vehicle import AutonomousVehicle
    from planning.lattice_planner import LatticePlanner
    from mapping.occupancy_grid import OccupancyGrid
    from visualization.matplotlib_viz import MatplotlibRenderer
    from utils.config import load_config
    
    # Load configuration from the parent directory
    config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config.yaml')
    
    # Check if config file exists
    if not os.path.exists(config_path):
        print(f"Config file not found at: {config_path}")
        print("Creating default configuration...")
        create_default_config(config_path)
    
    # Load configuration with fallback
    try:
        config = load_config(config_path)
        print("✓ Configuration loaded successfully")
    except Exception as e:
        print(f"Error loading config: {e}")
        print("Using default configuration...")
        config = get_default_config()
    
    # Check if config is empty or missing required keys
    required_keys = ['simulation', 'vehicle', 'lattice_planner', 'occupancy_grid']
    if not config or not all(key in config for key in required_keys):
        print("Configuration is incomplete. Using default configuration...")
        config = get_default_config()
    
    try:
        print("Initializing components...")
        
        # Initialize components with error checking
        occupancy_grid = OccupancyGrid(config['occupancy_grid'])
        print("✓ Occupancy grid created")
        
        planner = LatticePlanner(config['lattice_planner'])
        print("✓ Lattice planner created")
        
        vehicle = AutonomousVehicle(config['vehicle'], planner)
        print("✓ Vehicle created")
        
        renderer = MatplotlibRenderer()
        print("✓ Renderer created")
        
        # Create simulator with correct argument order
        simulator = Simulator(config['simulation'], vehicle, occupancy_grid, renderer)
        print("✓ Simulator created")
        
        # Add some obstacles
        simulator.add_obstacles()
        print("✓ Obstacles added")
        
        # Set goal
        goal = np.array([40.0, 30.0, 0.0])  # x, y, theta
        vehicle.set_goal(goal)
        print("✓ Goal set")
        
        print("\nStarting autonomous vehicle simulation...")
        print(f"Vehicle starting position: {vehicle.state[:2]}")
        print(f"Goal position: {goal[:2]}")
        
        # Run simulation
        simulator.run()
        
    except Exception as e:
        print(f"Error during simulation: {e}")
        import traceback
        traceback.print_exc()

def create_default_config(config_path):
    """Create a default configuration file if it doesn't exist"""
    try:
        import yaml
    except ImportError:
        print("PyYAML not installed. Please run: pip install pyyaml")
        return
    
    default_config = get_default_config()
    
    # Create config directory if it doesn't exist
    try:
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        
        with open(config_path, 'w') as f:
            yaml.dump(default_config, f, default_flow_style=False, indent=2)
        
        print(f"Default configuration created at: {config_path}")
    except Exception as e:
        print(f"Error creating config file: {e}")

if __name__ == "__main__":
    main()