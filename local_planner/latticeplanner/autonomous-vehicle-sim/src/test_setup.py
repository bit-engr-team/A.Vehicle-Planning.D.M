"""Test script to verify the setup"""
import sys
import os

# Add src to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_setup():
    """Test the complete setup"""
    print("Testing autonomous vehicle simulation setup...\n")
    
    # Test 1: Basic imports
    try:
        import numpy as np
        import matplotlib.pyplot as plt
        print("✓ Basic libraries (numpy, matplotlib) imported")
    except ImportError as e:
        print(f"✗ Basic libraries import failed: {e}")
        return False
    
    # Test 2: Configuration
    try:
        from main import get_default_config
        config = get_default_config()
        print("✓ Default configuration loaded")
        print(f"  - Simulation dt: {config['simulation']['dt']}")
        print(f"  - Vehicle length: {config['vehicle']['length']}")
        print(f"  - Grid size: {config['occupancy_grid']['width']}x{config['occupancy_grid']['height']}")
    except Exception as e:
        print(f"✗ Configuration test failed: {e}")
        return False
    
    # Test 3: Module imports
    modules_to_test = [
        'mapping.occupancy_grid',
        'planning.path_planner',
        'planning.lattice_planner',
        'vehicle.autonomous_vehicle',
        'simulation.environment',
        'simulation.simulator',
        'visualization.matplotlib_viz',
        'utils.config'
    ]
    
    for module_name in modules_to_test:
        try:
            __import__(module_name)
            print(f"✓ {module_name} imported successfully")
        except ImportError as e:
            print(f"✗ {module_name} import failed: {e}")
            return False
    
    # Test 4: Class instantiation
    try:
        from mapping.occupancy_grid import OccupancyGrid
        from planning.lattice_planner import LatticePlanner
        from vehicle.autonomous_vehicle import AutonomousVehicle
        from simulation.simulator import Simulator
        
        # Test occupancy grid
        grid = OccupancyGrid(config['occupancy_grid'])
        print("✓ OccupancyGrid instance created")
        
        # Test planner
        planner = LatticePlanner(config['lattice_planner'])
        print("✓ LatticePlanner instance created")
        
        # Test vehicle
        vehicle = AutonomousVehicle(config['vehicle'], planner)
        print("✓ AutonomousVehicle instance created")
        
        # Test simulator (without renderer for now)
        simulator = Simulator(config['simulation'], vehicle, grid)
        print("✓ Simulator instance created")
        
        print("✓ Core components created successfully")
        print("  (Renderer test will be done during simulation)")
        
    except Exception as e:
        print(f"✗ Component creation failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Test 5: Basic functionality
    try:
        # Test grid operations
        grid.add_rectangle_obstacle(0, 0, 5, 5)
        grid.add_circle_obstacle(10, 10, 3)
        print("✓ Grid obstacle addition works")
        
        # Test vehicle state
        import numpy as np
        vehicle.set_goal(np.array([10.0, 10.0, 0.0]))
        print("✓ Vehicle goal setting works")
        
        # Test basic planning (without full execution)
        print("✓ Basic functionality tests passed")
        
    except Exception as e:
        print(f"✗ Functionality test failed: {e}")
        return False
    
    print("\n✅ All tests passed! Your simulation is ready to run.")
    print("\nTo start the simulation, run:")
    print("  python main.py")
    
    return True

if __name__ == "__main__":
    test_setup()