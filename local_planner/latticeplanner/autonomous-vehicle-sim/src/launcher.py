"""Simple launcher script with comprehensive error handling"""
import sys
import os

# Add current directory to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

def check_dependencies():
    """Check if all required dependencies are installed"""
    required_packages = {
        'numpy': 'numpy',
        'matplotlib': 'matplotlib',
        'scipy': 'scipy (optional, but recommended)'
    }
    
    missing_packages = []
    
    for package, display_name in required_packages.items():
        try:
            __import__(package)
            print(f"✓ {display_name} is installed")
        except ImportError:
            missing_packages.append(display_name)
            print(f"✗ {display_name} is missing")
    
    if missing_packages:
        print(f"\nMissing packages: {', '.join(missing_packages)}")
        print("Please install them using:")
        print("pip install numpy matplotlib scipy")
        return False
    
    return True

def launch_simulation():
    """Launch the simulation with error handling"""
    print("Autonomous Vehicle Lattice Planner Simulation")
    print("=" * 50)
    
    # Check dependencies
    print("\nChecking dependencies...")
    if not check_dependencies():
        return False
    
    # Run setup test
    print("\nRunning setup tests...")
    try:
        from test_setup import test_setup
        if not test_setup():
            print("Setup test failed. Please check the errors above.")
            return False
    except Exception as e:
        print(f"Could not run setup test: {e}")
        # Continue anyway
    
    # Launch main simulation
    print("\nLaunching simulation...")
    try:
        from main import main
        main()
        return True
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
        return True
    except Exception as e:
        print(f"Simulation error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    try:
        success = launch_simulation()
        if not success:
            print("\nSimulation failed to start properly.")
            input("Press Enter to exit...")
    except Exception as e:
        print(f"Critical error: {e}")
        input("Press Enter to exit...")