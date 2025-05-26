# Autonomous Vehicle Simulation

This project is a modular and realistic simulation of an autonomous vehicle using a lattice planner and occupancy grid mapping. The simulation includes 2.5D visualization using Matplotlib and is designed to be extensible for further development.

## Project Structure

```
autonomous-vehicle-sim
├── src
│   ├── main.py                # Entry point of the simulation
│   ├── vehicle
│   │   ├── __init__.py        # Marks the vehicle directory as a package
│   │   ├── autonomous_vehicle.py # Class for controlling the autonomous vehicle
│   │   └── vehicle_dynamics.py  # Class for simulating vehicle dynamics
│   ├── planning
│   │   ├── __init__.py        # Marks the planning directory as a package
│   │   ├── lattice_planner.py  # Implements the lattice planning algorithm
│   │   └── path_planner.py     # Creates a path based on the occupancy grid
│   ├── mapping
│   │   ├── __init__.py        # Marks the mapping directory as a package
│   │   ├── occupancy_grid.py    # Represents the occupancy grid map
│   │   └── map_loader.py        # Loads a map from a file
│   ├── visualization
│   │   ├── __init__.py        # Marks the visualization directory as a package
│   │   ├── matplotlib_viz.py    # Functions for visualizing the simulation
│   │   └── plotter_2d5.py       # Class for 2.5D visualization
│   ├── simulation
│   │   ├── __init__.py        # Marks the simulation directory as a package
│   │   ├── simulator.py         # Manages the simulation loop
│   │   └── environment.py       # Represents the simulation environment
│   └── utils
│       ├── __init__.py        # Marks the utils directory as a package
│       └── config.py           # Functions for loading configuration settings
├── data
│   └── maps                    # Directory containing map files
├── requirements.txt            # Python dependencies for the project
├── config.yaml                 # Configuration settings for the simulation
└── README.md                   # Documentation for the project
```

## Installation

1. Clone the repository:
   ```
   git clone https://github.com/yourusername/autonomous-vehicle-sim.git
   cd autonomous-vehicle-sim
   ```

2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

3. Configure the simulation settings in `config.yaml` as needed.

## Usage

To run the simulation, execute the following command:
```
python src/main.py
```

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.