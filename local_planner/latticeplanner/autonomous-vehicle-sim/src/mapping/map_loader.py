def load_map(file_path):
    import numpy as np
    from .occupancy_grid import OccupancyGrid

    try:
        map_data = np.loadtxt(file_path, delimiter=',')
        occupancy_grid = OccupancyGrid(map_data.shape[0], map_data.shape[1])
        occupancy_grid.update_grid(map_data)
        return occupancy_grid
    except Exception as e:
        print(f"Error loading map from {file_path}: {e}")
        return None