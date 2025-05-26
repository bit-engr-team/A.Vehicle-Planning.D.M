class Plotter2D5:
    def __init__(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def plot(self, occupancy_grid, vehicle_position):
        self.ax.clear()
        self.ax.set_title('2.5D Visualization of Autonomous Vehicle Simulation')

        # Plot occupancy grid
        x, y = occupancy_grid.get_grid_coordinates()
        z = occupancy_grid.get_heights()
        self.ax.plot_surface(x, y, z, alpha=0.5, rstride=100, cstride=100)

        # Plot vehicle position
        self.ax.scatter(vehicle_position[0], vehicle_position[1], vehicle_position[2], color='r', s=100)

        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.set_zlabel('Height')
        plt.draw()
        plt.pause(0.01)

    def show(self):
        import matplotlib.pyplot as plt
        plt.show()