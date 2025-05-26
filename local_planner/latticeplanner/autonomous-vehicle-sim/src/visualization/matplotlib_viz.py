import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

class MatplotlibRenderer:
    def __init__(self):
        """Initialize the matplotlib renderer"""
        plt.style.use('default')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 12))
        self.fig.suptitle('Autonomous Vehicle Simulation - Lattice Planner')
        
        # Setup subplots
        self.map_ax = self.axes[0, 0]
        self.trajectory_ax = self.axes[0, 1]
        self.sensor_ax = self.axes[1, 0]
        self.status_ax = self.axes[1, 1]
        
        # Initialize plots
        self._setup_plots()
        
        # Enable interactive mode
        plt.ion()
        plt.show()
        
    def _setup_plots(self):
        """Setup initial plot configurations"""
        # Map view
        self.map_ax.set_title('Occupancy Grid & Vehicle')
        self.map_ax.set_xlabel('X (m)')
        self.map_ax.set_ylabel('Y (m)')
        self.map_ax.grid(True, alpha=0.3)
        self.map_ax.set_aspect('equal')
        
        # Trajectory view
        self.trajectory_ax.set_title('Planned Trajectory')
        self.trajectory_ax.set_xlabel('X (m)')
        self.trajectory_ax.set_ylabel('Y (m)')
        self.trajectory_ax.grid(True, alpha=0.3)
        self.trajectory_ax.set_aspect('equal')
        
        # Sensor view
        self.sensor_ax.set_title('Vehicle Details')
        self.sensor_ax.set_xlabel('X (m)')
        self.sensor_ax.set_ylabel('Y (m)')
        self.sensor_ax.grid(True, alpha=0.3)
        self.sensor_ax.set_aspect('equal')
        
        # Status view
        self.status_ax.set_title('Vehicle Information')
        self.status_ax.axis('off')
        
    def render(self, vehicle, occupancy_grid, trajectory=None, time=0.0):
        """Main rendering function"""
        try:
            # Clear previous plots
            self.map_ax.clear()
            self.trajectory_ax.clear()
            self.sensor_ax.clear()
            self.status_ax.clear()
            
            # Re-setup plots
            self._setup_plots()
            
            # Render components
            self._render_occupancy_grid(occupancy_grid)
            self._render_vehicle(vehicle)
            self._render_goal(vehicle.goal)
            
            if trajectory is not None:
                self._render_trajectory(trajectory)
                
            self._render_status(vehicle, time)
            
            # Update display
            plt.draw()
            plt.pause(0.01)
            
        except Exception as e:
            print(f"Rendering error: {e}")
        
    def _render_occupancy_grid(self, occupancy_grid):
        """Render occupancy grid"""
        try:
            extent = [occupancy_grid.origin_x, 
                     occupancy_grid.origin_x + occupancy_grid.width * occupancy_grid.resolution,
                     occupancy_grid.origin_y,
                     occupancy_grid.origin_y + occupancy_grid.height * occupancy_grid.resolution]
            
            self.map_ax.imshow(occupancy_grid.grid, extent=extent, origin='lower', 
                              cmap='gray_r', alpha=0.7, vmin=0, vmax=1)
            
            self.trajectory_ax.imshow(occupancy_grid.grid, extent=extent, origin='lower',
                                     cmap='gray_r', alpha=0.5, vmin=0, vmax=1)
        except Exception as e:
            print(f"Grid rendering error: {e}")
        
    def _render_vehicle(self, vehicle):
        """Render vehicle"""
        try:
            x, y, theta = vehicle.state[:3]
            
            # Get vehicle footprint
            footprint = vehicle.get_footprint()
            
            # Vehicle body
            vehicle_patch = patches.Polygon(footprint, closed=True, 
                                          facecolor='blue', edgecolor='darkblue', 
                                          linewidth=2, alpha=0.8)
            self.map_ax.add_patch(vehicle_patch)
            
            # Vehicle direction arrow
            arrow_length = 3.0
            dx = arrow_length * np.cos(theta)
            dy = arrow_length * np.sin(theta)
            self.map_ax.arrow(x, y, dx, dy, head_width=0.5, head_length=0.8,
                             fc='red', ec='red', linewidth=2)
            
            # Vehicle center point
            self.map_ax.plot(x, y, 'ro', markersize=6, zorder=5)
            
        except Exception as e:
            print(f"Vehicle rendering error: {e}")
        
    def _render_goal(self, goal):
        """Render goal position"""
        try:
            if goal is not None:
                self.map_ax.plot(goal[0], goal[1], 'g*', markersize=15, 
                               markeredgecolor='darkgreen', markeredgewidth=2)
                
                # Goal circle
                goal_circle = patches.Circle((goal[0], goal[1]), 2.0, 
                                           facecolor='green', alpha=0.2, 
                                           edgecolor='green', linewidth=2)
                self.map_ax.add_patch(goal_circle)
        except Exception as e:
            print(f"Goal rendering error: {e}")
            
    def _render_trajectory(self, trajectory):
        """Render planned trajectory"""
        try:
            if len(trajectory) > 1:
                self.trajectory_ax.plot(trajectory[:, 0], trajectory[:, 1], 
                                      'b-', linewidth=3, alpha=0.8, label='Planned Path')
                
                # Trajectory points
                self.trajectory_ax.plot(trajectory[:, 0], trajectory[:, 1], 
                                      'bo', markersize=4, alpha=0.6)
                    
                self.trajectory_ax.legend()
        except Exception as e:
            print(f"Trajectory rendering error: {e}")
            
    def _render_status(self, vehicle, time):
        """Render vehicle status information"""
        try:
            x, y, theta, v = vehicle.state
            
            status_text = f"""Time: {time:.1f} s

Vehicle State:
Position: ({x:.2f}, {y:.2f}) m
Heading: {np.degrees(theta):.1f}°
Speed: {v:.2f} m/s

Control:
Steering: {np.degrees(vehicle.steering_angle):.1f}°
Acceleration: {vehicle.acceleration:.2f} m/s²

Goal:"""
            
            if vehicle.goal is not None:
                goal_distance = np.linalg.norm(vehicle.state[:2] - vehicle.goal[:2])
                status_text += f"\nDistance: {goal_distance:.2f} m"
                status_text += f"\nPosition: ({vehicle.goal[0]:.1f}, {vehicle.goal[1]:.1f})"
            else:
                status_text += "\nNo goal set"
                
            self.status_ax.text(0.05, 0.95, status_text, transform=self.status_ax.transAxes,
                               fontsize=10, verticalalignment='top', fontfamily='monospace',
                               bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
        except Exception as e:
            print(f"Status rendering error: {e}")
        
    def close(self):
        """Close the renderer"""
        try:
            plt.close(self.fig)
        except:
            pass


# Legacy functions for backward compatibility
def draw_vehicle(ax, vehicle_position, vehicle_orientation):
    """Legacy function - draws the vehicle on the given axes."""
    import numpy as np
    vehicle_length = 1.0
    vehicle_width = 0.5

    # Calculate the corners of the vehicle rectangle
    corners = [
        (vehicle_position[0] - vehicle_length / 2, vehicle_position[1] - vehicle_width / 2),
        (vehicle_position[0] + vehicle_length / 2, vehicle_position[1] - vehicle_width / 2),
        (vehicle_position[0] + vehicle_length / 2, vehicle_position[1] + vehicle_width / 2),
        (vehicle_position[0] - vehicle_length / 2, vehicle_position[1] + vehicle_width / 2),
    ]

    # Rotate corners based on vehicle orientation
    rotated_corners = []
    for corner in corners:
        x_rotated = (corner[0] - vehicle_position[0]) * np.cos(vehicle_orientation) - (corner[1] - vehicle_position[1]) * np.sin(vehicle_orientation) + vehicle_position[0]
        y_rotated = (corner[0] - vehicle_position[0]) * np.sin(vehicle_orientation) + (corner[1] - vehicle_position[1]) * np.cos(vehicle_orientation) + vehicle_position[1]
        rotated_corners.append((x_rotated, y_rotated))

    # Create a polygon for the vehicle
    vehicle_polygon = plt.Polygon(rotated_corners, closed=True, color='blue')
    ax.add_patch(vehicle_polygon)

def draw_grid(ax, occupancy_grid):
    """Legacy function - draws the occupancy grid on the given axes."""
    grid_size = occupancy_grid.shape
    for x in range(grid_size[0]):
        for y in range(grid_size[1]):
            if occupancy_grid[x, y] == 1:  # Occupied
                ax.add_patch(plt.Rectangle((x, y), 1, 1, color='red', alpha=0.5))
            else:  # Free
                ax.add_patch(plt.Rectangle((x, y), 1, 1, color='green', alpha=0.5))

def show_simulation(occupancy_grid, vehicle_position, vehicle_orientation):
    """Legacy function - displays the simulation with the occupancy grid and vehicle."""
    import numpy as np
    fig, ax = plt.subplots()
    draw_grid(ax, occupancy_grid)
    draw_vehicle(ax, vehicle_position, vehicle_orientation)

    ax.set_xlim(0, occupancy_grid.shape[0])
    ax.set_ylim(0, occupancy_grid.shape[1])
    ax.set_aspect('equal')
    plt.title('Autonomous Vehicle Simulation')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid()
    plt.show()


# Test the class creation
if __name__ == "__main__":
    print("MatplotlibRenderer class defined successfully!")
    try:
        renderer = MatplotlibRenderer()
        print("MatplotlibRenderer instance created successfully!")
        import time
        time.sleep(2)
        renderer.close()
        print("Test completed successfully!")
    except Exception as e:
        print(f"Error creating renderer: {e}")