import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import matplotlib.colors as colors

class AdvancedRenderer:
    def __init__(self):
        """Initialize the advanced renderer with lattice visualization"""
        plt.style.use('default')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(16, 12))
        self.fig.suptitle('Autonomous Vehicle - Advanced Lattice Planner', fontsize=16)
        
        # Setup subplots
        self.main_ax = self.axes[0, 0]
        self.lattice_ax = self.axes[0, 1]
        self.dynamic_ax = self.axes[1, 0]
        self.status_ax = self.axes[1, 1]
        
        # Initialize plots
        self._setup_plots()
        
        # Color maps for different trajectory types
        self.trajectory_colors = {
            'maintain': 'blue',
            'left_change': 'green',
            'right_change': 'orange'
        }
        
        plt.ion()
        plt.show()
        
    def _setup_plots(self):
        """Setup initial plot configurations"""
        # Main view
        self.main_ax.set_title('Main View - Vehicle & Environment', fontsize=12)
        self.main_ax.set_xlabel('X (m)')
        self.main_ax.set_ylabel('Y (m)')
        self.main_ax.grid(True, alpha=0.3)
        self.main_ax.set_aspect('equal')
        
        # Lattice view
        self.lattice_ax.set_title('Lattice Structure - All Trajectories', fontsize=12)
        self.lattice_ax.set_xlabel('X (m)')
        self.lattice_ax.set_ylabel('Y (m)')
        self.lattice_ax.grid(True, alpha=0.3)
        self.lattice_ax.set_aspect('equal')
        
        # Dynamic obstacles view
        self.dynamic_ax.set_title('Dynamic Environment Analysis', fontsize=12)
        self.dynamic_ax.set_xlabel('X (m)')
        self.dynamic_ax.set_ylabel('Y (m)')
        self.dynamic_ax.grid(True, alpha=0.3)
        self.dynamic_ax.set_aspect('equal')
        
        # Status view
        self.status_ax.set_title('Vehicle & Planning Status', fontsize=12)
        self.status_ax.axis('off')
        
    def render(self, vehicle, occupancy_grid, planner, selected_trajectory, dynamic_obstacles=None, time=0.0):
        """Main rendering function with lattice visualization"""
        try:
            # Clear previous plots
            for ax in [self.main_ax, self.lattice_ax, self.dynamic_ax, self.status_ax]:
                ax.clear()
            
            self._setup_plots()
            
            # Render main view
            self._render_main_view(vehicle, occupancy_grid, selected_trajectory, dynamic_obstacles)
            
            # Render lattice structure
            self._render_lattice_structure(vehicle, planner)
            
            # Render dynamic analysis
            self._render_dynamic_analysis(vehicle, occupancy_grid, dynamic_obstacles)
            
            # Render status
            self._render_advanced_status(vehicle, planner, time)
            
            plt.draw()
            plt.pause(0.05)
            
        except Exception as e:
            print(f"Rendering error: {e}")
    
    def _render_main_view(self, vehicle, occupancy_grid, selected_trajectory, dynamic_obstacles):
        """Render main view with vehicle and environment"""
        # Render occupancy grid
        extent = [occupancy_grid.origin_x, 
                 occupancy_grid.origin_x + occupancy_grid.width * occupancy_grid.resolution,
                 occupancy_grid.origin_y,
                 occupancy_grid.origin_y + occupancy_grid.height * occupancy_grid.resolution]
        
        self.main_ax.imshow(occupancy_grid.grid, extent=extent, origin='lower', 
                           cmap='gray_r', alpha=0.6, vmin=0, vmax=1)
        
        # Render vehicle
        self._render_vehicle(self.main_ax, vehicle)
        
        # Render goal
        self._render_goal(self.main_ax, vehicle.goal)
        
        # Render selected trajectory
        if selected_trajectory is not None:
            self.main_ax.plot(selected_trajectory[:, 0], selected_trajectory[:, 1], 
                             'r-', linewidth=4, alpha=0.8, label='Selected Path')
            
        # Render dynamic obstacles
        if dynamic_obstacles:
            for obs in dynamic_obstacles:
                if hasattr(obs, 'get_corners'):  # Vehicle obstacle
                    corners = obs.get_corners()
                    vehicle_patch = patches.Polygon(corners, closed=True, 
                                                  facecolor='red', edgecolor='darkred',
                                                  alpha=0.7, linewidth=2)
                    self.main_ax.add_patch(vehicle_patch)
                    
                    # Velocity arrow
                    arrow_length = 3.0
                    self.main_ax.arrow(obs.x, obs.y, obs.vx * arrow_length, obs.vy * arrow_length,
                                      head_width=1.0, head_length=1.5, fc='red', ec='red')
                else:  # Circular obstacle
                    circle = patches.Circle((obs.x, obs.y), obs.radius, 
                                          facecolor='red', alpha=0.7, edgecolor='darkred')
                    self.main_ax.add_patch(circle)
        
        self.main_ax.legend()
        
    def _render_lattice_structure(self, vehicle, planner):
        """Render the complete lattice structure"""
        if not hasattr(planner, 'get_all_trajectories'):
            return
            
        all_trajectories = planner.get_all_trajectories()
        
        # Group trajectories by maneuver type
        maneuver_groups = {
            'maintain': [],
            'left_change': [],
            'right_change': []
        }
        
        for traj_data in all_trajectories:
            maneuver = traj_data.get('maneuver', 'maintain')
            if maneuver in maneuver_groups:
                maneuver_groups[maneuver].append(traj_data['trajectory'])
        
        # Render trajectories by group
        for maneuver, trajectories in maneuver_groups.items():
            color = self.trajectory_colors.get(maneuver, 'gray')
            alpha = 0.6 if maneuver == 'maintain' else 0.4
            linewidth = 2 if maneuver == 'maintain' else 1
            
            for trajectory in trajectories:
                self.lattice_ax.plot(trajectory[:, 0], trajectory[:, 1], 
                                   color=color, alpha=alpha, linewidth=linewidth)
        
        # Render vehicle position
        x, y = vehicle.state[:2]
        self.lattice_ax.plot(x, y, 'ko', markersize=8, label='Vehicle')
        
        # Render goal
        if vehicle.goal is not None:
            self.lattice_ax.plot(vehicle.goal[0], vehicle.goal[1], 'g*', 
                               markersize=15, label='Goal')
        
        # Add legend for maneuvers
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='blue', lw=2, label='Lane Maintaining'),
            Line2D([0], [0], color='green', lw=2, label='Left Lane Change'),
            Line2D([0], [0], color='orange', lw=2, label='Right Lane Change')
        ]
        self.lattice_ax.legend(handles=legend_elements)
        
    def _render_dynamic_analysis(self, vehicle, occupancy_grid, dynamic_obstacles):
        """Render dynamic obstacle analysis"""
        # Render static map lightly
        extent = [occupancy_grid.origin_x, 
                 occupancy_grid.origin_x + occupancy_grid.width * occupancy_grid.resolution,
                 occupancy_grid.origin_y,
                 occupancy_grid.origin_y + occupancy_grid.height * occupancy_grid.resolution]
        
        self.dynamic_ax.imshow(occupancy_grid.grid, extent=extent, origin='lower', 
                              cmap='gray_r', alpha=0.3, vmin=0, vmax=1)
        
        # Vehicle
        self._render_vehicle(self.dynamic_ax, vehicle)
        
        if dynamic_obstacles:
            for i, obs in enumerate(dynamic_obstacles):
                # Current position
                if hasattr(obs, 'get_corners'):
                    corners = obs.get_corners()
                    vehicle_patch = patches.Polygon(corners, closed=True, 
                                                  facecolor='red', alpha=0.6)
                    self.dynamic_ax.add_patch(vehicle_patch)
                else:
                    circle = patches.Circle((obs.x, obs.y), obs.radius, 
                                          facecolor='red', alpha=0.6)
                    self.dynamic_ax.add_patch(circle)
                
                # Predicted trajectory
                prediction_time = 5.0
                prediction_steps = 20
                dt = prediction_time / prediction_steps
                
                pred_x = []
                pred_y = []
                for step in range(prediction_steps):
                    t = step * dt
                    future_x = obs.x + obs.vx * t
                    future_y = obs.y + obs.vy * t
                    pred_x.append(future_x)
                    pred_y.append(future_y)
                
                self.dynamic_ax.plot(pred_x, pred_y, 'r--', alpha=0.7, linewidth=2,
                                    label=f'Obstacle {i+1} Prediction' if i == 0 else "")
                
                # Future positions
                for step in range(0, prediction_steps, 5):
                    t = step * dt
                    future_x = obs.x + obs.vx * t
                    future_y = obs.y + obs.vy * t
                    future_circle = patches.Circle((future_x, future_y), obs.radius, 
                                                 facecolor='red', alpha=0.2)
                    self.dynamic_ax.add_patch(future_circle)
        
        if dynamic_obstacles:
            self.dynamic_ax.legend()
    
    def _render_vehicle(self, ax, vehicle):
        """Render vehicle on specified axes"""
        x, y, theta = vehicle.state[:3]
        
        # Vehicle footprint
        footprint = vehicle.get_footprint()
        vehicle_patch = patches.Polygon(footprint, closed=True, 
                                      facecolor='blue', edgecolor='darkblue', 
                                      linewidth=2, alpha=0.8)
        ax.add_patch(vehicle_patch)
        
        # Direction arrow
        arrow_length = 3.0
        dx = arrow_length * np.cos(theta)
        dy = arrow_length * np.sin(theta)
        ax.arrow(x, y, dx, dy, head_width=0.8, head_length=1.2,
                fc='yellow', ec='orange', linewidth=2)
        
        # Center point
        ax.plot(x, y, 'ro', markersize=6, zorder=5)
    
    def _render_goal(self, ax, goal):
        """Render goal position"""
        if goal is not None:
            ax.plot(goal[0], goal[1], 'g*', markersize=20, 
                   markeredgecolor='darkgreen', markeredgewidth=3)
            
            goal_circle = patches.Circle((goal[0], goal[1]), 3.0, 
                                       facecolor='green', alpha=0.2, 
                                       edgecolor='green', linewidth=2)
            ax.add_patch(goal_circle)
    
    def _render_advanced_status(self, vehicle, planner, time):
        """Render advanced status information"""
        x, y, theta, v = vehicle.state
        
        status_text = f"""SIMULATION TIME: {time:.1f} s

VEHICLE STATE:
Position: ({x:.2f}, {y:.2f}) m
Heading: {np.degrees(theta):.1f}°
Speed: {v:.2f} m/s

CONTROL:
Steering: {np.degrees(vehicle.steering_angle):.1f}°
Acceleration: {vehicle.acceleration:.2f} m/s²

PLANNING:"""

        if hasattr(planner, 'get_all_trajectories'):
            all_trajs = planner.get_all_trajectories()
            maintain_count = sum(1 for t in all_trajs if t.get('maneuver') == 'maintain')
            left_count = sum(1 for t in all_trajs if t.get('maneuver') == 'left_change')
            right_count = sum(1 for t in all_trajs if t.get('maneuver') == 'right_change')
            
            status_text += f"""
Total Trajectories: {len(all_trajs)}
- Lane Keeping: {maintain_count}
- Left Changes: {left_count}
- Right Changes: {right_count}"""

        if vehicle.goal is not None:
            goal_distance = np.linalg.norm(vehicle.state[:2] - vehicle.goal[:2])
            status_text += f"""

GOAL:
Distance: {goal_distance:.2f} m
Position: ({vehicle.goal[0]:.1f}, {vehicle.goal[1]:.1f})"""
        else:
            status_text += "\n\nGOAL: Not set"
            
        self.status_ax.text(0.05, 0.95, status_text, transform=self.status_ax.transAxes,
                           fontsize=10, verticalalignment='top', fontfamily='monospace',
                           bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    def close(self):
        """Close the renderer"""
        try:
            plt.close(self.fig)
        except:
            pass