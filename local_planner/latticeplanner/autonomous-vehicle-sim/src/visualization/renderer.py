import matplotlib.pyplot as plt
import numpy as np
from typing import Dict, Any, List, Optional, Tuple
import matplotlib.patches as patches
from matplotlib.colors import LinearSegmentedColormap

class AdvancedRenderer:
    """Advanced renderer for complex simulation visualization"""
    
    def __init__(self, figure_size: Tuple[int, int] = (20, 14)):
        self.figure_size = figure_size
        self.fig = None
        self.ax = None
        
        # Color schemes
        self.colors = {
            'vehicle': '#2E86AB',
            'vehicle_parked': '#A23B72',
            'goal': '#F18F01',
            'parking': '#C73E1D',
            'road': '#5A5A5A',
            'building': '#8B4513',
            'obstacle': '#DC143C',
            'trajectory': '#00FF00',
            'lattice': '#87CEEB',
            'background': '#F5F5DC'
        }
        
        # Initialize matplotlib settings
        self._setup_matplotlib()
    
    def _setup_matplotlib(self):
        """Setup matplotlib for optimal rendering"""
        plt.style.use('default')
        plt.rcParams['figure.facecolor'] = 'white'
        plt.rcParams['axes.facecolor'] = self.colors['background']
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.grid'] = True
        plt.rcParams['grid.alpha'] = 0.3
    
    def initialize_plot(self):
        """Initialize the plot window"""
        self.fig = plt.figure(figsize=self.figure_size)
        self.ax = self.fig.add_subplot(111)
        plt.ion()  # Interactive mode
        
    def render_complete_scene(self, simulation_data: Dict[str, Any]):
        """Render the complete simulation scene"""
        if self.ax is None:
            self.initialize_plot()
        
        # Clear previous frame
        self.ax.clear()
        
        # Set background
        self.ax.set_facecolor(self.colors['background'])
        
        # Render layers in order (back to front)
        self._render_occupancy_grid(simulation_data.get('occupancy_grid'))
        self._render_roads(simulation_data.get('roads', []))
        self._render_buildings(simulation_data.get('buildings', []))
        self._render_static_obstacles(simulation_data.get('static_obstacles', []))
        self._render_trajectories(simulation_data.get('lattice_trajectories', []))
        self._render_selected_trajectory(simulation_data.get('selected_trajectory'))
        self._render_dynamic_obstacles(simulation_data.get('dynamic_obstacles', []))
        self._render_goal_and_parking(simulation_data.get('goal'), simulation_data.get('parking_zone'))
        self._render_vehicle(simulation_data.get('vehicle'))
        self._render_ui_overlay(simulation_data.get('ui_data', {}))
        
        # Set view and labels
        self._configure_view(simulation_data.get('view_bounds', {}))
        
        # Update display
        plt.pause(0.01)
    
    def _render_occupancy_grid(self, occupancy_grid):
        """Render occupancy grid as background"""
        if occupancy_grid is None:
            return
        
        try:
            grid = occupancy_grid.grid
            origin_x = occupancy_grid.origin_x
            origin_y = occupancy_grid.origin_y
            resolution = occupancy_grid.resolution
            
            extent = [origin_x, origin_x + grid.shape[1] * resolution,
                     origin_y, origin_y + grid.shape[0] * resolution]
            
            # Create custom colormap for occupancy grid
            colors_list = ['white', '#ffcccc', '#ff9999', '#ff6666', '#ff0000']
            custom_cmap = LinearSegmentedColormap.from_list('occupancy', colors_list)
            
            self.ax.imshow(grid, extent=extent, origin='lower', 
                          cmap=custom_cmap, alpha=0.6, zorder=1)
        except Exception as e:
            print(f"Error rendering occupancy grid: {e}")
    
    def _render_roads(self, roads: List[Dict[str, Any]]):
        """Render road network"""
        for road in roads:
            self._draw_single_road(road)
    
    def _draw_single_road(self, road: Dict[str, Any]):
        """Draw a single road with lanes and markings"""
        road_color = self.colors['road']
        lane_color = '#FFFF00'
        
        if road['type'].startswith('horizontal'):
            y = road['y']
            width = road['width']
            x_start = road['x_start']
            x_end = road['x_end']
            
            # Road surface
            road_rect = patches.Rectangle(
                (x_start, y - width/2), x_end - x_start, width,
                facecolor=road_color, alpha=0.4, zorder=2
            )
            self.ax.add_patch(road_rect)
            
            # Road boundaries
            self.ax.plot([x_start, x_end], [y + width/2, y + width/2], 
                        color=road_color, linewidth=3, zorder=3)
            self.ax.plot([x_start, x_end], [y - width/2, y - width/2], 
                        color=road_color, linewidth=3, zorder=3)
            
            # Lane markings
            num_lanes = road.get('lanes', 2)
            if num_lanes > 1:
                # Center line
                for x in np.arange(x_start, x_end, 6):
                    self.ax.plot([x, x+3], [y, y], color=lane_color, linewidth=2, zorder=3)
        
        else:  # vertical road
            x = road['x']
            width = road['width']
            y_start = road['y_start']
            y_end = road['y_end']
            
            # Road surface
            road_rect = patches.Rectangle(
                (x - width/2, y_start), width, y_end - y_start,
                facecolor=road_color, alpha=0.4, zorder=2
            )
            self.ax.add_patch(road_rect)
            
            # Road boundaries
            self.ax.plot([x + width/2, x + width/2], [y_start, y_end], 
                        color=road_color, linewidth=3, zorder=3)
            self.ax.plot([x - width/2, x - width/2], [y_start, y_end], 
                        color=road_color, linewidth=3, zorder=3)
            
            # Lane markings
            num_lanes = road.get('lanes', 2)
            if num_lanes > 1:
                for y in np.arange(y_start, y_end, 6):
                    self.ax.plot([x, x], [y, y+3], color=lane_color, linewidth=2, zorder=3)
    
    def _render_buildings(self, buildings: List[Any]):
        """Render buildings"""
        for building in buildings:
            building_rect = patches.Rectangle(
                (building.x - building.width/2, building.y - building.height/2),
                building.width, building.height,
                facecolor=self.colors['building'], alpha=0.8,
                edgecolor='black', linewidth=1, zorder=4
            )
            self.ax.add_patch(building_rect)
            
            # Building label (for larger buildings)
            if building.width > 10 or building.height > 10:
                self.ax.text(building.x, building.y, '🏢', fontsize=12, 
                           ha='center', va='center', zorder=5)
    
    def _render_static_obstacles(self, obstacles: List[Any]):
        """Render static obstacles with different symbols"""
        for obstacle in obstacles:
            color = self.colors['obstacle']
            symbol = '⬛'
            
            # Different rendering based on obstacle type
            if obstacle.obstacle_type == 'parked_car':
                color = '#800080'
                symbol = '🚗'
            elif obstacle.obstacle_type == 'tree':
                color = '#228B22'
                symbol = '🌳'
            elif obstacle.obstacle_type == 'barrier':
                color = '#FFA500'
                symbol = '🚧'
            elif obstacle.obstacle_type == 'pole':
                color = '#696969'
                symbol = '🚩'
            
            # Draw obstacle
            obstacle_rect = patches.Rectangle(
                (obstacle.x - obstacle.width/2, obstacle.y - obstacle.height/2),
                obstacle.width, obstacle.height,
                facecolor=color, alpha=0.7,
                edgecolor='black', linewidth=1, zorder=4
            )
            self.ax.add_patch(obstacle_rect)
            
            # Add symbol
            self.ax.text(obstacle.x, obstacle.y, symbol, fontsize=8, 
                        ha='center', va='center', zorder=5)
    
    def _render_trajectories(self, lattice_trajectories: List[np.ndarray]):
        """Render lattice trajectories"""
        for i, trajectory in enumerate(lattice_trajectories):
            if len(trajectory) > 1:
                x_coords = trajectory[:, 0]
                y_coords = trajectory[:, 1]
                
                self.ax.plot(x_coords, y_coords, color=self.colors['lattice'], 
                           alpha=0.3, linewidth=1, zorder=6)
    
    def _render_selected_trajectory(self, trajectory: Optional[np.ndarray]):
        """Render the selected trajectory"""
        if trajectory is not None and len(trajectory) > 1:
            x_coords = trajectory[:, 0]
            y_coords = trajectory[:, 1]
            
            self.ax.plot(x_coords, y_coords, color=self.colors['trajectory'], 
                        linewidth=4, alpha=0.9, zorder=7, label='Planned Path')
    
    def _render_dynamic_obstacles(self, dynamic_obstacles: List[Dict[str, Any]]):
        """Render dynamic obstacles with motion indicators"""
        for obs_data in dynamic_obstacles:
            self._draw_dynamic_obstacle(obs_data)
    
    def _draw_dynamic_obstacle(self, obs_data: Dict[str, Any]):
        """Draw a single dynamic obstacle"""
        x, y = obs_data['x'], obs_data['y']
        radius = obs_data['radius']
        vx, vy = obs_data['vx'], obs_data['vy']
        obstacle_type = obs_data['type']
        
        # Color and symbol based on type
        type_config = {
            'pedestrian': {'color': '#FF6B6B', 'symbol': '🚶', 'size': 12},
            'cyclist': {'color': '#4ECDC4', 'symbol': '🚴', 'size': 12},
            'slow_vehicle': {'color': '#45B7D1', 'symbol': '🚗', 'size': 14},
            'service_vehicle': {'color': '#96CEB4', 'symbol': '🚚', 'size': 14}
        }
        
        config = type_config.get(obstacle_type, {'color': '#999999', 'symbol': '?', 'size': 10})
        
        # Draw obstacle circle
        circle = patches.Circle((x, y), radius, facecolor=config['color'], 
                              alpha=0.7, edgecolor='black', linewidth=1, zorder=8)
        self.ax.add_patch(circle)
        
        # Velocity arrow
        if abs(vx) > 0.1 or abs(vy) > 0.1:
            arrow_scale = 4.0
            self.ax.annotate('', xy=(x + vx * arrow_scale, y + vy * arrow_scale), 
                           xytext=(x, y),
                           arrowprops=dict(arrowstyle='->', color=config['color'], 
                                         lw=2, alpha=0.8), zorder=8)
        
        # Symbol
        self.ax.text(x, y, config['symbol'], fontsize=config['size'], 
                    ha='center', va='center', zorder=9)
    
    def _render_vehicle(self, vehicle_data: Dict[str, Any]):
        """Render the autonomous vehicle"""
        if vehicle_data is None:
            return
        
        x, y = vehicle_data['position']
        heading = vehicle_data['heading']
        corners = vehicle_data['corners']
        is_parked = vehicle_data.get('is_parked', False)
        speed = vehicle_data.get('speed', 0.0)
        
        # Vehicle body
        vehicle_color = self.colors['vehicle_parked'] if is_parked else self.colors['vehicle']
        
        vehicle_polygon = patches.Polygon(corners, facecolor=vehicle_color, 
                                        alpha=0.9, edgecolor='black', 
                                        linewidth=2, zorder=10)
        self.ax.add_patch(vehicle_polygon)
        
        # Heading direction (arrow)
        arrow_length = 4.0
        self.ax.annotate('', xy=(x + arrow_length * np.cos(heading), 
                               y + arrow_length * np.sin(heading)), 
                       xytext=(x, y),
                       arrowprops=dict(arrowstyle='->', color='red', lw=2, alpha=0.7),
                       zorder=11)
        
        # Speed indication (text)
        if speed > 0.1:
            self.ax.text(x, y - 1, f'🚀 {speed:.1f} m/s', fontsize=10, 
                        ha='center', va='center', color='black', zorder=12)
    
    def _render_ui_overlay(self, ui_data: Dict[str, Any]):
        """Render the UI overlay elements"""
        # Example: Render speedometer, gear indicator, etc.
        speed = ui_data.get('speed', 0.0)
        gear = ui_data.get('gear', 'N')
        
        # Speedometer
        self.ax.text(15, 25, f'Speed: {speed:.1f} m/s', fontsize=12, 
                    ha='left', va='center', color='black', zorder=13)
        
        # Gear indicator
        self.ax.text(15, 10, f'Gear: {gear}', fontsize=12, 
                    ha='left', va='center', color='black', zorder=13)
    
    def _configure_view(self, view_bounds: Dict[str, float]):
        """Set the view limits and aspect ratio"""
        if view_bounds:
            x_min = view_bounds.get('x_min', -50)
            x_max = view_bounds.get('x_max', 50)
            y_min = view_bounds.get('y_min', -50)
            y_max = view_bounds.get('y_max', 50)
            
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(y_min, y_max)
        
        # Equal aspect ratio
        self.ax.set_aspect('equal', adjustable='datalim')