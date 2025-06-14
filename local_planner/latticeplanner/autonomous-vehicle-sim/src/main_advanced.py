import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import time
import random
from typing import Dict, Any, List, Tuple, Optional

# Add the src directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def import_modules():
    """Import all required modules"""
    try:
        from utils.randomizer import create_random_scenario, ComplexEnvironmentGenerator
        from mapping.occupancy_grid import OccupancyGrid
        from planning.lattice_planner import LatticePlanner
        from vehicle.autonomous_vehicle import AutonomousVehicle
        from simulation.dynamic_obstacles import DynamicObstacleManager
        
        print("✅ All modules imported successfully")
        return True, {
            'randomizer': create_random_scenario,
            'env_generator': ComplexEnvironmentGenerator,
            'occupancy_grid': OccupancyGrid,
            'planner': LatticePlanner,
            'vehicle': AutonomousVehicle,
            'dynamic_manager': DynamicObstacleManager
        }
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("💡 Make sure all required files are in place")
        return False, {}

class AdvancedSimulationManager:
    """Advanced simulation manager with complex environments"""
    
    def __init__(self):
        self.scenario = None
        self.occupancy_grid = None
        self.vehicle = None
        self.planner = None
        self.dynamic_manager = None
        self.simulation_stats = {}
        
        # Simulation parameters
        self.config = {
            'simulation': {
                'dt': 0.1,
                'max_time': 180.0,  # 3 minutes max
                'real_time_factor': 1.0
            },
            'vehicle': {
                'length': 4.5,
                'width': 2.0,
                'wheelbase': 2.7,
                'max_speed': 15.0,
                'max_acceleration': 3.0,
                'max_deceleration': -4.0,
                'max_steering_angle': 0.6
            },
            'lattice_planner': {
                'prediction_horizon': 4.0,
                'lateral_resolution': 2.5,
                'speed_resolution': 2.0,
                'max_lateral_offset': 5.0,
                'trajectory_length': 20
            }
        }
        
        # Performance monitoring
        self.frame_times = []
        self.planning_times = []
        
        print("🚀 AdvancedSimulationManager initialized")
    
    def create_new_scenario(self) -> bool:
        """Create a new random scenario"""
        try:
            print("\n" + "="*60)
            print("🎲 GENERATING NEW RANDOM SCENARIO")
            print("="*60)
            
            # Import modules
            success, modules = import_modules()
            if not success:
                return False
            
            # Create random scenario
            self.scenario = modules['randomizer']()
            
            # Create occupancy grid
            self.occupancy_grid = modules['occupancy_grid'](self.scenario['grid_config'])
            self.occupancy_grid.populate_from_scenario(self.scenario['city_data'])
            
            # Create lattice planner
            self.planner = modules['planner'](self.config['lattice_planner'])
            
            # Create vehicle
            self.vehicle = modules['vehicle'](self.config['vehicle'], self.planner)
            
            # Set vehicle initial state
            start_pos = self.scenario['start_position']
            self.vehicle.state = np.array([start_pos[0], start_pos[1], start_pos[2], 
                                         random.uniform(3.0, 6.0), 0.0, 0.0])
            
            # Set goal with parking
            self.vehicle.set_goal(self.scenario['goal_position'], self.scenario['parking_zone'])
            
            # Create dynamic obstacle manager
            self.dynamic_manager = modules['dynamic_manager']()
            self.dynamic_manager.add_obstacles_from_scenario(self.scenario['city_data']['dynamic_obstacles'])
            
            # Initialize stats
            self.simulation_stats = {
                'start_time': time.time(),
                'total_distance': 0.0,
                'goal_reached': False,
                'parked': False,
                'collisions': 0,
                'planning_failures': 0
            }
            
            print(f"✅ New scenario created successfully!")
            print(f"   🚗 Vehicle start: ({start_pos[0]:.1f}, {start_pos[1]:.1f})")
            print(f"   🎯 Goal: ({self.scenario['goal_position'][0]:.1f}, {self.scenario['goal_position'][1]:.1f})")
            print(f"   🅿️ Parking: ({self.scenario['parking_zone'].x:.1f}, {self.scenario['parking_zone'].y:.1f})")
            print(f"   🏢 {len(self.scenario['city_data']['buildings'])} buildings")
            print(f"   🚧 {len(self.scenario['city_data']['obstacles'])} static obstacles")
            print(f"   🚶 {len(self.scenario['city_data']['dynamic_obstacles'])} dynamic obstacles")
            
            return True
            
        except Exception as e:
            print(f"❌ Error creating scenario: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def run_simulation(self):
        """Run the main simulation loop"""
        if not self.scenario:
            print("❌ No scenario loaded. Create a scenario first.")
            return
        
        print("\n" + "="*60)
        print("🎮 STARTING ADVANCED SIMULATION")
        print("="*60)
        print("📝 Controls:")
        print("   - Press Ctrl+C to stop simulation")
        print("   - Close plot window to stop")
        print("   - Simulation will auto-stop when parked or time limit reached")
        print("="*60)
        
        # Setup matplotlib
        plt.ion()  # Interactive mode
        fig = plt.figure(figsize=(20, 14))
        plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)
        
        # Simulation variables
        dt = self.config['simulation']['dt']
        max_time = self.config['simulation']['max_time']
        current_time = 0.0
        frame_count = 0
        last_render_time = time.time()
        
        try:
            while current_time < max_time:
                frame_start_time = time.time()
                
                # Update dynamic obstacles
                self.dynamic_manager.update_all(dt)
                
                # Check if vehicle is parked
                if self.vehicle.is_parked():
                    if not self.simulation_stats['parked']:
                        self.simulation_stats['parked'] = True
                        self.simulation_stats['parking_time'] = current_time
                        print(f"🎉 VEHICLE SUCCESSFULLY PARKED at {current_time:.1f}s!")
                    
                    # Continue for a few seconds after parking to show result
                    if current_time - self.simulation_stats.get('parking_time', 0) > 5.0:
                        break
                
                # Check goal reached
                if not self.simulation_stats['goal_reached']:
                    goal_distance = np.linalg.norm(self.vehicle.state[:2] - self.scenario['goal_position'][:2])
                    if goal_distance < 3.0:
                        self.simulation_stats['goal_reached'] = True
                        print(f"🎯 Goal reached at {current_time:.1f}s!")
                
                # Planning
                planning_start = time.time()
                try:
                    trajectory = self.planner.plan(
                        self.vehicle.state, 
                        self.scenario['goal_position'], 
                        self.occupancy_grid
                    )
                    
                    # Check trajectory collision with dynamic obstacles
                    if trajectory is not None:
                        if self.dynamic_manager.check_collision_with_trajectory(trajectory, safety_margin=3.0):
                            trajectory = None  # Force replanning or emergency stop
                    
                except Exception as e:
                    print(f"⚠️ Planning error: {e}")
                    trajectory = None
                    self.simulation_stats['planning_failures'] += 1
                
                planning_time = time.time() - planning_start
                self.planning_times.append(planning_time)
                
                # Vehicle control and update
                self.vehicle.control(trajectory)
                self.vehicle.update_dynamics(dt)
                
                # Check boundaries (simple check)
                vehicle_pos = self.vehicle.get_position()
                if (abs(vehicle_pos[0]) > 150 or abs(vehicle_pos[1]) > 120):
                    print("⚠️ Vehicle went out of bounds!")
                    break
                
                # Rendering (every 3rd frame for performance)
                current_render_time = time.time()
                if frame_count % 3 == 0 or current_render_time - last_render_time > 0.2:
                    self.render_simulation(current_time, trajectory)
                    last_render_time = current_render_time
                
                # Performance monitoring
                frame_time = time.time() - frame_start_time
                self.frame_times.append(frame_time)
                
                # Progress reporting
                if frame_count % 50 == 0:
                    self._print_progress(current_time, frame_time, planning_time)
                
                # Time management
                current_time += dt
                frame_count += 1
                
                # Real-time simulation
                sleep_time = max(0, dt - frame_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                # Check if plot window is closed
                if not plt.get_fignums():
                    print("📊 Plot window closed, stopping simulation")
                    break
                    
        except KeyboardInterrupt:
            print("\n⏹️ Simulation stopped by user")
        except Exception as e:
            print(f"❌ Simulation error: {e}")
            import traceback
            traceback.print_exc()
        
        # Finalize simulation
        self._finalize_simulation(current_time)
    
    def render_simulation(self, current_time: float, trajectory: Optional[np.ndarray]):
        """Render the simulation"""
        try:
            plt.clf()
            
            # Set background
            plt.gca().set_facecolor('#f5f5f5')
            
            # Render occupancy grid
            self._render_occupancy_grid()
            
            # Render city elements
            self._render_city_elements()
            
            # Render dynamic obstacles
            self._render_dynamic_obstacles()
            
            # Render vehicle
            self._render_vehicle()
            
            # Render trajectories
            self._render_trajectories(trajectory)
            
            # Render goal and parking
            self._render_goal_and_parking()
            
            # Render UI elements
            self._render_ui(current_time)
            
          
            # Set MEDIUM view for larger world
            plt.xlim(-180, 180)  # Biraz daha büyük (-100 -> -120)
            plt.ylim(-180, 180)  # Biraz daha büyük (-100 -> -120)
            plt.grid(True, alpha=0.2, color='white')
            plt.axis('equal')
            
            # Title
            status = "PARKED ✅" if self.vehicle.is_parked() else "DRIVING 🚗"
            plt.title(f'City Simulation - {status}', fontsize=20, weight='bold')
            
            plt.pause(0.01)
            
        except Exception as e:
            print(f"Rendering error: {e}")
    
    def _render_occupancy_grid(self):
        """Render occupancy grid"""
        try:
            if hasattr(self.occupancy_grid, 'grid'):
                grid = self.occupancy_grid.grid
                origin_x = self.occupancy_grid.origin_x
                origin_y = self.occupancy_grid.origin_y
                resolution = self.occupancy_grid.resolution
                
                extent = [origin_x, origin_x + grid.shape[1] * resolution,
                         origin_y, origin_y + grid.shape[0] * resolution]
                
                plt.imshow(grid, extent=extent, origin='lower', 
                          cmap='Reds', alpha=0.6, zorder=1)
        except:
            pass
    
    def _render_city_elements(self):
        """Render city roads and buildings"""
        # Roads
        for road in self.scenario['city_data']['roads']:
            self._draw_road(road)
        
        # Buildings (already in occupancy grid, but add labels)
        for i, building in enumerate(self.scenario['city_data']['buildings'][:20]):  # Limit for performance
            if i % 3 == 0:  # Only label every 3rd building
                plt.text(building.x, building.y, '🏢', fontsize=8, ha='center', va='center', alpha=0.7)
    
    def _draw_road(self, road: Dict[str, Any]):
        """Draw a single road"""
        color = '#444444'
        lane_color = '#ffff00'
        
        if road['type'].startswith('horizontal'):
            y = road['y']
            width = road['width']
            x_start = road['x_start']
            x_end = road['x_end']
            
            # Road surface
            plt.fill_between([x_start, x_end], [y - width/2, y - width/2], 
                           [y + width/2, y + width/2], color=color, alpha=0.3, zorder=2)
            
            # Road edges
            plt.plot([x_start, x_end], [y + width/2, y + width/2], color=color, linewidth=2, alpha=0.8)
            plt.plot([x_start, x_end], [y - width/2, y - width/2], color=color, linewidth=2, alpha=0.8)
            
            # Lane markings
            if road.get('lanes', 2) > 1:
                for x in range(int(x_start), int(x_end), 8):
                    plt.plot([x, x+3], [y, y], color=lane_color, linewidth=2, alpha=0.8)
        
        else:  # vertical
            x = road['x']
            width = road['width']
            y_start = road['y_start']
            y_end = road['y_end']
            
            # Road surface
            plt.fill_betweenx([y_start, y_end], [x - width/2, x - width/2], 
                            [x + width/2, x + width/2], color=color, alpha=0.3, zorder=2)
            
            # Road edges
            plt.plot([x + width/2, x + width/2], [y_start, y_end], color=color, linewidth=2, alpha=0.8)
            plt.plot([x - width/2, x - width/2], [y_start, y_end], color=color, linewidth=2, alpha=0.8)
            
            # Lane markings
            if road.get('lanes', 2) > 1:
                for y in range(int(y_start), int(y_end), 8):
                    plt.plot([x, x], [y, y+3], color=lane_color, linewidth=2, alpha=0.8)
    
    def _render_dynamic_obstacles(self):
        """Render dynamic obstacles"""
        for obs_data in self.dynamic_manager.get_render_data():
            self._draw_dynamic_obstacle(obs_data)
    
    def _draw_dynamic_obstacle(self, obs_data: Dict[str, Any]):
        """Draw a single dynamic obstacle"""
        x, y = obs_data['x'], obs_data['y']
        obstacle_type = obs_data['type']
        radius = obs_data['radius']
        vx, vy = obs_data['vx'], obs_data['vy']
        
        # Color and symbol based on type
        if obstacle_type == 'pedestrian':
            color, symbol = 'red', '🚶'
            circle_color = 'red'
        elif obstacle_type == 'cyclist':
            color, symbol = 'orange', '🚴'
            circle_color = 'orange'
        elif obstacle_type == 'slow_vehicle':
            color, symbol = 'purple', '🚗'
            circle_color = 'purple'
        else:  # service_vehicle
            color, symbol = 'brown', '🚚'
            circle_color = 'brown'
        
        # Draw obstacle
        circle = plt.Circle((x, y), radius, color=circle_color, alpha=0.6, zorder=5)
        plt.gca().add_patch(circle)
        
        # Velocity arrow
        if abs(vx) > 0.1 or abs(vy) > 0.1:
            arrow_scale = 3.0
            plt.arrow(x, y, vx * arrow_scale, vy * arrow_scale,
                     head_width=0.8, head_length=0.5, fc=color, ec=color, alpha=0.7, zorder=5)
        
        # Symbol (reduced frequency for performance)
        if random.random() < 0.3:  # Only show 30% of symbols
            plt.text(x, y, symbol, fontsize=10, ha='center', va='center', zorder=6)
    
    def _render_vehicle(self):
        """Render the autonomous vehicle"""
        vehicle_pos = self.vehicle.get_position()
        heading = self.vehicle.get_heading()
        speed = self.vehicle.get_speed()
        
        # Vehicle corners
        corners = self.vehicle.get_vehicle_corners()
        
        # Vehicle body
        vehicle_color = 'green' if self.vehicle.is_parked() else 'blue'
        plt.fill(corners[:, 0], corners[:, 1], color=vehicle_color, alpha=0.8, zorder=8)
        plt.plot(corners[:, 0], corners[:, 1], 'k-', linewidth=2, zorder=8)
        
        # Vehicle direction indicator
        arrow_length = 4.0
        front_x = vehicle_pos[0] + arrow_length * np.cos(heading)
        front_y = vehicle_pos[1] + arrow_length * np.sin(heading)
        plt.arrow(vehicle_pos[0], vehicle_pos[1], 
                 arrow_length * np.cos(heading), arrow_length * np.sin(heading),
                 head_width=1.5, head_length=1.0, fc='red', ec='red', linewidth=2, zorder=9)
        
        # Speed indicator
        plt.text(vehicle_pos[0], vehicle_pos[1] - 6, f'{speed:.1f} m/s', 
                ha='center', fontsize=12, weight='bold', 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8))
    
    def _render_trajectories(self, trajectory: Optional[np.ndarray]):
        """Render planned trajectories"""
        # All lattice trajectories (faded)
        if hasattr(self.planner, 'all_trajectories'):
            for i, traj in enumerate(self.planner.all_trajectories[:8]):  # Limit for performance
                if len(traj) > 1:
                    x_coords = [point[0] for point in traj]
                    y_coords = [point[1] for point in traj]
                    plt.plot(x_coords, y_coords, 'cyan', alpha=0.2, linewidth=1, zorder=3)
        
        # Selected trajectory (highlighted)
        if trajectory is not None and len(trajectory) > 1:
            x_coords = [point[0] for point in trajectory]
            y_coords = [point[1] for point in trajectory]
            plt.plot(x_coords, y_coords, 'lime', linewidth=4, alpha=0.9, zorder=4, label='Planned Path')
    
    def _render_goal_and_parking(self):
        """Render goal and parking zone"""
        # Goal
        goal = self.scenario['goal_position']
        goal_color = 'green' if self.simulation_stats['goal_reached'] else 'red'
        plt.plot(goal[0], goal[1], '*', color=goal_color, markersize=20, zorder=7, label='Goal')
        
        # Distance to goal
        vehicle_pos = self.vehicle.get_position()
        distance = np.linalg.norm(vehicle_pos - goal[:2])
        plt.text(goal[0], goal[1] + 4, f'{distance:.1f}m', ha='center', fontsize=10, 
                color=goal_color, weight='bold')
        
        # Parking zone
        parking = self.scenario['parking_zone']
        parking_color = 'green' if self.vehicle.is_parked() else 'blue'
        parking_alpha = 0.9 if self.vehicle.is_parked() else 0.5
        
        rect = plt.Rectangle((parking.x - parking.width/2, parking.y - parking.height/2), 
                           parking.width, parking.height, 
                           facecolor=parking_color, alpha=parking_alpha, 
                           edgecolor='white', linewidth=3, zorder=6)
        plt.gca().add_patch(rect)
        
        plt.text(parking.x, parking.y, 'P', fontsize=14, ha='center', va='center',
                color='white', weight='bold', zorder=7)
    
    def _render_ui(self, current_time: float):
        """Render UI elements and statistics"""
        # Performance metrics
        vehicle_metrics = self.vehicle.get_performance_metrics()
        
        # Status panel
        status_x, status_y = -180, 130  # Adjusted for LARGER view
        
        # Background panel
        panel_width, panel_height = 40, 30
        panel = plt.Rectangle((status_x, status_y - panel_height), panel_width, panel_height,
                            facecolor='white', alpha=0.9, edgecolor='black', linewidth=2)
        plt.gca().add_patch(panel)
        
        # Status text
        status_text = []
        status_text.append(f"Time: {current_time:.1f}s")
        status_text.append(f"Distance: {vehicle_metrics['total_distance']:.1f}m")
        status_text.append(f"Speed: {vehicle_metrics['current_speed']:.1f} m/s")
        status_text.append(f"Avg Speed: {vehicle_metrics['average_speed']:.1f} m/s")
        status_text.append(f"State: {vehicle_metrics['parking_state'].upper()}")
        
        for i, text in enumerate(status_text):
            plt.text(status_x + 2, status_y - 4 - i*4, text, fontsize=12, weight='bold')
        
        # Performance panel
        if len(self.frame_times) > 10:
            avg_frame_time = np.mean(self.frame_times[-50:])
            avg_planning_time = np.mean(self.planning_times[-50:])
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0
            
            perf_text = [
                f"FPS: {fps:.1f}",
                f"Frame: {avg_frame_time*1000:.1f}ms",
                f"Planning: {avg_planning_time*1000:.1f}ms"
            ]
            
            for i, text in enumerate(perf_text):
                plt.text(status_x + 2, status_y - 25 + i*3, text, fontsize=10, alpha=0.7)
    
    def _print_progress(self, current_time: float, frame_time: float, planning_time: float):
        """Print simulation progress"""
        vehicle_pos = self.vehicle.get_position()
        goal_pos = self.scenario['goal_position'][:2]
        distance_to_goal = np.linalg.norm(vehicle_pos - goal_pos)
        
        print(f"⏱️ T:{current_time:.1f}s | Pos:({vehicle_pos[0]:.1f},{vehicle_pos[1]:.1f}) | "
              f"Goal:{distance_to_goal:.1f}m | Speed:{self.vehicle.get_speed():.1f} | "
              f"Frame:{frame_time*1000:.1f}ms | Planning:{planning_time*1000:.1f}ms")
    
    def _finalize_simulation(self, final_time: float):
        """Finalize simulation and show results"""
        print("\n" + "="*60)
        print("🏁 SIMULATION COMPLETED")
        print("="*60)
        
        # Calculate final statistics
        vehicle_metrics = self.vehicle.get_performance_metrics()
        goal_distance = np.linalg.norm(self.vehicle.get_position() - self.scenario['goal_position'][:2])
        
        print(f"📊 Final Statistics:")
        print(f"   ⏱️ Total time: {final_time:.1f}s")
        print(f"   📏 Total distance: {vehicle_metrics['total_distance']:.1f}m")
        print(f"   🏎️ Average speed: {vehicle_metrics['average_speed']:.1f} m/s")
        print(f"   🎯 Distance to goal: {goal_distance:.1f}m")
        print(f"   🅿️ Parking status: {vehicle_metrics['parking_state'].upper()}")
        print(f"   ❌ Planning failures: {self.simulation_stats['planning_failures']}")
        
        if len(self.frame_times) > 0:
            avg_fps = 1.0 / np.mean(self.frame_times)
            print(f"   🖥️ Average FPS: {avg_fps:.1f}")
        
        # Success criteria
        success_criteria = []
        if self.simulation_stats['goal_reached']:
            success_criteria.append("✅ Goal reached")
        if self.vehicle.is_parked():
            success_criteria.append("✅ Successfully parked")
        if goal_distance < 5.0:
            success_criteria.append("✅ Close to target")
        
        if success_criteria:
            print(f"🎉 Success: {', '.join(success_criteria)}")
        else:
            print("⚠️ Mission incomplete")
        
        print("="*60)

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
            elif obstacle.obstacle_type == 'construction':
                color = '#FF4500'
                symbol = '🚧'
            elif obstacle.obstacle_type == 'traffic_cone':
                color = '#FF6347'
                symbol = '🚦'
            elif obstacle.obstacle_type == 'roadwork':
                color = '#DAA520'
                symbol = '⚠️'
            elif obstacle.obstacle_type == 'broken_vehicle':
                color = '#8B0000'
                symbol = '🚙'
            elif obstacle.obstacle_type == 'dumpster':
                color = '#556B2F'
                symbol = '🗑️'
            elif obstacle.obstacle_type == 'bench':
                color = '#8B4513'
                symbol = '🪑'
            
            # Draw obstacle with LARGER size for LARGE world
            obstacle_rect = patches.Rectangle(
                (obstacle.x - obstacle.width/2, obstacle.y - obstacle.height/2),
                obstacle.width, obstacle.height,
                facecolor=color, alpha=0.8,
                edgecolor='black', linewidth=2, zorder=4
            )
            self.ax.add_patch(obstacle_rect)
            
            # Add symbol with LARGER font
            self.ax.text(obstacle.x, obstacle.y, symbol, fontsize=12, 
                        ha='center', va='center', zorder=5)

def main():
    """Main entry point"""
    print("🌟 ADVANCED AUTONOMOUS VEHICLE SIMULATION")
    print("🌟 Complex Random Environments with Dynamic Obstacles")
    print("🌟 Intelligent Path Planning and Parking")
    print()
    
    # Create simulation manager
    sim_manager = AdvancedSimulationManager()
    
    while True:
        # Create new scenario
        if not sim_manager.create_new_scenario():
            print("❌ Failed to create scenario")
            break
        
        # Run simulation
        sim_manager.run_simulation()
        
        # Ask for another simulation
        print("\n" + "="*60)
        choice = input("🔄 Run another random simulation? (y/n): ")

        if choice != 'y':
            break
        
        print("\n🔄 Generating new random scenario...")
        plt.close('all')  # Close previous plots
        time.sleep(1)
    
    print("\n👋 Thank you for using the Advanced Simulation!")
    print("🔬 This simulation demonstrates:")
    print("   • Complex random environment generation")
    print("   • Advanced lattice-based path planning")
    print("   • Realistic vehicle dynamics and control")
    print("   • Dynamic obstacle avoidance")
    print("   • Autonomous parking capabilities")
    
    # Final cleanup
    plt.close('all')

if __name__ == "__main__":
    main()