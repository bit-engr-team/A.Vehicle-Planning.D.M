import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation


def create_map(ax):
    
    ax.set_xlim(21,-21)
    ax.set_ylim(-21,21)

    static_obstacles = [
        #Outer walls
        patches.Rectangle((-21, -21) ,1, 42 , color='gray'),
        patches.Rectangle((20, -20) ,1, 42 , color='gray'),

        patches.Rectangle((-21, -21) ,42, 1 , color='gray'),
        patches.Rectangle((-21, 20) ,42, 1 , color='gray'),

        #Inner Walls
        patches.Rectangle(( -16, 2) , 32, 14 , color='gray'),
        patches.Rectangle(( -16, -16) , 32, 14 , color='gray'),
    ]

    for obstacle in static_obstacles:
        ax.add_patch(obstacle)

    # FIX #1: Corrected node list - removed duplicate and ordered correctly         
    nodes = [
        (0, 0),        # Starting position (added)
        (18, -18),     # First goal
        (-18, -18),    # Second goal
        (-18, -3),     # Third goal
        (-18, 18),     # Fourth goal 
        (18, 18),      # Fifth goal
        (18, -3)       # Sixth goal
    ]

    # Add visualization for the nodes
    for i, vertex in enumerate(nodes):
        if i > 0:  # Skip drawing the starting position
            ax.add_patch(patches.Circle(vertex, 3, fill=False, ec="red", lw=3))
            ax.add_patch(patches.Circle(vertex, 0.1, color="red"))
            # Add node number for clarity
            ax.text(vertex[0]+1, vertex[1]+1, f"{i}", fontsize=12, color="black")
    
    return static_obstacles, nodes

class Vehicle:
    def __init__(self, p = tuple(), yaw = 0, speed = 0):
        self.x = p[0]  # X konumu
        self.y = p[1]  # Y konumu
        self.yaw = yaw  # Yön (radyan)
        self.speed = speed  # Hız

    def move(self, dt, steering_angle):
        # Yönü güncelle (steering_angle'a göre)
        self.yaw += steering_angle * dt
        # Konumu güncelle
        self.x += self.speed * np.cos(self.yaw) * dt
        self.y += self.speed * np.sin(self.yaw) * dt

    def plot(self, ax):
        # Aracı çiz
        ax.add_patch(patches.Rectangle(
            (self.x - 1.5, self.y - .7), 3, 1.4, angle=np.rad2deg(self.yaw),
            rotation_point='center', color='blue'
        ))


def add_car(ax, position=tuple(),rotation=0):
        x = position[0]
        y = position[1]

        car = patches.Rectangle(
            (x - 1.5, y - .7), 3, 1.4, angle=np.rad2deg(rotation),
            rotation_point='center', color='black'
        )
        ax.add_patch(car)
        return car




# Daha uzak offsetler ve her yönde çalışan yol üretimi
def generate_lattice_paths(start_pos, goal_pos, num_paths=5, num_points=20, max_offset=2.5, vehicle_yaw=0.0):
    """Her yönde düzgün çalışan, daha geniş offsetli yollar oluşturur."""
    paths = []
    
    # Başlangıç ve hedef arasındaki mesafe ve yön
    dx = goal_pos[0] - start_pos[0]
    dy = goal_pos[1] - start_pos[1]
    dist = np.hypot(dx, dy)
    path_angle = np.arctan2(dy, dx)
    
    # Bakış mesafesini araca göre ayarla - daha uzağı görebilsin
    lookahead_dist = min(dist, 10.0 + 5.0 * (1.0 - abs(np.cos(vehicle_yaw - path_angle))))
    
    if dist > lookahead_dist:
        # Bakış mesafesine göre bir ara hedef belirle
        local_goal = (
            start_pos[0] + lookahead_dist * np.cos(path_angle),
            start_pos[1] + lookahead_dist * np.sin(path_angle)
        )
    else:
        local_goal = goal_pos
    
    # YÖN-BAĞIMSIZ YOL OLUŞTURMA: İlerleyiş yönünde bir parametre tanımla
    t = np.linspace(0, 1, num_points)
    
    # Yönden bağımsız doğrusal ilerleme 
    base_x = start_pos[0] + (local_goal[0] - start_pos[0]) * t
    base_y = start_pos[1] + (local_goal[1] - start_pos[1]) * t
    
    # Daha geniş ofset için adapte edilmiş hesaplama - min. 1.5, max değere kadar
    adaptive_max_offset = max(1.5, min(max_offset, 1.0 + lookahead_dist * 0.15))
    
    # İlerleyiş yönüne dik olan vektörü hesapla
    perpendicular_angle = path_angle + np.pi/2
    perp_x = np.cos(perpendicular_angle)
    perp_y = np.sin(perpendicular_angle)
    
    for i in range(num_paths):
        # Orta yol için ofset sıfır, diğer yollar için simetrik ofsetler
        offset_ratio = (i - (num_paths - 1) / 2) / ((num_paths - 1) / 2)
        offset_magnitude = adaptive_max_offset * offset_ratio
        
        # Daha keskin S-eğrisi - daha erken maksimuma ulaşıp daha geç azalan
        s_curve = 8 * (t**2) - 10 * (t**3) + 3 * (t**4)
        
        # Her noktanın ofset değerini hesapla
        offset_values = offset_magnitude * s_curve
        
        # Yolu oluştur - yön doğrultusunda ilerler, dik yönde offset alır
        path_x = base_x + offset_values * perp_x
        path_y = base_y + offset_values * perp_y
        
        paths.append(list(zip(path_x, path_y)))
    
    return paths

# Çarpışma kontrolü
def check_collision(path, obstacles, vehicle_radius=0.8, resolution=0.4):
    """Bir yolun engellerle çarpışma kontrolü"""
    jump = int(1/resolution)
    safe_steps = 0
    
    for point in path[::jump]:
        x, y = point
        
        for obstacle in obstacles:
            # Dikdörtgen engelin sınırlarını al
            x_min = obstacle.get_x() 
            y_min = obstacle.get_y() 
            x_max = x_min + obstacle.get_width()
            y_max = y_min + obstacle.get_height()
            
            # Noktanın dikdörtgene en yakın noktasını bul
            closest_x = np.clip(x, x_min, x_max)
            closest_y = np.clip(y, y_min, y_max)
            
            # Aradaki mesafeyi hesapla
            distance = np.hypot(x - closest_x, y - closest_y)
            
            # Çarpışma varsa, güvenli adım sayısını döndür
            if distance <= vehicle_radius:
                return safe_steps
        
        safe_steps += 1
    
    return safe_steps

# Yol değerlendirme fonksiyonunda düzgünlüğün ağırlığını azaltıp, hedefe yakınlığı daha önemli yap
def evaluate_path(path, goal, weight_dist=0.75, weight_smooth=0.25):  # 0.6/0.4'ten 0.75/0.25'e değiştirdik
    """Bir yolun hedefe yakınlık ve düzgünlük açısından değerlendirilmesi."""
    # Hedefe uzaklık maliyeti
    end_point = path[-1]
    dist_cost = np.hypot(end_point[0] - goal[0], end_point[1] - goal[1])
    
    # Yolun düzgünlüğünü hesapla
    smoothness_cost = 0.0
    if len(path) > 2:
        for i in range(len(path) - 2):
            v1 = [path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]]
            v2 = [path[i+2][0] - path[i+1][0], path[i+2][1] - path[i+1][1]]
            
            # Vektörleri normalize et
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm > 0 and v2_norm > 0:
                v1_normalized = [v1[0]/v1_norm, v1[1]/v1_norm]
                v2_normalized = [v2[0]/v2_norm, v2[1]/v2_norm]
                
                # Açı değişimi (dot product)
                dot_product = max(min(v1_normalized[0]*v2_normalized[0] + 
                                      v1_normalized[1]*v2_normalized[1], 1.0), -1.0)
                angle_change = np.arccos(dot_product)
                smoothness_cost += angle_change
    
    # Toplam maliyet
    total_cost = weight_dist * dist_cost + weight_smooth * smoothness_cost
    return total_cost

# Conformal Lattice Planner çağrısında daha keskin dönüşlere izin ver
def conformal_lattice_planner(vehicle, goal, obstacles, num_paths=7, max_offset=2.5):
    """Conformal Lattice Planner - daha geniş offsetli yollarla engelden kaçınır."""
    all_paths = generate_lattice_paths(
        (vehicle.x, vehicle.y), goal, 
        num_paths=num_paths, 
        max_offset=max_offset,
        vehicle_yaw=vehicle.yaw  # Araç yönelimini ekle
    )
    
    valid_paths = []
    path_safety = []
    path_costs = []
    
    for path in all_paths:
        # Çarpışma kontrolü
        safety = check_collision(path, obstacles, vehicle_radius=0.8)
        
        if safety > 0:  # En az bir adım güvenli ise
            # Yolun maliyetini hesapla
            cost = evaluate_path(path, goal)
            valid_paths.append(path)
            path_safety.append(safety)
            path_costs.append(cost)
    
    # En iyi yolu seç (önce güvenlik, sonra maliyet)
    best_path = None
    best_steering = 0.0
    
    if valid_paths:
        # Önce en uzun güvenli yolu bul
        max_safety = max(path_safety)
        safest_indices = [i for i, s in enumerate(path_safety) if s == max_safety]
        
        # Güvenli yollar arasında en düşük maliyetli olanı seç
        if safest_indices:
            best_idx = safest_indices[np.argmin([path_costs[i] for i in safest_indices])]
            best_path = valid_paths[best_idx]
            
            # Yönlendirme açısını hesapla
            if len(best_path) > 1:
                target_point = best_path[min(10, len(best_path)-1)]
                dx = target_point[0] - vehicle.x
                dy = target_point[1] - vehicle.y
                target_angle = np.arctan2(dy, dx)
                
                # Yönlendirme açısı
                angle_diff = target_angle - vehicle.yaw
                # Açıyı normalize et
                while angle_diff > np.pi: angle_diff -= 2*np.pi
                while angle_diff < -np.pi: angle_diff += 2*np.pi
                
                # Daha keskin dönüşlere izin ver
                best_steering = np.clip(angle_diff, -np.pi/3, np.pi/3)  # π/4 yerine π/3 kullan (~60 derece)
    
    return all_paths, valid_paths, best_path, best_steering

# Animasyon fonksiyonunda daha keskin hareketlere izin ver
def animate(frame):
    global ego_vehicle, goal, ax, counter, transitioning, global_nodes, transition_timer

    # Clear the plot
    ax.clear()
    static_obstacles, _ = create_map(ax)  # Recreate the map

    # Add NPC vehicles
    npc1 = add_car(ax, (-10, 1), 0)
    npc2 = add_car(ax, (0, -1), 0)
    npc3 = add_car(ax, (10, 1), 0)
    npc4 = add_car(ax, (-10, 17), 0)
    npc5 = add_car(ax, (0, 19), 0)
    npc6 = add_car(ax, (10, 17), 0)
    npc7 = add_car(ax, (-10, -19), 0)
    npc8 = add_car(ax, (0, -17), 0)
    npc9 = add_car(ax, (10, -19), 0)
    npc10 = patches.Rectangle((19 - .7, 9 - 1.5), 1.4, 3, rotation_point='center', color='black')
    npc11 = patches.Rectangle((-19 - .7, -9 - 1.5), 1.4, 3, rotation_point='center', color='black')
    ax.add_patch(npc10)
    ax.add_patch(npc11)

    obstacles = static_obstacles + [npc1, npc2, npc3, npc4, npc5, npc6, npc7, npc8, npc9, npc10, npc11]

    # Add current goal information to the plot
    ax.set_title(f"Goal: {counter} of {len(global_nodes)-1}, Position: {goal}")

    # Call the planner
    all_paths, valid_paths, best_path, best_steering = conformal_lattice_planner(
        ego_vehicle, goal, obstacles, num_paths=7, max_offset=2.5
    )

    # Plot all paths
    for path in all_paths:
        x_path, y_path = zip(*path)
        ax.plot(x_path, y_path, 'gray', alpha=0.3)

    # Plot valid paths
    for path in valid_paths:
        x_path, y_path = zip(*path)
        ax.plot(x_path, y_path, 'c-', alpha=0.5)

    # Plot the best path
    if best_path:
        x_best, y_best = zip(*best_path)
        ax.plot(x_best, y_best, 'g-', linewidth=2, label='Best Path')

    # Plot the goal
    ax.plot(goal[0], goal[1], 'go', markersize=10, label='Goal')

    # Plot the vehicle
    ego_vehicle.plot(ax)

    # Collision check
    if check_collision([(ego_vehicle.x, ego_vehicle.y)], obstacles, resolution=1) == 0:
        print("Fail - Collision!")
        anim.event_source.stop()
        plt.pause(0.5)
        plt.close()

    # Move the vehicle with more aggressive steering
    if best_path:
        scaled_steering = best_steering * 0.95  # Increased from 0.9 to 0.95
        ego_vehicle.move(0.1, scaled_steering)

    # Check if the vehicle has reached the goal
    if np.hypot(ego_vehicle.x - goal[0], ego_vehicle.y - goal[1]) < 1.8:  # Increased from 1.5 to 1.8
        if not transitioning:  # If not already transitioning
            transitioning = True
            transition_timer = 0  # Reset the timer
            counter += 1
            print(f"Reached goal {counter-1} at {goal}")

            if counter >= len(global_nodes):  # Check if all goals are reached
                print("All goals completed!")
                anim.event_source.stop()
                plt.pause(0.5)
                plt.close()
            else:
                # Set the new goal
                goal = global_nodes[counter]
                print(f"New goal set: {goal}")

        else:
            # During transition, check if the vehicle is aligned with the next goal
            dx = goal[0] - ego_vehicle.x
            dy = goal[1] - ego_vehicle.y
            target_angle = np.arctan2(dy, dx)
            angle_diff = target_angle - ego_vehicle.yaw

            # Normalize the angle difference
            while angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            while angle_diff < -np.pi:
                angle_diff += 2 * np.pi

            # Increment transition timer
            transition_timer += 1
            
            # FIX #3: Either proper alignment OR timeout after 20 frames
            if abs(angle_diff) < 0.8 or transition_timer > 20:  # More lenient angle (0.5 to 0.8) + timeout
                transitioning = False
                print(f"Transition complete: angle diff = {abs(angle_diff):.2f}, timer = {transition_timer}")

# FIX #4: Initialize transition_timer
fig, ax = plt.subplots(figsize=(10, 10))
static_obstacles, global_nodes = create_map(ax)  # Global olarak bir kez oluştur

# FIX #5: Change starting yaw to 0 (facing right) from pi (facing left)
ego_vehicle = Vehicle(global_nodes[0], 0, 1)  # Changed yaw from pi to 0
goal = global_nodes[1]  
counter = 1
transitioning = False
transition_timer = 0  # Added transition timer

# Animasyonu oluştur
anim = FuncAnimation(fig, animate, frames=500, interval=100, repeat=False)
plt.show()

# Fix for animation saving
try:
    # Use a relative path or create the directory if it doesn't exist
    import os
    output_dir = 'output'
    
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    # Save using the correct path
    output_path = os.path.join(output_dir, 'conformal_lattice.mp4')
    anim.save(output_path, writer='ffmpeg', fps=30)
    print(f"Simulation completed and saved to {output_path}")
except Exception as e:
    print(f"Error saving animation: {e}")
    print("Simulation completed but not saved")