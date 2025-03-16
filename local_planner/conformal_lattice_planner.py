import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

def create_map():
    """Harita oluşturur ve statik engeller ekler."""
    fig, ax = plt.subplots(figsize=(20, 3))  # Harita boyutlarını küçült
    ax.set_xlim(0, 20)  # X eksenini 20 birime ayarla
    ax.set_ylim(-1.5, 1.5)  # Y eksenini -1.5 ile 1.5 arasında ayarla
    
    # Yol ve engeller
    static_obstacles = [
        patches.Rectangle((0, 1.25), 20, 0.2, color='black'),   # Üst yol sınırı
        patches.Rectangle((0, -1.5), 20, 0.2, color='black'),  # Alt yol sınırı
        patches.Rectangle((8, 0), 1, 0.5, color='black'),      # Orta engel
        patches.Rectangle((16, -1), 1, 0.5, color='black')     # Alt engel
    ]
    for obs in static_obstacles:
        ax.add_patch(obs)
    
    return fig, ax, static_obstacles

class Vehicle:
    """Basit bir araç sınıfı."""
    def __init__(self, x, y, yaw=0.0, speed=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed
        self.width = 0.5  # Araç genişliği
        self.height = 0.25  # Araç yüksekliği
    
    def move(self, dt, steering_angle):
        """Aracı, verilen direksiyon açısı ile hareket ettir."""
        self.yaw += steering_angle * dt
        self.x += self.speed * np.cos(self.yaw) * dt
        self.y += self.speed * np.sin(self.yaw) * dt

    def plot(self, ax):
        """Aracı dikdörtgen olarak çizer."""
        ax.add_patch(
            patches.Rectangle(
                (self.x - self.width / 2, self.y - self.height / 2),  # Sol alt köşe
                self.width, self.height,  # Genişlik ve yükseklik
                angle=np.rad2deg(self.yaw),  # Yön
                color='blue',
                fill=True
            )
        )

def generate_lattice_paths(start, goal, num_paths=7, num_points=50):
    """Başlangıç ve hedef arasında çatal şeklinde kafes yolları oluştur."""
    paths = []
    x = np.linspace(start[0], goal[0], num_points)  # X koordinatları

    # Yanal ofset dağılımları (çatal etkisi için artan ofsetler)
    max_offset = 2.0  # Çatalın genişliği
    for i in range(num_paths):
        # Orta yol için ofset sıfır, diğer yollar için simetrik ofsetler
        offset = max_offset * (i - (num_paths - 1) / 2) / ((num_paths - 1) / 2)
        y = start[1] + (goal[1] - start[1]) * (x - start[0]) / (goal[0] - start[0]) + offset * (x - start[0]) / (goal[0] - start[0])
        paths.append((x, y))
    
    return paths

def check_collision(path_x, path_y, obstacles):
    """Yolun, engellerle çarpışma kontrolü."""
    for (px, py) in zip(path_x, path_y):
        for obs in obstacles:
            ox, oy = obs.get_x(), obs.get_y()
            ox2, oy2 = ox + obs.get_width(), oy + obs.get_height()
            if ox <= px <= ox2 and oy <= py <= oy2:
                return True
    return False

def evaluate_path_cost(path_x, path_y, goal):
    """Yolları, hedefe uzaklığa göre değerlendir."""
    return np.hypot(path_x[-1] - goal[0], path_y[-1] - goal[1])

def lattice_planner(vehicle, goal, obstacles, lattice_paths):
    """Basit Lattice Planner: engelden kaçıp en yakın yolu seçer."""
    valid_paths = []
    path_costs = []
    for (px, py) in lattice_paths:
        if not check_collision(px, py, obstacles):
            cost = evaluate_path_cost(px, py, goal)
            valid_paths.append((px, py))
            path_costs.append(cost)
    
    if valid_paths:
        best_idx = np.argmin(path_costs)
        best_path = valid_paths[best_idx]
        return valid_paths, best_path
    else:
        return [], None

def animate(frame):
    """Animasyon döngüsü."""
    global vehicle, goal, ax, static_obstacles, lattice_paths
    
    ax.clear()
    ax.set_xlim(0, 20)  # X eksenini 20 birime ayarla
    ax.set_ylim(-1.5, 1.5)  # Y eksenini -1.5 ile 1.5 arasında ayarla
    
    # Engelleri yeniden çiz
    for obs in static_obstacles:
        ax.add_patch(obs)
    
    # Tüm yolları çiz
    for px, py in lattice_paths:
        ax.plot(px, py, 'gray', alpha=0.3)  # Gri renkte tüm yollar
    
    # Geçerli yollar ve en iyi yol
    valid_paths, best_path = lattice_planner(vehicle, goal, static_obstacles, lattice_paths)
    for px, py in valid_paths:
        ax.plot(px, py, 'c-', alpha=0.5)  # Geçerli yolları mavi renkte çiz
    
    if best_path is not None:
        bx, by = best_path
        ax.plot(bx, by, 'r-', linewidth=2, label='En İyi Yörünge')  # En iyi yolu kırmızı renkte çiz
        
        # Bir sonraki nokta için direksiyon hesabı
        if len(bx) > 1:
            steer_angle = np.arctan2(by[1] - vehicle.y, bx[1] - vehicle.x)
        else:
            steer_angle = 0.0
        vehicle.move(0.1, steer_angle)
    
    # Aracı ve hedefi çiz
    ax.plot(goal[0], goal[1], 'go', markersize=10, label='Hedef')
    vehicle.plot(ax)
    ax.legend(loc='upper right')
    
    # Hedefe yakınsa animasyonu sonlandır
    if np.hypot(vehicle.x - goal[0], vehicle.y - goal[1]) < 0.5:
        anim.event_source.stop()
        plt.title("Hedefe Ulaşıldı!")

# Başlangıç
fig, ax, static_obstacles = create_map()

# Başlangıç ve hedef noktalarını tanımlayın
vehicle = Vehicle(1, 0.0, 0.0, 1.0)
goal = (19, 0.0)  # Hedef noktası

# Çatal şeklinde yollar oluştur
lattice_paths = generate_lattice_paths((vehicle.x, vehicle.y), goal)

# Animasyonu başlat
anim = FuncAnimation(fig, animate, frames=100, interval=200, repeat=False)
plt.show()