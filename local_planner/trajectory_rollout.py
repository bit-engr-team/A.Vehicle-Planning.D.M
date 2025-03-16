import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

def create_map():
    # Boş bir harita oluştur
    fig, ax = plt.subplots(figsize=(20, 3.5))
    ax.set_xlim(0, 20)
    ax.set_ylim(-1.75, 1.75)
    
    # Statik engeller ekle (örneğin, dikdörtgenler)
    static_obstacles = [
        patches.Rectangle((0, 1.5), 20, .5, color='black'),  # (x, y), edges
        patches.Rectangle((0, -2), 20, .5, color='black'),  # (x, y), edges

        patches.Rectangle((8, .25), 2, 1, color='black'),  # vehicles
        patches.Rectangle((16, -1.25), 2, 1, color='black')
    ]
    for obstacle in static_obstacles:
        ax.add_patch(obstacle)
    
    return fig, ax, static_obstacles

class Vehicle:
    def __init__(self, x, y, yaw, speed):
        self.x = x  # X konumu
        self.y = y  # Y konumu
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
            (self.x - 1, self.y - 0.5), 2, 1, angle=np.rad2deg(self.yaw),
            rotation_point='center', color='blue'
        ))

# Parametrik eğri (aracın yapabileceği manevralar)
def parametric_curve(vehicle, steering_angle, dt=0.1, num_steps=25):
    # Araç dinamiklerini simüle et
    trajectory = []
    x, y, yaw = vehicle.x, vehicle.y, vehicle.yaw
    speed = vehicle.speed
    
    for _ in range(num_steps):
        # Yönü güncelle (steering_angle'a göre)
        yaw += steering_angle * dt
        # Konumu güncelle
        x += speed * np.cos(yaw) * dt
        y += speed * np.sin(yaw) * dt
        trajectory.append((x, y))
    
    return trajectory

# Çarpışma kontrolü
def check_collision(trajectory, obstacles):
    r = .6
    space = 0
    for (x, y) in trajectory[::3]:
        for obstacle in obstacles:
            # Dikdörtgen engelin sınırlarını al
            x_min = obstacle.get_x()
            y_min = obstacle.get_y()
            x_max = x_min + obstacle.get_width()
            y_max = y_min + obstacle.get_height()
            
            # Noktanın dikdörtgene en yakın noktasını bul
            closest_x = np.clip(x, x_min, x_max)
            closest_y = np.clip(y, y_min, y_max)
            
            # Nokta ile en yakın nokta arasındaki mesafeyi hesapla
            distance = np.hypot(x - closest_x, y - closest_y)
            
            # Mesafe dairenin yarıçapından küçükse çarpışma var
            if distance <= r:
                return space
        space += 1
    return space

# Trajectory Rollout algoritması

def trajectory_rollout(vehicle, goal, obstacles, dt=0.1, num_trajectories=25):
    best_trajectory = None
    best_cost = float('inf')
    best_steering_angle = 0  # En iyi yörüngeye ait steering açısı
    best_space = 0
    all_trajectories = []  # Tüm yörüngeleri saklamak için
    
    # Steering açıları (aracın yapabileceği manevralar)
    steering_angles = np.linspace(-np.pi/4, np.pi/4, num_trajectories)  # -45° ile +45° arası
    
    for angle in steering_angles:
        # Parametrik eğriyi hesapla
        trajectory = parametric_curve(vehicle, angle, dt)
        all_trajectories.append(trajectory)
        
        # Yörünge maliyetini hesapla (hedefe uzaklık)
        last_point = trajectory[-1]
        cost = np.hypot(last_point[0] - goal[0], last_point[1] - goal[1])
        
        space = check_collision(trajectory, obstacles)
        print(f"{space} {best_space}")
        # Çarpışma kontrolü // en iyi yolu seçmek colision a en uzak olan
        if space == 9:
            if cost < best_cost:
                best_cost = cost
                best_trajectory = trajectory
                best_space = space
                best_steering_angle = angle
        elif space > best_space and space != 0:
            if cost < best_cost:
                best_cost = cost
                best_trajectory = trajectory
                best_space = space
                best_steering_angle = angle
    
    return all_trajectories, best_trajectory, best_steering_angle

# Animasyon fonksiyonu
def animate(frame):
    global vehicle, goal, ax, static_obstacles
    
    # Önceki çizimleri temizle
    ax.clear()
    ax.set_xlim(0, 20)
    ax.set_ylim(-1.75, 1.75)
    
    # Haritayı yeniden çiz
    for obstacle in static_obstacles:
        ax.add_patch(obstacle)
    
    # Yeni yörüngeleri hesapla
    all_trajectories, best_trajectory, best_steering_angle = trajectory_rollout(vehicle, goal, static_obstacles)
    
    # Tüm yörüngeleri çiz
    for trajectory in all_trajectories:
        x_traj, y_traj = zip(*trajectory)
        ax.plot(x_traj, y_traj, 'gray', alpha=0.5)  # Gri renkte tüm yörüngeler
    
    # En iyi yörüngeyi çiz
    if best_trajectory:
        x_best, y_best = zip(*best_trajectory)
        ax.plot(x_best, y_best, 'r--', linewidth=2, label='En İyi Yörünge')
    
    # Hedefi çiz
    ax.plot(goal[0], goal[1], 'go', markersize=10, label='Hedef')
    
    # Aracı çiz
    vehicle.plot(ax)
    
    # Araç durumunu güncelle
    if best_trajectory:
        part = len(best_trajectory) // 5
        for (x, y) in best_trajectory[:part]:
            vehicle.move(0.1, best_steering_angle)
    
    # Eğer hedefe ulaşıldıysa animasyonu durdur
    if goal[0] - vehicle.x < .1:
        anim.event_source.stop()
        plt.pause(0.5)
        plt.close()

# Simülasyonu başlat
fig, ax, static_obstacles = create_map()
vehicle = Vehicle(1, 0.75, 0, 1.5)  # Başlangıç konumu ve hız
goal = (19, 0.5)  # Hedef konumu

# Animasyonu oluştur
anim = FuncAnimation(fig, animate, frames=100, interval=250, repeat=False)
plt.show()