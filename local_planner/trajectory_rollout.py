import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

def create_map():
    # Boş bir harita oluştur
    fig, ax = plt.subplots(figsize=(20, 3))
    ax.set_xlim(0, 20)
    ax.set_ylim(-1.5, 1.5)
    
    # Statik engeller ekle (örneğin, dikdörtgenler)
    static_obstacles = [
        patches.Rectangle((0, 1.25), 20, .5, color='black'),  # (x, y), edges
        patches.Rectangle((0, -1.75), 20, .5, color='black'),  # (x, y), edges
        patches.Rectangle((8, 0), 2, 1, color='black'),  # (x, y), genişlik, yükseklik
        patches.Rectangle((16, -1), 2, 1, color='black')
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
def parametric_curve(vehicle, steering_angle, dt=0.1, num_steps=20):
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
    for (x, y) in trajectory:
        for obstacle in obstacles:
            # Dikdörtgen engelin sınırlarını al
            x_min = obstacle.get_x()
            y_min = obstacle.get_y()
            x_max = x_min + obstacle.get_width()
            y_max = y_min + obstacle.get_height()
            
            # Nokta dikdörtgenin içinde mi?
            if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                return True
    return False

# Trajectory Rollout algoritması
def trajectory_rollout(vehicle, goal, obstacles, dt=0.1, num_trajectories=11):
    best_trajectory = None
    best_cost = float('inf')
    best_steering_angle = 0  # En iyi yörüngeye ait steering açısı
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
        
        # Çarpışma kontrolü
        if not check_collision(trajectory, obstacles):
            if cost < best_cost:
                best_cost = cost
                best_trajectory = trajectory
                best_steering_angle = angle
    
    return all_trajectories, best_trajectory, best_steering_angle

# Animasyon fonksiyonu
def animate(frame):
    global vehicle, goal, ax, static_obstacles
    
    # Önceki çizimleri temizle
    ax.clear()
    ax.set_xlim(0, 20)
    ax.set_ylim(-1.5, 1.5)
    
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
        part = len(best_trajectory) // 3
        for (x, y) in best_trajectory[:part]:
            vehicle.move(0.1, best_steering_angle)
    
    # Eğer hedefe ulaşıldıysa animasyonu durdur
    if goal[0] - vehicle.x < 1:
        anim.event_source.stop()
        plt.close()

# Simülasyonu başlat
fig, ax, static_obstacles = create_map()
vehicle = Vehicle(1, 0.5, 0, 2)  # Başlangıç konumu ve hız
goal = (19, 0.5)  # Hedef konumu

# Animasyonu oluştur
anim = FuncAnimation(fig, animate, frames=100, interval=250, repeat=False)
plt.show()