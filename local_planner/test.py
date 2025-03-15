import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def create_map():
    # Boş bir harita oluştur
    fig, ax = plt.subplots()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    
    # Statik engeller ekle (örneğin, dikdörtgenler)
    static_obstacles = [
        patches.Rectangle((2, 2), 1, 1, color='black'),  # (x, y), genişlik, yükseklik
        patches.Rectangle((6, 6), 1, 1, color='black')
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

    def move(self, dt):
        # Araç hareketini güncelle
        self.x += self.speed * np.cos(self.yaw) * dt
        self.y += self.speed * np.sin(self.yaw) * dt

    def plot(self, ax):
        # Aracı çiz
        ax.plot(self.x, self.y, 'bo')  # Mavi nokta olarak araç

def trajectory_rollout(vehicle, goal, obstacles, dt=0.1, num_trajectories=10):
    best_trajectory = None
    best_cost = float('inf')
    
    for _ in range(num_trajectories):
        # Rastgele bir yön ve hız seç
        yaw = np.random.uniform(0, 2 * np.pi)
        speed = np.random.uniform(0.5, 1.5)
        
        # Yörüngeyi simüle et
        trajectory = []
        temp_vehicle = Vehicle(vehicle.x, vehicle.y, yaw, speed)
        for _ in range(10):  # 10 adım simüle et
            temp_vehicle.move(dt)
            trajectory.append((temp_vehicle.x, temp_vehicle.y))
        
        # Yörünge maliyetini hesapla (hedefe uzaklık ve engel çarpışmaları)
        cost = np.hypot(trajectory[-1][0] - goal[0], trajectory[-1][1] - goal[1])
        if not check_collision(trajectory, obstacles):
            if cost < best_cost:
                best_cost = cost
                best_trajectory = trajectory
    
    return best_trajectory

def check_collision(trajectory, obstacles):
    # Yörüngenin engellere çarpıp çarpmadığını kontrol et
    for (x, y) in trajectory:
        for obstacle in obstacles:
            if obstacle.contains_point((x, y)):
                return True
    return False

def run_simulation():
    fig, ax, static_obstacles = create_map()
    goal = (8, 8)  # Hedef konumu
    vehicle = Vehicle(1, 1, 0, 1)  # Başlangıç konumu ve hız
    
    # Trajectory Rollout ile en iyi yörüngeyi bul
    best_trajectory = trajectory_rollout(vehicle, goal, static_obstacles)
    
    # Yörüngeyi çiz
    if best_trajectory:
        x_traj, y_trajectory = zip(*best_trajectory)
        ax.plot(x_traj, y_trajectory, 'r--', label='En İyi Yörünge')
    
    # Hedefi çiz
    ax.plot(goal[0], goal[1], 'go', label='Hedef')
    
    # Araç ve haritayı göster
    vehicle.plot(ax)
    plt.legend()
    plt.savefig("trajectory_plot.png")  # Grafiği PNG olarak kaydet
    

run_simulation()

