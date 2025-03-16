import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from heapq import heappop, heappush

class Node:
    def __init__(self, x, y, yaw, cost, parent=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def create_map():
    """Haritayı ve engelleri oluşturur."""
    fig, ax = plt.subplots(figsize=(20, 5))
    ax.set_xlim(0, 50)
    ax.set_ylim(-10, 10)

    obstacles = [
        patches.Rectangle((0, 9.5), 50, 0.5, color='black'),  # Üst sınır
        patches.Rectangle((0, -10), 50, 0.5, color='black'),  # Alt sınır
        patches.Rectangle((10, -10), 1, 20, color='black'),
        patches.Rectangle((20, -10), 1, 20, color='black'),
        patches.Rectangle((30, -10), 1, 20, color='black'),
        patches.Rectangle((40, -10), 1, 20, color='black'),
    ]

    for obs in obstacles:
        ax.add_patch(obs)

    return fig, ax, obstacles

class Vehicle:
    def __init__(self, x, y, yaw, speed):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed
        self.width = 2.0
        self.height = 1.0

    def plot(self, ax):
        """Aracı çizer."""
        rect = patches.Rectangle(
            (self.x - self.width / 2, self.y - self.height / 2),
            self.width, self.height,
            angle=np.rad2deg(self.yaw),
            rotation_point='center',
            color='blue'
        )
        ax.add_patch(rect)

def check_collision(x, y, obstacles, vehicle_width=2.0, vehicle_height=1.0):
    """Aracın boyutlarını dikkate alarak çarpışma kontrolü yapar."""
    for obs in obstacles:
        x_min = obs.get_x()
        y_min = obs.get_y()
        x_max = x_min + obs.get_width()
        y_max = y_min + obs.get_height()
        if (x_min - vehicle_width / 2 <= x <= x_max + vehicle_width / 2) and \
           (y_min - vehicle_height / 2 <= y <= y_max + vehicle_height / 2):
            return True
    return False

def heuristic(node, goal):
    """Hedefe olan öklid mesafesini hesaplar."""
    return np.hypot(node.x - goal[0], node.y - goal[1])

def hybrid_astar(start, goal, vehicle, obstacles, dt=0.1, max_steering=np.pi/4, num_angles=15, goal_tolerance=1.5):
    """Hybrid A* algoritması."""
    open_list = []
    closed_list = set()
    heappush(open_list, (0, Node(start[0], start[1], start[2], 0)))

    while open_list:
        _, current = heappop(open_list)

        if (current.x, current.y, current.yaw) in closed_list:
            continue
        closed_list.add((current.x, current.y, current.yaw))

        # Hedefe ulaşma kontrolü
        if np.hypot(current.x - goal[0], current.y - goal[1]) < goal_tolerance:
            path = []
            while current:
                path.append((current.x, current.y, current.yaw))
                current = current.parent
            return path[::-1]

        for steering_angle in np.linspace(-max_steering, max_steering, num_angles):
            new_yaw = current.yaw + steering_angle * dt
            new_x = current.x + vehicle.speed * np.cos(new_yaw) * dt
            new_y = current.y + vehicle.speed * np.sin(new_yaw) * dt

            if check_collision(new_x, new_y, obstacles):
                continue

            new_cost = current.cost + dt
            new_node = Node(new_x, new_y, new_yaw, new_cost, current)
            heappush(open_list, (new_cost + heuristic(new_node, goal), new_node))

    return None

def draw_path(ax, path):
    """Bulunan yolu çizer."""
    if not path:
        return
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    ax.plot(xs, ys, 'r--', linewidth=2, label="Yol")

def visualize_path(fig, ax, vehicle, path, goal, obstacles):
    """Yolu ve aracı görselleştirir."""
    ax.clear()
    ax.set_xlim(0, 50)
    ax.set_ylim(-10, 10)

    # Engelleri çiz
    for obs in obstacles:
        ax.add_patch(obs)

    # Hedefi çiz
    ax.plot(goal[0], goal[1], 'go', markersize=10, label='Hedef')

    # Yolu çiz
    draw_path(ax, path)

    # Aracı çiz
    if path:
        for x, y, yaw in path:
            vehicle.x = x
            vehicle.y = y
            vehicle.yaw = yaw
            vehicle.plot(ax)

    ax.legend()
    plt.show()

# Simülasyonu başlat
fig, ax, static_obstacles = create_map()
vehicle = Vehicle(1, 0, 0, 1.0)  # Başlangıç konumu ve hız
goal = (48, 0)  # Hedef konumu

# Hybrid A* algoritmasını çalıştır
path = hybrid_astar(
    (vehicle.x, vehicle.y, vehicle.yaw), 
    goal, 
    vehicle, 
    static_obstacles, 
    dt=0.2,  # Daha büyük zaman adımı
    goal_tolerance=2.0  # Daha geniş hedef toleransı
)

# Sonuçları görselleştir
if path:
    visualize_path(fig, ax, vehicle, path, goal, static_obstacles)
else:
    print("❌ Yol bulunamadı.")
