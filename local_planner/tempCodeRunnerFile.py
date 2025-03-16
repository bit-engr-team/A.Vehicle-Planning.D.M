
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
