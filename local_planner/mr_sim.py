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

             
    nodes = [(18,-18),
             (-18,-18),
             (-18,0),
             (18,0),
             (18,18),
             (-18,18),
         ]

    for vertex in nodes:
        ax.add_patch(patches.Circle(vertex,3 ,fill = False ,ec = "red",lw=3))
        ax.add_patch(patches.Circle(vertex,0.1, color = "red"))
    
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


def main():
    fig, ax = plt.subplots(figsize=(10, 10))

    static_obstacles, nodes = create_map(fig, ax)

    vehicleA = Vehicle((0,1), 0, 0)
    vehicleB = Vehicle((0,-1), 0, 0) 


    vehicleA.plot(ax)
    vehicleB.plot(ax)


    ax.plot()

    plt.show()
