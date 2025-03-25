import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from mr_sim import *
import time

# Parametrik eğri (aracın yapabileceği manevralar)
def parametric_curve(vehicle, steering_angle, num_steps=25 ,dt=0.1):
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
def check_collision(trajectory, obstacles, radius = 0.8 ,resolution = 0.4):
    jump = int(1/resolution)
    space = 0

    for (x, y) in trajectory[::jump]:
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
            if distance <= radius:
                return space
        space += 1
    return space

# Trajectory Rollout algoritması

def trajectory_rollout(vehicle, goal, obstacles,  num_trajectories=25 , pred_range = 20 , dt=0.1):
    all_trajectories = [] 

    resolution = 0.2

    best_trajectory = None
    best_cost = float('inf')
    best_steering_angle = 0

    best_space = 0


    # Steering açıları (aracın yapabileceği manevralar)
    steering_angles = np.linspace(-np.pi/4, np.pi/4, num_trajectories)  # -45° ile +45° arası
    
    for angle in steering_angles:
        # Parametrik eğriyi hesapla
        trajectory = parametric_curve(vehicle, angle, pred_range ,dt)

        all_trajectories.append(trajectory)
        
        # Yörünge maliyetini hesapla (hedefe uzaklık)
        cost_point = trajectory[-1]
        dist_cost = np.hypot(cost_point[0] - goal[0], cost_point[1] - goal[1]) # minimum

        space = check_collision(trajectory, obstacles , 1 ,resolution) # maximum
        # Çarpışma kontrolü // en iyi yolu seçmek colision a en uzak olan
        if best_space < space:
            best_space = space
            best_cost = dist_cost
            best_trajectory = trajectory
            best_steering_angle = angle   
        elif best_space == space:
            if best_cost > dist_cost:
                best_cost = dist_cost
                best_trajectory = trajectory
                best_steering_angle = angle

        
    
    return all_trajectories, best_trajectory, best_steering_angle

# Animasyon fonksiyonu
def animate(frame):
    
    global ego_vehicle, goal, ax, counter

    traj_count = 15
    pred_range = 40

    # Güncelle
    ax.clear()
    static_obstacles, nodes= create_map(ax)
    
    npc1 = add_car(ax,(-10,1),0)
    npc2 = add_car(ax,(0,-1),0)
    npc3 = add_car(ax,(+10,1),0)

    npc4 = add_car(ax,(-10,17),0)
    npc5 = add_car(ax,(0,19),0)
    npc6 = add_car(ax,(+10,17),0)

    npc7 = add_car(ax,(-10,-19),0)
    npc8 = add_car(ax,(0,-17),0)
    npc9 = add_car(ax,(+10,-19),0)

    npc10 = patches.Rectangle((19 - .7, 9 - 1.5), 1.4, 3,rotation_point='center', color='black')
    npc11 = patches.Rectangle((-19 - .7, -9 - 1.5), 1.4, 3,rotation_point='center', color='black')
    ax.add_patch(npc10)
    ax.add_patch(npc11)
    

    obstacles = static_obstacles + [npc1,npc2,npc3] + [npc4,npc5,npc6] + [npc7,npc8,npc9] + [npc10,npc11]
    # Yeni yörüngeleri hesapla

    all_trajectories, best_trajectory, best_steering_angle = trajectory_rollout(ego_vehicle, goal, obstacles, traj_count , pred_range,0.1)
    
    
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
    ego_vehicle.plot(ax)
    
    if 0 == check_collision([(ego_vehicle.x,ego_vehicle.y)],obstacles,resolution=1):
        print("Fail")
        anim.event_source.stop()
        plt.pause(0.5)
        plt.close()

    # Araç durumunu güncelle
    if best_trajectory:
        part = 5#len(best_trajectory) // 5
        for (x, y) in best_trajectory[:part]:
            ego_vehicle.move(0.1, best_steering_angle)
    
    # Eğer hedefe ulaşıldıysa animasyonu durdur
    if np.hypot(ego_vehicle.x-goal[0] , ego_vehicle.y-goal[1] )< 3:
        counter += 1
        print(counter)
        if counter == 8:
            anim.event_source.stop()
            plt.pause(0.5)
            plt.close()
        goal = nodes[counter]


# Simülasyonu başlat

fig, ax = plt.subplots(figsize=(10, 10))

static_obstacles , nodes = create_map(ax)

ego_vehicle = Vehicle(nodes[0], np.pi, 1)  # Başlangıç konumu ve hız

goal = nodes[1]  # Hedef konumu
counter = 1
# Animasyonu oluştur
anim = FuncAnimation(fig, animate, frames=500, interval=100, repeat=False)
plt.show()
anim.save('local_planner/output/trajectory_rollout.mp4', writer='ffmpeg', fps=30)
print("end")