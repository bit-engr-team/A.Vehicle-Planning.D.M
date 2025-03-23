import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

def create_map():
    """Harita oluşturur ve statik engeller ekler."""
    fig, ax = plt.subplots(figsize=(32, 6))  # Harita boyutlarını genişlet
    ax.set_xlim(0, 80)  # X eksenini 40'dan 80'e uzattık
    ax.set_ylim(-2.5, 2.5)  # Y ekseni aynı
    
    # Yol ve engeller - daha düzenli engeller ve daha uzun yol
    static_obstacles = [
        patches.Rectangle((0, 2.0), 80, 0.5, color='black'),   # Üst yol sınırı
        patches.Rectangle((0, -2.5), 80, 0.5, color='black'),  # Alt yol sınırı
        # Engeller - 12'deki engeli yukarı kaydırdım
        patches.Rectangle((12, 0.75), 0.5, 0.5, color='red'),   # İlk engel yukarı taşındı (-0.25->0.75)
        patches.Rectangle((20, 0.75), 0.6, 0.6, color='red'),   # Üst engel
        patches.Rectangle((30, -1.0), 0.7, 0.7, color='red'),   # Alt engel
        patches.Rectangle((40, 0.0), 0.5, 0.5, color='red'),    # Orta engel
        patches.Rectangle((50, -0.5), 0.6, 0.6, color='red'),   # Alt engel 2
        patches.Rectangle((60, 0.5), 0.7, 0.7, color='red'),    # Üst engel 2
        patches.Rectangle((70, -0.25), 0.5, 0.5, color='red'),  # Son engel
    ]
    for obs in static_obstacles:
        ax.add_patch(obs)
    
    # Yol çizgileri
    for i in range(0, 81, 10):  # 5'ten 10'a çıkardık, 80'e kadar
        ax.plot([i, i], [-2.0, 2.0], 'y--', alpha=0.3)  # Yol çizgileri
    
    return fig, ax, static_obstacles

class Vehicle:
    """Basit bir araç sınıfı."""
    def __init__(self, x, y, yaw=0.0, speed=1.5):  # Hızı 1.5'e çıkardım
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

def generate_lattice_paths(start, goal, num_paths=9, num_points=100, max_offset=1.8):
    """Başlangıç ve hedef arasında daha kısa ve duyarlı kafes yolları oluştur."""
    paths = []
    
    # Başlangıç ve hedef arasındaki mesafe
    dist_to_goal = np.hypot(goal[0] - start[0], goal[1] - start[1])
    
    # Hedef çok uzaktaysa, ileri bakış mesafesini uzun tut
    lookahead_dist = min(dist_to_goal, 20.0)  # 25.0'dan 20.0'a düşürdüm
    
    # İleri bakış mesafesine göre bir ara hedef belirle
    if dist_to_goal > lookahead_dist:
        direction = [(goal[0] - start[0])/dist_to_goal, (goal[1] - start[1])/dist_to_goal]
        local_goal = (start[0] + direction[0] * lookahead_dist, 
                     start[1] + direction[1] * lookahead_dist)
    else:
        local_goal = goal
    
    # X koordinatları - başlangıçtan ara hedefe
    x = np.linspace(start[0], local_goal[0], num_points)
    
    # Adaptif ofset - daha uzun mesafe için daha geniş yollar
    adaptive_max_offset = min(max_offset, lookahead_dist * 0.15)  # 0.2'den 0.15'e düşürdük
    
    for i in range(num_paths):
        # Orta yol için ofset sıfır, diğer yollar için simetrik ofsetler
        ratio = (i - (num_paths - 1) / 2) / ((num_paths - 1) / 2)
        offset = adaptive_max_offset * ratio
        
        t = np.linspace(0, 1, num_points)
        
        # Daha yumuşak geçiş
        transition = 3 * (t**2) - 2 * (t**3)  # Yumuşak S-eğrisi
        
        # Temel doğrusal yol + ofset
        y = start[1] + (local_goal[1] - start[1]) * t + offset * transition
        
        paths.append((x, y))
    
    return paths

def check_collision(path_x, path_y, obstacles, vehicle_width=0.6, vehicle_height=0.3):
    """Yolun, engellerle çarpışma kontrolü - araç boyutlarını dikkate alır ve ileri görüş ekler."""
    half_width = vehicle_width / 2
    half_height = vehicle_height / 2
    
    # Daha geniş güvenlik marjı
    safety_margin = 0.25  # 0.25'ten 0.4'e arttırdık
    
    # Engellerin ilerisini de kontrol et (tüm yolu değil, sadece belirli bir kısmını)
    for i, (px, py) in enumerate(zip(path_x, path_y)):
        # Aracın sınır kutusu (güvenlik marjı ile genişletilmiş)
        car_left = px - half_width - safety_margin
        car_right = px + half_width + safety_margin
        car_bottom = py - half_height - safety_margin
        car_top = py + half_height + safety_margin
        
        for obs in obstacles:
            ox, oy = obs.get_x(), obs.get_y()
            ox2, oy2 = ox + obs.get_width(), oy + obs.get_height()
            
            # Dikdörtgen-dikdörtgen çarpışma kontrolü
            if not (car_right < ox or car_left > ox2 or car_top < oy or car_bottom > oy2):
                return True
    
    return False

def evaluate_path_cost(path_x, path_y, goal, weight_dist=0.6, weight_smooth=0.4):
    """Yolları, hedefe uzaklık ve düzgünlük açısından değerlendir."""
    # Hedefe olan uzaklık maliyeti
    dist_cost = np.hypot(path_x[-1] - goal[0], path_y[-1] - goal[1])
    
    # Yol düzgünlüğünü ölç (açısal değişimler)
    smooth_cost = 0.0
    if len(path_x) > 2:
        for i in range(len(path_x)-2):
            # Ardışık üç nokta arasındaki açı değişimini hesapla
            v1 = [path_x[i+1] - path_x[i], path_y[i+1] - path_y[i]]
            v2 = [path_x[i+2] - path_x[i+1], path_y[i+2] - path_y[i+1]]
            
            # Vektörleri normalize et
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm > 0 and v2_norm > 0:
                v1 = [v1[0]/v1_norm, v1[1]/v1_norm]
                v2 = [v2[0]/v2_norm, v2[1]/v2_norm]
                
                # Açı değişimi (dot product)
                dot_product = max(min(v1[0]*v2[0] + v1[1]*v2[1], 1.0), -1.0)
                angle_change = np.arccos(dot_product)
                smooth_cost += angle_change
    
    # Toplam maliyet - düzgünlüğe daha fazla önem (0.2->0.4)
    total_cost = weight_dist * dist_cost + weight_smooth * smooth_cost
    return total_cost

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
        return [], None  # Return empty list and None for best_path

def animate(frame):
    """Animasyon döngüsü."""
    global vehicle, goal, ax, static_obstacles, lattice_paths, prev_best_path, current_path_time, best_path
    
    ax.clear()
    ax.set_xlim(0, 80)
    ax.set_ylim(-2.5, 2.5)
    
    # Engelleri yeniden çiz
    for obs in static_obstacles:
        ax.add_patch(obs)
    
    # Yol çizgileri
    for i in range(0, 81, 10):
        ax.plot([i, i], [-2.0, 2.0], 'y--', alpha=0.3)
    
    # Her karede yeniden planlama yap - ilk andan itibaren çatal yapısının görünmesi için
    lattice_paths = generate_lattice_paths((vehicle.x, vehicle.y), goal, num_paths=9, max_offset=1.8)
    valid_paths, new_best_path = lattice_planner(vehicle, goal, static_obstacles, lattice_paths)
    
    # Tüm yolları çiz
    for px, py in lattice_paths:
        ax.plot(px, py, 'gray', alpha=0.2)
    
    # Geçerli yolları çiz
    for px, py in valid_paths:
        ax.plot(px, py, 'c-', alpha=0.4)
    
    # Eğer yeni bir yol bulunursa veya yol yoksa değiştir
    if new_best_path is not None and (best_path is None or current_path_time > 10):
        best_path = new_best_path
        current_path_time = 0
    else:
        current_path_time += 1
    
    # Yol bulunamazsa önceki yolu kullanmaya devam et
    if best_path is None and 'prev_best_path' in globals() and prev_best_path is not None:
        best_path = prev_best_path
        # Önceki yol tekrar tekrar başarısız oluyorsa, aracı hafifçe kaydır
        vehicle.y += 0.02
    
    if best_path is not None:
        prev_best_path = best_path
        bx, by = best_path
        ax.plot(bx, by, 'g-', linewidth=2, label='En İyi Yörünge')
        
        # Daha uzun ileri görüşlü hedefleme için
        lookahead = min(15, len(bx) - 1)
        if len(bx) > lookahead:
            target_idx = lookahead
            dx = bx[target_idx] - vehicle.x
            dy = by[target_idx] - vehicle.y
            target_angle = np.arctan2(dy, dx)
            
            # Yumuşak direksiyon değişimi
            angle_diff = target_angle - vehicle.yaw
            while angle_diff > np.pi: angle_diff -= 2*np.pi
            while angle_diff < -np.pi: angle_diff += 2*np.pi
            
            # Daha yumuşak direksiyon değişimi için adaptif direksiyon açısı
            max_steer = 0.05
            
            # Mesafeye göre adaptif direksiyon
            steer_angle = np.clip(angle_diff, -max_steer, max_steer)
            
            # Küçük açılar için daha az direksiyon (düz gitmesini sağlar)
            if abs(angle_diff) < 0.05:
                steer_angle *= 0.2
        else:
            steer_angle = 0.0
            
        vehicle.move(0.06, steer_angle)
    else:
        # Yol bulunamazsa, aracı yavaşça ileri doğru götür
        vehicle.move(0.05, 0.0)
    
    # Aracı ve hedefi çiz
    ax.plot(goal[0], goal[1], 'go', markersize=12, label='Hedef')
    vehicle.plot(ax)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    
    # Hedefe yakınsa animasyonu sonlandır
    if np.hypot(vehicle.x - goal[0], vehicle.y - goal[1]) < 0.5:
        anim.event_source.stop()
        plt.title("Hedefe Ulaşıldı!")

# Başlangıç
fig, ax, static_obstacles = create_map()

# Başlangıç ve hedef noktalarını tanımlayın
vehicle = Vehicle(1.0, 0.0, 0.0, 0.8)
goal = (75.0, 0.0)

# Global değişkenleri ayarla
prev_best_path = None
best_path = None
current_path_time = 0

# Başlangıçta daha az çatal ve daha dar yollar oluştur
lattice_paths = generate_lattice_paths((vehicle.x, vehicle.y), goal, num_paths=9, max_offset=1.8)

# İlk yol planlaması
valid_paths, initial_best_path = lattice_planner(vehicle, goal, static_obstacles, lattice_paths)
if initial_best_path is not None:
    best_path = initial_best_path

# Animasyonu başlat
anim = FuncAnimation(fig, animate, frames=1500, interval=20, repeat=False)
plt.show()