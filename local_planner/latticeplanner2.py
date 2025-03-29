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

    # DÜZELTİLMİŞ: Tekrarlanan hedefi kaldırdım         
    nodes = [(18,-18),
             (-18,-18),
             (-18,-3),
             (-18,18),
             (18,18),
             (18,-3)
         ]

    # Her düğüm için numaralandırma ekledim
    for i, vertex in enumerate(nodes):
        ax.add_patch(patches.Circle(vertex, 3, fill=False, ec="red", lw=3))
        ax.add_patch(patches.Circle(vertex, 0.1, color="red"))
        # Hedef numarasını ekle
        ax.text(vertex[0]+1, vertex[1]+1, f"{i+1}", fontsize=12, color="black")
    
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


# NPC araçlarını daha belirgin olarak ayırt etmek için renk değişikliği
def add_car(ax, position=tuple(), rotation=0):
    x = position[0]
    y = position[1]

    car = patches.Rectangle(
        (x - 1.5, y - .7), 3, 1.4, angle=np.rad2deg(rotation),
        rotation_point='center', color='black'  # Siyah renk NPC araçlar için
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
def check_collision(path, obstacles, vehicle_radius=0.9, resolution=0.4, is_npc=False):
    """Bir yolun engellerle çarpışma kontrolü - NPC/duvar ayrımı ile"""
    jump = int(1/resolution)
    safe_steps = 0
    
    # NPC araçlar için güvenlik mesafesi - özellikle NPC10 için daha küçük değer
    actual_radius = 0.8 if is_npc else vehicle_radius  # 1.0 -> 0.8
    
    for point in path[::jump]:
        x, y = point
        
        for i, obstacle in enumerate(obstacles):
            # Dikdörtgen engelin sınırlarını al
            x_min = obstacle.get_x() 
            y_min = obstacle.get_y() 
            x_max = x_min + obstacle.get_width()
            y_max = y_min + obstacle.get_height()
            
            # NPC10 için özel güvenlik mesafesi (obstacles listesinde NPC10 son ikinci eleman)
            is_npc10 = (i == len(obstacles) - 2)
            special_radius = 0.8 if is_npc10 else actual_radius  # NPC10 için çok daha küçük yarıçap
            
            # Noktanın dikdörtgene en yakın noktasını bul
            closest_x = np.clip(x, x_min, x_max)
            closest_y = np.clip(y, y_min, y_max)
            
            # Aradaki mesafeyi hesapla
            distance = np.hypot(x - closest_x, y - closest_y)
            
            # Çarpışma kontrolü - NPC10 için özel değer kullan
            if distance <= (special_radius if is_npc10 else actual_radius):
                return safe_steps
        
        safe_steps += 1
    
    return safe_steps

# Yol değerlendirme fonksiyonunda geniş yollara öncelik ver
def evaluate_path(path, goal, weight_dist=0.6, weight_smooth=0.4):  # 0.75/0.25 -> 0.6/0.4
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
                v2_normalized = [v2[0]/v2_norm, v2_norm]
                
                # Açı değişimi (dot product)
                dot_product = max(min(v1_normalized[0]*v2_normalized[0] + 
                                      v1_normalized[1]*v2_normalized[1], 1.0), -1.0)
                angle_change = np.arccos(dot_product)
                smoothness_cost += angle_change
    
    # Toplam maliyet
    total_cost = weight_dist * dist_cost + weight_smooth * smoothness_cost
    return total_cost

# Conformal Lattice Planner parametrelerini optimize et - offset ve yol sayısını dengele
def conformal_lattice_planner(vehicle, goal, obstacles, num_paths=11, max_offset=3.5):  # 4.0 -> 3.5
    """Conformal Lattice Planner - daha geniş offsetli yollarla engelden kaçınır."""
    all_paths = generate_lattice_paths(
        (vehicle.x, vehicle.y), goal, 
        num_paths=num_paths, 
        max_offset=max_offset,
        vehicle_yaw=vehicle.yaw
    )
    
    valid_paths = []
    path_safety = []
    path_costs = []
    
    # Statik engelleri ve NPC araçlarını ayır
    static_obstacles = obstacles[:6]  # İlk 6 engel duvarlar
    npc_obstacles = obstacles[6:]     # Geri kalanlar NPC araçlar
    
    for path in all_paths:
        # Duvarlar için çarpışma kontrolü
        static_safety = check_collision(path, static_obstacles, vehicle_radius=0.75)  # 0.8 -> 0.75
        
        # NPC araçları için daha geniş çarpışma kontrolü
        npc_safety = check_collision(path, npc_obstacles, vehicle_radius=1.1, is_npc=True)  # 1.2 -> 1.1
        
        # Her iki kontrol de güvenli olmalı
        if static_safety > 0 and npc_safety > 0:
            # Yolun maliyetini hesapla
            cost = evaluate_path(path, goal, weight_dist=0.65, weight_smooth=0.35)  # 0.6/0.4 -> 0.65/0.35
            valid_paths.append(path)
            path_safety.append(min(static_safety, npc_safety))
            path_costs.append(cost)
    
    # En iyi yolu seç (önce güvenlik, sonra maliyet)
    best_path = None
    best_steering = 0.0
    
    # YENİ: Eğer hiç geçerli yol yoksa, en yakın geçersiz yolu kullan (acil durum)
    if not valid_paths and all_paths:
        # Tüm yollar için minimal güvenlik değerleri hesapla
        temp_static_safety = [check_collision(path, static_obstacles, vehicle_radius=0.7) for path in all_paths]
        temp_npc_safety = [check_collision(path, npc_obstacles, vehicle_radius=1.0, is_npc=True) for path in all_paths]
        
        # Her yol için minimal güvenlik değeri ve maliyet hesapla
        temp_path_safety = [min(s1, s2) for s1, s2 in zip(temp_static_safety, temp_npc_safety)]
        temp_path_costs = [evaluate_path(path, goal) for path in all_paths]
        
        # En az riskli yolu seç
        best_emergency_idx = np.argmax(temp_path_safety)
        best_path = all_paths[best_emergency_idx]
        
        # Yönlendirme açısını hesapla
        if len(best_path) > 1:
            target_point = best_path[min(5, len(best_path)-1)]  # 10 -> 5 (daha yakın hedef)
            dx = target_point[0] - vehicle.x
            dy = target_point[1] - vehicle.y
            target_angle = np.arctan2(dy, dx)
            
            # Yönlendirme açısı
            angle_diff = target_angle - vehicle.yaw
            # Açıyı normalize et
            while angle_diff > np.pi: angle_diff -= 2*np.pi
            while angle_diff < -np.pi: angle_diff += 2*np.pi
            
            # Daha keskin dönüşlere izin ver
            best_steering = np.clip(angle_diff, -np.pi/3, np.pi/3)
            
        
    
    elif valid_paths:
        # Normal yol seçimi
        max_safety = max(path_safety)
        safest_indices = [i for i, s in enumerate(path_safety) if s == max_safety]
        
        if safest_indices:
            best_idx = safest_indices[np.argmin([path_costs[i] for i in safest_indices])]
            best_path = valid_paths[best_idx]
            
            # Yönlendirme açısını hesapla
            if len(best_path) > 1:
                target_point = best_path[min(8, len(best_path)-1)]  # 10 -> 8 (daha yakın hedef)
                dx = target_point[0] - vehicle.x
                dy = target_point[1] - vehicle.y
                target_angle = np.arctan2(dy, dx)
                
                # Yönlendirme açısı
                angle_diff = target_angle - vehicle.yaw
                # Açıyı normalize et
                while angle_diff > np.pi: angle_diff -= 2*np.pi
                while angle_diff < -np.pi: angle_diff += 2*np.pi
                
                # Daha keskin dönüşlere izin ver
                best_steering = np.clip(angle_diff, -np.pi/3, np.pi/3)
    
    return all_paths, valid_paths, best_path, best_steering

# Animasyon fonksiyonunda daha keskin hareketlere izin ver
# Animasyon fonksiyonundaki planlayıcı çağrısını güncelliyoruz
def animate(frame):
    global ego_vehicle, goal, ax, counter, transitioning, global_nodes, transition_timer, stuck_timer, last_pos
    
    # Araç daha önce stuck oldu mu kontrol et
    if frame > 10 and 'last_pos' in globals():
        # Araç ne kadar hareket etti?
        dist_moved = np.hypot(ego_vehicle.x - last_pos[0], ego_vehicle.y - last_pos[1])
        
        # Neredeyse hiç hareket etmediyse stuck sayacını artır
        if dist_moved < 0.05:  # 5cm'den az hareket
            if 'stuck_timer' not in globals():
                stuck_timer = 0
            else:
                stuck_timer += 1
                
            # Kritik bölgede mi kontrol et (y=10 civarı)
            is_critical_area = abs(ego_vehicle.y - 10) < 5 and ego_vehicle.x > 10
            
            # Kritik bölgede daha hızlı tepki ver
            if stuck_timer > (10 if is_critical_area else 25):  # Kritik bölgede daha hızlı müdahale
                if is_critical_area:
                    # Y=10 civarında: Duvardan uzaklaş, sola doğru hareket et
                    escape_angle = np.pi  # Sola doğru (X ekseninin negatif yönüne)
                    ego_vehicle.yaw = escape_angle
                    # Daha güçlü hamle yap
                    ego_vehicle.move(0.8, 0)  # Daha hızlı kaç
                    # Duvardan uzaklaş
                    ego_vehicle.y += np.random.uniform(-2.0, 2.0) # Y yönünde rastgele bir hareket ekle
                    print("KRİTİK BÖLGEDE SIKIŞMA! Acil kaçış hareketi yapılıyor...")
                else:
                    # Normal stuck kurtulma
                    random_angle = ego_vehicle.yaw + np.random.uniform(-np.pi/4, np.pi/4)
                    ego_vehicle.yaw = random_angle
                    ego_vehicle.move(0.5, 0)
                    print("Stuck durumundan kurtulma hareketi...")
                
                stuck_timer = 0  # Sayacı sıfırla
    
    # Mevcut konumu kaydet (bir sonraki kontrolde kullanmak için)
    last_pos = (ego_vehicle.x, ego_vehicle.y)
    
    # Güncelle
    ax.clear()
    static_obstacles, _ = create_map(ax)  # Nodes'u kullanma, global_nodes kullan
    
    # Bilgi ekle - hangi hedefte olduğumuzu göster
    ax.set_title(f"Hedef: {counter}/{len(global_nodes)}, Konum: {goal}")
    
    # NPC araçları ekle
    npc1 = add_car(ax, (-10, 1), 0)
    npc2 = add_car(ax, (0, -1), 0)
    npc3 = add_car(ax, (10, 1), 0)
    
    npc4 = add_car(ax, (-10, 17), 0)
    npc5 = add_car(ax, (0, 19), 0)
    npc6 = add_car(ax, (10, 17), 0)
    
    npc7 = add_car(ax, (-10, -19), 0)
    npc8 = add_car(ax, (0, -17), 0)
    npc9 = add_car(ax, (10, -19), 0)
    
    # NPC10'u kaldır veya konumunu değiştir
    # npc10 = patches.Rectangle((19 - 0.7, 9 - 1.0), 1.4, 2.0, rotation_point='center', color='black')  # Kaldır

    # Alternatif: NPC10'u daha uygun bir konuma taşı
    npc10 = patches.Rectangle((17 - 0.7, 9 - 3.5), 1, 4, rotation_point='center', color='black')  # Duvardan uzaklaştır
    npc11 = patches.Rectangle((-19 - .7, -9 - 1.5), 1.4, 3, rotation_point='center', color='black')
    ax.add_patch(npc10)
    ax.add_patch(npc11)
    
    obstacles = static_obstacles + [npc1, npc2, npc3] + [npc4, npc5, npc6] + [npc7, npc8, npc9] + [npc10, npc11]
    
    # Planlayıcı çağrısı
    all_paths, valid_paths, best_path, best_steering = conformal_lattice_planner(
        ego_vehicle, goal, obstacles, num_paths=11, max_offset=4.0  # 9->11 yol ve 3.0->4.0 offset
    )
    
    # Tüm yolları çiz (güvenlik kontrolü)
    for path in all_paths:
        if path and len(path) > 1:
            x_path, y_path = zip(*path)
            ax.plot(x_path, y_path, 'gray', alpha=0.3)
    
    # Geçerli yolları çiz (güvenlik kontrolü)
    for path in valid_paths:
        if path and len(path) > 1:
            x_path, y_path = zip(*path)
            ax.plot(x_path, y_path, 'c-', alpha=0.5)
    
    # En iyi yolu çiz (güvenlik kontrolü)
    if best_path and len(best_path) > 1:
        x_best, y_best = zip(*best_path)
        ax.plot(x_best, y_best, 'g-', linewidth=2, label='En İyi Yörünge')
    
    # Hedefi çiz
    ax.plot(goal[0], goal[1], 'go', markersize=10, label='Hedef')
    
    # Aracı çiz
    ego_vehicle.plot(ax)
    
    # İlk 10 frame için çarpışma kontrolünü devre dışı bırakalım (başlangıçta stabilizasyon için)
    if frame > 10:
        # Yalnızca aracın geometrik merkezi için çarpışma kontrolü yapalım
        # Çok hassas çarpışma kontrolü yerine daha gerçekçi bir kontrol
        
        # Statik duvarlar için çarpışma kontrolü - daha dar yarıçap
        static_collision = check_collision([(ego_vehicle.x, ego_vehicle.y)], static_obstacles, 
                                          vehicle_radius=0.55, resolution=1)  # 0.6 -> 0.55
        
        # NPC araçlar için çarpışma kontrolü - gerçek çarpışma için daha dar yarıçap
        npc_obstacles = obstacles[len(static_obstacles):]
        npc_collision = check_collision([(ego_vehicle.x, ego_vehicle.y)], npc_obstacles, 
                                       vehicle_radius=0.7, resolution=1)  # 0.8 -> 0.7
        
        # Yalnızca gerçek çarpışma varsa bildirme
        if static_collision == 0 or npc_collision == 0:
            print("Fail - Çarpışma!")
            anim.event_source.stop()
            plt.pause(0.5)
            plt.close()
    
    # Aracı daha keskin hareketlerle hareket ettir
    if best_path and len(best_path) > 1:
        scaled_steering = best_steering * 0.98  # 0.95 -> 0.98 (daha keskin dönüş)
        ego_vehicle.move(0.12, scaled_steering)  # 0.1 -> 0.12 (daha hızlı)
    
    # Hedefe ulaşıldı mı kontrol et (mesafeyi 1.5'ten 1.8'e artırdık - daha kolay)
    if np.hypot(ego_vehicle.x - goal[0], ego_vehicle.y - goal[1]) < 1.8:
        if not transitioning:  # Henüz geçiş yapmadıysak
            transitioning = True
            transition_timer = 0  # Zamanlayıcıyı sıfırla
            counter += 1
            print(f"Hedef {counter-1} ulaşıldı, konum: {goal}")

            if counter >= len(global_nodes):  # Tüm hedeflere ulaşıldı mı?
                print("Tüm hedeflere ulaşıldı!")
                anim.event_source.stop()
                plt.pause(0.5)
                plt.close()
            else:
                # Yeni hedefi ayarla
                goal = global_nodes[counter]
                print(f"Yeni hedef ayarlandı: {goal}")

        else:
            # Geçiş sırasında aracın yönelimini kontrol et
            dx = goal[0] - ego_vehicle.x
            dy = goal[1] - ego_vehicle.y
            target_angle = np.arctan2(dy, dx)
            angle_diff = target_angle - ego_vehicle.yaw

            # Açıyı normalize et
            while angle_diff > np.pi: angle_diff -= 2*np.pi
            while angle_diff < -np.pi: angle_diff += 2*np.pi

            # Zamanlayıcıyı artır - stuck kalmasın diye
            transition_timer += 1
            
            # ÖNEMLİ YENİLİK: Ya doğru açıya ulaşırsa YA DA zaman aşımı olursa geçişi tamamla
            if abs(angle_diff) < 0.8 or transition_timer > 20:  # Açı eşiği (0.5 --> 0.8) + zaman aşımı
                transitioning = False
                print(f"Geçiş tamamlandı: açı farkı = {abs(angle_diff):.2f}, zaman = {transition_timer}")
                
    # Animasyon için bir şey döndürmek gerekiyor (bazı arayüzlerde)
    return []

# Düzeltme 1: Nodes listesini global olarak sakla
fig, ax = plt.subplots(figsize=(10, 10))
static_obstacles, global_nodes = create_map(ax)  # Global olarak bir kez oluştur

# 4. Araç parametrelerini başlangıçta ayarla - Daha hızlı ve daha keskin dönüşlü
# Düzeltme 2: Araç başlangıç konumu, yönünü ve hızını düzelt
ego_vehicle = Vehicle(global_nodes[0], np.pi, 1.2)  # 1.0 -> 1.2 hız
goal = global_nodes[1]
counter = 1
transitioning = False
transition_timer = 0
stuck_timer = 0  # Stuck kontrolü için sayaç ekle
last_pos = (0, 0)  # Son konum bilgisini sakla

# Animasyonu oluştur - daha fazla kare ve daha yavaş gösterim
anim = FuncAnimation(fig, animate, frames=800, interval=50, repeat=False, blit=False)

# Animasyonu göster
plt.show()

# Animasyonu kaydet
try:
    import os
    # Çıktı klasörünü oluştur
    output_dir = 'output'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # .gif formatında kaydet (daha güvenli)
    output_path = os.path.join(output_dir, 'conformal_lattice.gif')
    
    # PillowWriter kullan (ffmpeg gerektirmez)
    from matplotlib.animation import PillowWriter
    writer = PillowWriter(fps=20)
    anim.save(output_path, writer=writer)
    
    print(f"Simülasyon tamamlandı ve kaydedildi: {output_path}")
except Exception as e:
    print(f"Animasyon kaydedilirken hata: {e}")
    import traceback
    traceback.print_exc()
    print("Simülasyon tamamlandı ancak kaydedilemedi")