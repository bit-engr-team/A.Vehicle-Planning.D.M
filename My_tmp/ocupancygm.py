import open3d as o3d
import numpy as np
import yaml
from PIL import Image
import os
import math

def ply_to_occupancy_grid_files(ply_file_path, output_dir, resolution=0.05,
                                occupied_thresh=0.65, free_thresh=0.196,
                                map_height_range=None, default_value='unknown'):
    """
    Bir .ply dosyasını okur, 2D Occupancy Grid Map'e dönüştürür ve
    ROS map_server uyumlu .pgm ve .yaml dosyaları olarak kaydeder.

    Args:
        ply_file_path (str): Girdi .ply dosyasının yolu.
        output_dir (str): Çıktı .pgm ve .yaml dosyalarının kaydedileceği klasör.
        resolution (float): Haritanın metre cinsinden çözünürlüğü (bir hücrenin boyutu).
        occupied_thresh (float): PGM dosyasındaki işgal edilmişlik eşiği (map_server için).
                                 Genellikle 0.65 kullanılır.
        free_thresh (float): PGM dosyasındaki boş alan eşiği (map_server için).
                             Genellikle 0.196 kullanılır.
        map_height_range (tuple, optional): Haritaya dahil edilecek noktaların
                                           min ve max Z yükseklik aralığı (metre).
                                           None ise tüm noktalar kullanılır.
                                           Örnek: (0.1, 2.0) -> z=0.1m ile z=2.0m arası.
        default_value (str): Grid hücrelerinin başlangıç değeri ('free', 'occupied', 'unknown').
                             'unknown' tavsiye edilir.
    """
    print(f"Loading point cloud from: {ply_file_path}")
    try:
        pcd = o3d.io.read_point_cloud(ply_file_path)
        if not pcd.has_points():
            print("Error: Point cloud is empty or could not be read.")
            return
        points = np.asarray(pcd.points)
        print(f"Point cloud loaded with {len(points)} points.")
    except Exception as e:
        print(f"Error loading point cloud: {e}")
        return

    # --- Noktaları Yüksekliğe Göre Filtrele (Opsiyonel) ---
    if map_height_range is not None:
        min_z, max_z = map_height_range
        points = points[(points[:, 2] >= min_z) & (points[:, 2] <= max_z)]
        print(f"Filtered points by height ({min_z}m - {max_z}m): {len(points)} points remaining.")
        if len(points) == 0:
            print("Error: No points remaining after height filtering.")
            return

    if len(points) == 0:
         print("Error: Point cloud has no points (possibly after filtering).")
         return

    # --- Grid Sınırlarını ve Boyutlarını Hesapla ---
    min_coords = [-25.0,-25.0]#np.min(points[:, :2], axis=0)
    max_coords = [25.0,25.0]#np.max(points[:, :2], axis=0)
    print(min_coords)
    print()
    # Grid boyutlarını hesaplarken tavan fonksiyonu kullanıyoruz
    grid_width = math.ceil((max_coords[0] - min_coords[0]) / resolution)
    grid_height = math.ceil((max_coords[1] - min_coords[1]) / resolution)

    # Haritanın orijini (dünya koordinat sisteminde sol alt köşe)
    origin_x = min_coords[0]
    origin_y = min_coords[1]
    origin_yaw = 0.0  # Genellikle 2D projeksiyonda 0 olur

    print(f"Grid dimensions: {grid_width} x {grid_height}")
    print(f"Grid resolution: {resolution} m/cell")
    print(f"Map origin (bottom-left): ({origin_x:.3f}, {origin_y:.3f}) m")

    # --- Grid'i Oluştur ve Başlangıç Değeri Ata ---
    # PGM formatı için: 0 = işgal edilmiş, 255 = boş, 205 = bilinmeyen
    # ROS OccupancyGrid: 100 = işgal edilmiş, 0 = boş, -1 = bilinmeyen
    if default_value == 'free':
        initial_grid_value = 255 # PGM için boş
    elif default_value == 'occupied':
         initial_grid_value = 0 # PGM için dolu
    else: # 'unknown' veya geçersizse
        initial_grid_value = 205 # PGM için bilinmeyen

    # Dikkat: numpy dizisinde (row, col) yani (y, x) sırası kullanılır
    # Grid'i başlangıç değeriyle doldur
    grid_map = np.full((grid_height, grid_width), initial_grid_value, dtype=np.uint8)

    # --- Noktaları Grid Hücrelerine Dönüştür ve İşaretle ---
    # Dünya koordinatlarını grid indekslerine çevir
    # Floor işlemi ile hangi hücreye düştüğünü bul
    points_grid_x = np.floor((points[:, 0] - origin_x) / resolution).astype(int)
    points_grid_y = np.floor((points[:, 1] - origin_y) / resolution).astype(int)

    # Grid sınırları içinde kalan geçerli indeksleri al
    valid_indices = (points_grid_x >= 0) & (points_grid_x < grid_width) & \
                    (points_grid_y >= 0) & (points_grid_y < grid_height)

    valid_grid_x = points_grid_x[valid_indices]
    valid_grid_y = points_grid_y[valid_indices]

    # PGM formatında (0,0) sol üst köşedir, ancak harita orijini sol alttadır.
    # Bu nedenle y eksenini ters çevirmemiz gerekir.
    # Numpy'da [row, col] -> [height - 1 - grid_y, grid_x]
    pgm_y_indices = grid_height - 1 - valid_grid_y

    # İlgili hücreleri "işgal edilmiş" (0) olarak işaretle
    grid_map[pgm_y_indices, valid_grid_x] = 0 # 0 = occupied in PGM

    # --- Çıktı Dosyalarını Oluştur ---
    os.makedirs(output_dir, exist_ok=True) # Klasör yoksa oluştur
    base_name = os.path.splitext(os.path.basename(ply_file_path))[0]
    pgm_file_path = os.path.join(output_dir, f"{base_name}.pgm")
    yaml_file_path = os.path.join(output_dir, f"{base_name}.yaml")

    # --- PGM Dosyasını Kaydet ---
    try:
        img = Image.fromarray(grid_map, mode='L') # 'L' modu 8-bit grayscale
        img.save(pgm_file_path)
        print(f"Occupancy grid map saved as PGM: {pgm_file_path}")
    except Exception as e:
        print(f"Error saving PGM file: {e}")
        return

    # --- YAML Dosyasını Oluştur ve Kaydet ---
    yaml_data = {
        'image': os.path.basename(pgm_file_path), # Sadece dosya adı
        'resolution': resolution,
        'origin': [origin_x, origin_y, origin_yaw], # [x, y, yaw]
        'negate': 0, # PGM'de siyah = dolu, beyaz = boş ise 0 olmalı
        'occupied_thresh': occupied_thresh, # 0 değeri (siyah) bu eşiğin üzerinde kabul edilir
        'free_thresh': free_thresh,       # 255 değeri (beyaz) bu eşiğin altında kabul edilir
        # Not: 205 (bilinmeyen) bu iki eşik arasında kalır.
    }

    try:
        with open(yaml_file_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=None)
        print(f"Map metadata saved as YAML: {yaml_file_path}")
    except Exception as e:
        print(f"Error saving YAML file: {e}")
        return

    print("Conversion complete.")


# --- KODU KULLANMA ---
if __name__ == "__main__":
    # Girdi ve Çıktı Ayarları
    input_ply = "input/3.367949.ply"  # .ply dosyanızın adını buraya yazın
    output_folder = "output/harita_dosyalari" # Çıktıların kaydedileceği klasör

    # Harita Parametreleri
    map_resolution = .5  # Metre cinsinden çözünürlük (örn: 5cm)
    # Sadece belirli yükseklikteki engelleri haritala (örn: zeminden 10cm yukarısı ile 2m arası)
    # Eğer tüm Z değerleri dahil edilecekse None yapın: height_filter = None
    height_filter = (-1, 2)

    # Kontrol: Örnek bir .ply dosyası oluşturalım (eğer yoksa)
    if not os.path.exists(input_ply):
        print(f"Warning: Input file '{input_ply}' not found. Creating a sample PLY file.")
        # Basit bir küp ve zemin noktaları oluşturalım
        points = []
        # Zemin (10x10 metrekare, z=0)
        for x in np.linspace(-5, 5, 20):
            for y in np.linspace(-5, 5, 20):
                points.append([x, y, 0.0])
        # Küp (1x1x1 metre, merkezde, z=0.5 üzerinde)
        for x in np.linspace(-0.5, 0.5, 5):
            for y in np.linspace(-0.5, 0.5, 5):
                points.append([x, y, 0.5]) # Alt yüzey
                points.append([x, y, 1.5]) # Üst yüzey
        for x in np.linspace(-0.5, 0.5, 5):
            for z in np.linspace(0.5, 1.5, 5):
                points.append([x, -0.5, z]) # Yan yüzeyler
                points.append([x, 0.5, z])
        for y in np.linspace(-0.5, 0.5, 5):
             for z in np.linspace(0.5, 1.5, 5):
                points.append([-0.5, y, z])
                points.append([0.5, y, z])

        pcd_sample = o3d.geometry.PointCloud()
        pcd_sample.points = o3d.utility.Vector3dVector(np.array(points))
        o3d.io.write_point_cloud(input_ply, pcd_sample)
        print(f"Sample PLY file '{input_ply}' created.")


    # Dönüştürme fonksiyonunu çağır
    ply_to_occupancy_grid_files(
        ply_file_path=input_ply,
        output_dir=output_folder,
        resolution=map_resolution,
        map_height_range=height_filter,
        default_value='unknown' # Başlangıçta tüm hücreler bilinmiyor olsun
    )