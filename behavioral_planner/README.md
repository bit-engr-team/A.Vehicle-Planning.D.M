# Behavioral Planner

Bu proje, otonom araçlar için davranış planlayıcı (behavioral planner) modülünü içerir. Davranış ağacı (behavior tree) yapısı kullanılarak, aracın çevresel koşullara ve görevlere göre kararlar alması sağlanır.

## Giriş/Çıkış Formatları

### Giriş Verileri

1. **Perception Verileri**
   ```json
   {
     "timestamp": float,
     "lane_lines": [
       {
         "points": [[float, float], [float, float]]
       }
     ],
     "lane_centerlines": [
       {
         "points": [[float, float], [float, float]]
       }
     ],
     "drivable_area": {
       "boundary_points": [[float, float], [float, float], [float, float], [float, float]],
       "area": float,
       "mask_shape": [int, int]
     },
     "objects": [
       {
         "class": "string",
         "distance": float,
         "type": "string"
       }
     ]
   }
   ```

2. **Mission Planner Verileri**
   ```json
   {
     "waypoints": [
       {
         "id": "string",
         "position": {"x": float, "y": float, "z": float},
         "type": "string",
         "action": "string"
       }
     ],
     "current_mission": {
       "id": "string",
       "status": "string",
       "priority": integer
     }
   }
   ```

### Çıkış Verileri

1. **Aksiyon Komutları**
   ```json
   {
     "action_type": "string",  // Örnek: "MOVE_TO_WAYPOINT", "STOP", "PICKUP_PASSENGER"
     "parameters": {
       "target_position": {"x": float, "y": float, "z": float},
       "velocity": float,
       "acceleration": float,
       "steering_angle": float
     },
     "priority": integer,
     "timestamp": "string"
   }
   ```

2. **Durum Bilgisi**
   ```json
   {
     "current_state": "string",
     "active_behaviors": ["string"],
     "warnings": ["string"],
     "timestamp": "string"
   }
   ```

## Modül Entegrasyonu

### 1. Perception Modülü Entegrasyonu

- **Veri Akışı**: Perception modülünden gelen ham sensör verileri `perception_converter.py` üzerinden RDF formatına dönüştürülür
- **Entegrasyon Noktası**: `bp_main.py` içindeki `update_from_perception()` metodu
- **Veri Güncelleme Sıklığı**: 100ms (10Hz)
- **Veri Dönüşümü**: `PerceptionConverter` sınıfı JSON verisini Turtle formatında RDF'e dönüştürür

### 2. Mission Planner Entegrasyonu

- **Veri Akışı**: Mission planner'dan gelen waypoint ve görev bilgileri davranış ağacındaki `RequestWaypointFromMissionPlanner_G` node'u üzerinden alınır
- **Entegrasyon Noktası**: `nodes/waypoint_nodes_G.py` içindeki `RequestWaypointFromMissionPlanner_G` sınıfı
- **Veri Güncelleme Sıklığı**: 1s (1Hz)
- **Callback Mekanizması**: Waypoint istekleri callback fonksiyonu üzerinden iletilir

### 3. Motion Planner Entegrasyonu

- **Veri Akışı**: Behavioral planner'dan çıkan aksiyon komutları callback mekanizması ile motion planner'a iletilir
- **Entegrasyon Noktası**: Tüm aksiyon node'ları (`action_executer_node_G.py` içindeki sınıflar)
- **Veri Güncelleme Sıklığı**: 50ms (20Hz)
- **Aksiyon Tipleri**: 
  - `MOVE_TO_WAYPOINT`: Hedef waypoint'e hareket
  - `STOP`: Durma komutu
  - `PICKUP_PASSENGER`: Yolcu alma
  - `STOP_FOR_MISSION`: Görev için durma
  - `FINISH_MISSION`: Görevi bitirme

### 4. RDF Arayüzü Entegrasyonu

- **Veri Akışı**: Semantic web verileri RDF formatında saklanır ve işlenir
- **Entegrasyon Noktası**: `rdf_interface_G.py` içindeki SPARQL sorguları
- **Veri Güncelleme Sıklığı**: 200ms (5Hz)
- **Veri Tipleri**:
  - Araç pozisyonu
  - Engel bilgileri
  - Trafik işaretleri
  - Waypoint bilgileri

## Kurulum

1. Python 3.x gereklidir
2. Gerekli kütüphaneleri yükleyin:
```bash
pip install rdflib py_trees numpy
```

## Proje Yapısı

```
behavioral_planner/
├── nodes/
│   ├── waypoint_nodes_G.py
│   ├── action_executer_node_G.py
│   └── cevresel_kontroller_G/
│       ├── dynamic_obstacle_nodes_G.py
│       └── traffic_sign_nodes_G.py
├── tree_handler_G.py
├── rdf_interface_G.py
├── perception_converter.py
├── bp_main.py
└── test_behavioral_planner_mock_G.py
```

## Test

Test dosyasını çalıştırmak için:
```bash
python3 test_behavioral_planner_mock_G.py
```

## Geliştirme

Yeni davranışlar eklemek için:

1. İlgili node sınıfını `nodes/` dizini altında oluşturun
2. `tree_handler_G.py` içinde davranış ağacına ekleyin
3. Gerekli RDF sorgularını `rdf_interface_G.py` içinde tanımlayın
4. Giriş/çıkış formatlarını yukarıda belirtilen JSON şemalarına uygun olarak güncelleyin

## Lisans

Bu proje [lisans bilgisi] altında lisanslanmıştır. 