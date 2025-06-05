import py_trees
import math

#Parallelin fallback nodeunun çocukları
class IsAtNextWaypoint_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Next WP'ye Ulaşıldı mı?", rdf_interface=None, threshold=1.5):
        super().__init__(name)
        self.rdf = rdf_interface
        self.threshold = threshold  # metre cinsinden tolerans mesafesi

    def update(self):
        if self.rdf is None:
            return py_trees.common.Status.FAILURE

        try:
            vehicle_pos = self.rdf.get_vehicle_position()   # (x, y) tuple bekleniyor
            waypoint_pos = self.rdf.get_next_waypoint_position()  # (x, y) tuple

            if vehicle_pos is None or waypoint_pos is None:
                print("[WAYPOINT] Konum verisi eksik.")
                return py_trees.common.Status.FAILURE

            # Öklidyen mesafe hesapla
            dx = vehicle_pos[0] - waypoint_pos[0]
            dy = vehicle_pos[1] - waypoint_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)

            print(f"[WAYPOINT] Mevcut mesafe: {distance:.2f} m")
            if distance < self.threshold:
                print("[WAYPOINT] Hedef waypoint'e ulaşıldı.")
                return py_trees.common.Status.SUCCESS
            else:
                print("[WAYPOINT] Hedefe henüz ulaşılmadı.")
                return py_trees.common.Status.FAILURE

        except Exception as e:
            print(f"[WAYPOINT] RDF veya mesafe hatası: {e}")
            return py_trees.common.Status.FAILURE


class MoveToWaypoint_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Waypointe hareket et", decision_callback=None):
        super().__init__(name)
        self.callback = decision_callback  # Local planner'a komut iletmek için callback fonksiyonu

    def update(self):
        print("[WAYPOINT] Aracın hedef waypoint'e yönlendirilmesi isteniyor.")
        if self.callback:
            self.callback("MOVE_TO_WAYPOINT")  # Local planner'a gönderilecek komut
        return py_trees.common.Status.SUCCESS
    

#Parallelin parallel nodeunun çocukları a.k.a görev nodeları
class IsBusStop_G(py_trees.behaviour.Behaviour):
    def __init__(self, rdf_interface, current_position, current_waypoint, name="Durak mı?"):
        super().__init__(name)
        self.rdf = rdf_interface  # RDF sorguları için arayüz
        self.position = current_position  # Aracın mevcut pozisyonu (x, y)
        self.waypoint = current_waypoint  # Beklenen otobüs durağı waypoint'i (x, y)

    def update(self):
        # RDF üzerinden durak tabelasının varlığını kontrol et
        query = """
        ASK WHERE {
            ?p a :BusStop ; :isDetected true .
        }
        """
        traffic_sign_detected = self.rdf.ask_query(query)

        # Konumsal eşleşme kontrolü (örnek eşik: 3 metre)
        dx = self.position[0] - self.waypoint[0]
        dy = self.position[1] - self.waypoint[1]
        distance = (dx**2 + dy**2)**0.5

        if traffic_sign_detected and distance < 3.0:
            print("[WAYPOINT] Durak tabelası ve uygun konum doğrulandı.")
            return py_trees.common.Status.SUCCESS
        else:
            print("[WAYPOINT] Durak koşulu sağlanmadı.")
            return py_trees.common.Status.FAILURE


class PickupPassenger_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Yolcu al", decision_callback=None):
        super().__init__(name)
        self.callback = decision_callback  # Local planner'a komut iletmek için callback fonksiyonu

    def update(self):
        print("[YOLCU] Yolcu alınıyor...")
        if self.callback:
            self.callback("PICKUP_PASSENGER")  # Yolcu al komutu
        return py_trees.common.Status.SUCCESS
    

class IsParkingArea_G(py_trees.behaviour.Behaviour):
    def __init__(self, rdf_interface, current_position, current_waypoint, name="Park yeri mi?"):
        super().__init__(name)
        self.rdf = rdf_interface
        self.position = current_position
        self.waypoint = current_waypoint

    def update(self):
        query = """
        ASK WHERE {
            ?p a :ParkingArea ; :isDetected true .
        }
        """
        traffic_sign_detected = self.rdf.ask_query(query)

        dx = self.position[0] - self.waypoint[0]
        dy = self.position[1] - self.waypoint[1]
        distance = (dx**2 + dy**2)**0.5

        if traffic_sign_detected and distance < 3.0:
            print("[WAYPOINT] Park alanı tabelası ve uygun konum doğrulandı.")
            return py_trees.common.Status.SUCCESS
        else:
            print("[WAYPOINT] Park koşulu sağlanmadı.")
            return py_trees.common.Status.FAILURE
        

class StopForMission_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Park alanında dur", decision_callback=None):
        super().__init__(name)
        self.callback = decision_callback

    def update(self):
        print("[PARK] Park konumunda duruluyor...")
        if self.callback:
            self.callback("STOP_FOR_MISSION")  # Park noktasında dur komutu
        return py_trees.common.Status.SUCCESS
    

class FinishMission_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Görevi Bitir", decision_callback=None):
        super().__init__(name)
        self.callback = decision_callback

    def update(self):
        print("[MİSYON] Görev başarıyla tamamlandı.")
        if self.callback:
            self.callback("FINISH_MISSION")  # Görevi sonlandırma komutu
        return py_trees.common.Status.SUCCESS


class UpdateToNextWaypoint_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Waypoint'i güncelle", decision_callback=None):
        super().__init__(name)
        self.callback = decision_callback

    def update(self):
        print("[WAYPOINT] Waypoint güncelleniyor...")
        if self.callback:
            self.callback("UPDATE_NEXT_WAYPOINT")  # Yeni waypoint isteği mission planner'a gönderilir
        return py_trees.common.Status.SUCCESS
