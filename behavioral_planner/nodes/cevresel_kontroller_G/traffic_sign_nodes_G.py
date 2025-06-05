import py_trees
from behavioral_planner_G.rdf_interface_G import RDFInterface_G

class IsTrafficSignDetected_G(py_trees.behaviour.Behaviour):
    def __init__(self, rdf_interface, name="Trafik işareti var mı?"):
        super().__init__(name)  # py_trees davranış sınıfı başlatılır
        self.rdf = rdf_interface  # RDF sorguları için arayüz alınır

    def update(self):
        # Trafik işareti algılanıp algılanmadığını kontrol eder
        query = """
        ASK WHERE {
            {
                ?s a :TrafficSign ; :isDetected true .
            }
            UNION
            {
                ?l a :TrafficLight ; :isDetected true .
            }
            UNION
            {
                ?c a :Crosswalk ; :isDetected true .
            }
        }
        """
        result = self.rdf.ask_query(query)  # Sorguyu çalıştır

        if result:
            print("[TRAFİK İŞARETİ] Trafik işareti algılandı.")
            return py_trees.common.Status.SUCCESS
        else:
            print("[TRAFİK İŞARETİ] Trafik işareti algılanmadı.")
            return py_trees.common.Status.FAILURE


class GetTrafficSignType_G(py_trees.behaviour.Behaviour):
    def __init__(self, rdf_interface, name="Trafik işareti tipini al"):
        super().__init__(name)  # py_trees davranış sınıfı başlatılır
        self.rdf = rdf_interface  # RDF arayüzü alınır
        self.detected_types = []  # Algılanan tüm işaret türleri burada tutulur

    def update(self):
        # Algılanan tüm işaretlerin türlerini almak için SPARQL sorgusu
        query = """
        SELECT DISTINCT ?type WHERE {
            ?s a ?type ; :isDetected true .
            FILTER(?type IN (:TrafficSign, :TrafficLight, :Crosswalk))
        }
        """
        results = self.rdf.select_query(query)

        self.detected_types = []  # Listeyi her seferinde sıfırla
        for row in results:
            detected = str(row.type).split("#")[-1]  # URI'den sınıf ismini al (örn. TrafficLight)
            self.detected_types.append(detected)
            print(f"[TRAFİK İŞARETİ] Algılanan nesne türü: {detected}")

        if self.detected_types:
            return py_trees.common.Status.SUCCESS

        print("[TRAFİK İŞARETİ] İşaret türü belirlenemedi.")
        return py_trees.common.Status.FAILURE

    def get_detected_types(self):
        return self.detected_types


class UpdateBehaviorFromSign_G(py_trees.behaviour.Behaviour):
    def __init__(self, rdf_interface=None, name="İşarete göre davranış üret", decision_callback=None):
        super().__init__(name)
        self.rdf = rdf_interface
        self.callback = decision_callback  # Local planner'a komut iletmek için callback

    def update(self):
        decision_candidates = []  # Aksiyon ve öncelik içeren liste

        # RDF üzerinden algılanan işaret türlerini al
        sign_types = self.rdf.get_detected_types() if hasattr(self.rdf, 'get_detected_types') else []

        for sign in sign_types:
            # Her bir işaret türü için aksiyon ve öncelik belirle
            if sign == "STOP":
                decision_candidates.append(("dur", 100))
            elif sign == "NO_ENTRY":
                decision_candidates.append(("girme", 95))
            elif sign == "NO_LEFT_TURN":
                decision_candidates.append(("sola_donme", 90))
            elif sign == "NO_RIGHT_TURN":
                decision_candidates.append(("saga_donme", 90))
            elif sign == "NO_PARKING":
                decision_candidates.append(("park_etme", 80))
            elif sign == "PEDESTRIAN_CROSSING":
                if self.rdf.is_obstacle_on_crosswalk():
                    decision_candidates.append(("yaya_varsa_dur", 85))
                else:
                    decision_candidates.append(("devam_et", 20))
            elif sign == "TRAFFIC_LIGHT":
                color = self.rdf.get_traffic_light_color()
                if color == "RED":
                    decision_candidates.append(("dur", 90))
                elif color == "YELLOW":
                    decision_candidates.append(("yavasla", 60))
                elif color == "GREEN":
                    decision_candidates.append(("devam_et", 10))
            elif sign == "ROUNDABOUT":
                decision_candidates.append(("kavsaga_gir", 75))
            elif sign == "TUNNEL":
                decision_candidates.append(("tunele_gir", 60))
            elif sign == "TWO_WAY_TRAFFIC":
                decision_candidates.append(("çift_yön_dikkat", 60))
            elif sign == "LANE_MERGE_RIGHT":
                decision_candidates.append(("saga_kay", 65))
            elif sign == "LANE_MERGE_LEFT":
                decision_candidates.append(("sola_kay", 65))
            elif sign == "MANDATORY_GO_STRAIGHT":
                decision_candidates.append(("düz_git", 70))
            elif sign == "MANDATORY_TURN_LEFT":
                decision_candidates.append(("sola_don", 70))
            elif sign == "MANDATORY_TURN_RIGHT":
                decision_candidates.append(("saga_don", 70))
            elif sign == "MANDATORY_STRAIGHT_OR_LEFT":
                decision_candidates.append(("düz_veya_sola", 65))
            elif sign == "MANDATORY_STRAIGHT_OR_RIGHT":
                decision_candidates.append(("düz_veya_saga", 65))
            elif sign == "MANDATORY_TURN_LEFT_AFTER":
                decision_candidates.append(("ileride_sola_don", 60))
            elif sign == "MANDATORY_TURN_RIGHT_AFTER":
                decision_candidates.append(("ileride_saga_don", 60))
            elif sign == "KEEP_LEFT":
                decision_candidates.append(("solda_kal", 40))
            elif sign == "KEEP_RIGHT":
                decision_candidates.append(("sagda_kal", 40))
            elif sign == "PARKING_AREA":
                decision_candidates.append(("park_izinli", 30))
            elif sign == "BUS_STOP":
                decision_candidates.append(("otobus_duragi", 25))

        if decision_candidates:
            # En yüksek öncelikli kararı uygula
            action = sorted(decision_candidates, key=lambda x: -x[1])[0][0]
            print(f"[DAVRANIŞ GÜNCELLEME] Uygulanacak karar: {action}")
            if self.callback:
                self.callback(action)  # Callback ile komut aktar
            return py_trees.common.Status.SUCCESS

        # Hiçbir karar yoksa FAILURE dön
        print("[DAVRANIŞ GÜNCELLEME] Uygun karar bulunamadı.")
        return py_trees.common.Status.FAILURE
