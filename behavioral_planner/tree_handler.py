# nodes klasöründeki davranış düğümlerini (nodeları) içe aktarıyoruz
from nodes.obstacle_nodes import IsObstacleAhead, StopVehicle
from nodes.traffic_light_nodes import IsTrafficLightAhead, IsTrafficLightRed, IsTrafficLightYellow, IsTrafficLightGreen, SlowDownAction, ProceedNormally
from nodes.traffic_sign_nodes import IsTrafficSignAhead, GetTrafficSignType
from nodes.traffic_sign_nodes import (
    IsPedestrianCrossingAhead, IsRoundaboutAhead, IsTrafficLightSignAhead,
    IsNoRightTurnAhead, IsNoLeftTurnAhead, IsNoEntryAhead,
    IsKeepRightAhead, IsKeepLeftAhead,
    IsMandatoryTurnRightAhead, IsMandatoryTurnLeftAhead, IsMandatoryGoStraightAhead,
    IsMandatoryGoStraightOrRightAhead, IsMandatoryGoStraightOrLeftAhead,
    IsMandatoryTurnRightAfterAhead, IsMandatoryTurnLeftAfterAhead,
    IsLaneMergeRightAhead, IsLaneMergeLeftAhead, IsTwoWayTrafficAhead,
    IsNoParkingAhead, IsParkingAreaAhead, IsTunnelAhead, IsBusStopAhead, IsStopSignAhead
)
from nodes.lane_nodes import IsWithinLane
from nodes.speed_limit_nodes import IsSpeedLimitRespected, AdjustSpeedToLimit
from nodes.wrong_way_nodes import IsWrongWayDriving, ReplanRoute
from nodes.parking_nodes import IsParkingAreaAhead, AllowParkingAction
from nodes.waypoint_nodes import IsAtWaypoint, UpdateToNextWaypoint

# py_trees kütüphanesini kullanıyoruz - davranış ağaçlarını oluşturmak için
import py_trees

class TreeHandler:
    def __init__(self, rdf_interface, ros_publisher_interface):
        self.rdf = rdf_interface
        self.ros_pub = ros_publisher_interface
        self.tree = self.build_full_tree()
        self.behaviour_tree = py_trees.trees.BehaviourTree(self.tree)

    def build_full_tree(self):
        # ENGEL kontrolü için node'lar
        obstacle_check = IsObstacleAhead(name="Engel Var mı?", rdf_interface=self.rdf)  # Engel algılandı mı?
        stop_action = StopVehicle(name="Dur", ros_publisher_interface=self.ros_pub)     # Engel varsa aracı durdur

        obstacle_sequence = py_trees.composites.Sequence(name="Engel Kontrolü")  # Sıralı çalışacak: önce kontrol, sonra aksiyon
        obstacle_sequence.add_children([obstacle_check, stop_action])

        # TRAFİK IŞIĞI kontrolü
        traffic_light_check = IsTrafficLightAhead(name="Trafik Işığı Var mı?", rdf_interface=self.rdf)  # Işık algılandı mı?

        # KIRMIZI ışık için: kontrol et ve dur
        red_sequence = py_trees.composites.Sequence(name="Kırmızı Işık")
        red_sequence.add_children([
            IsTrafficLightRed(name="Kırmızı mı?", rdf_interface=self.rdf),
            StopVehicle(name="Dur (Kırmızı)", ros_publisher_interface=self.ros_pub)
        ])

        # SARI ışık için: kontrol et ve yavaşla
        yellow_sequence = py_trees.composites.Sequence(name="Sarı Işık")
        yellow_sequence.add_children([
            IsTrafficLightYellow(name="Sarı mı?", rdf_interface=self.rdf),
            SlowDownAction(name="Yavaşla (Sarı)", ros_publisher_interface=self.ros_pub)
        ])

        # YEŞİL ışık için: kontrol et ve devam et
        green_sequence = py_trees.composites.Sequence(name="Yeşil Işık")
        green_sequence.add_children([
            IsTrafficLightGreen(name="Yeşil mi?", rdf_interface=self.rdf),
            ProceedNormally(name="Devam Et (Yeşil)", ros_publisher_interface=self.ros_pub)
        ])

        # Işık rengine göre uygun davranışı seç
        traffic_light_selector = py_trees.composites.Selector(name="Işık Rengine Göre Davranış")
        traffic_light_selector.add_children([red_sequence, yellow_sequence, green_sequence])

        # Ana trafik ışığı sequence: ışık var mı → rengi kontrol et → davran
        traffic_light_sequence = py_trees.composites.Sequence(name="Trafik Işığı Kontrolü")
        traffic_light_sequence.add_children([traffic_light_check, traffic_light_selector])

        # TRAFİK İŞARETİ kontrolü
        traffic_sign_check = IsTrafficSignAhead(name="Trafik İşareti Var mı?", rdf_interface=self.rdf)  # Herhangi bir tabela var mı?
        get_sign_type = GetTrafficSignType(name="İşaret Tipini Al", rdf_interface=self.rdf)             # Tipini al

        # Örnek: Dönel kavşak, dur levhası, yaya geçidi
        stop_sign_sequence = py_trees.composites.Sequence(name="Dur Levhası")
        stop_sign_sequence.add_children([
            IsStopSignAhead(name="Dur Levhası?", rdf_interface=self.rdf),
            StopVehicle(name="Dur (Levha)", ros_publisher_interface=self.ros_pub)
        ])

        pedestrian_sign_sequence = py_trees.composites.Sequence(name="Yaya Geçidi")
        pedestrian_sign_sequence.add_children([
            IsPedestrianCrossingAhead(name="Yaya Geçidi?", rdf_interface=self.rdf),
            SlowDownAction(name="Yavaşla (Yaya)", ros_publisher_interface=self.ros_pub)
        ])

        roundabout_sequence = py_trees.composites.Sequence(name="Dönel Kavşak")
        roundabout_sequence.add_children([
            IsRoundaboutAhead(name="Dönel Kavşak?", rdf_interface=self.rdf),
            ProceedNormally(name="Devam Et (Kavşak)", ros_publisher_interface=self.ros_pub)
        ])

        # Parallel yapı: birden fazla tabela aynı anda uygulanabilir
        traffic_sign_parallel = py_trees.composites.Parallel(
            name="İşaretlere Göre Davranışlar",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()  # Her çocuk çalışsın, başarılı olanlar davranışı uygulasın
        )
        traffic_sign_parallel.add_children([
            stop_sign_sequence,
            pedestrian_sign_sequence,
            roundabout_sequence
        ])

        # Trafik işareti sequence: önce kontrol et → tipi al → paralel davranışları uygula
        traffic_sign_sequence = py_trees.composites.Sequence(name="Trafik İşareti Kontrolü")
        traffic_sign_sequence.add_children([traffic_sign_check, get_sign_type, traffic_sign_parallel])

        # ŞERİT TAKİBİ
        lane_check = IsWithinLane(name="Şerit Takibi", rdf_interface=self.rdf)  # Şeritte kalınıyor mu?

        # HIZ SINIRI
        speed_check = IsSpeedLimitRespected(name="Hız Sınırı Uygun mu?", rdf_interface=self.rdf)
        speed_adjust = AdjustSpeedToLimit(name="Hızı Ayarla", ros_publisher_interface=self.ros_pub)
        speed_sequence = py_trees.composites.Sequence(name="Hız Kontrolü")  # Hız uygunsuzsa düzelt
        speed_sequence.add_children([speed_check, speed_adjust])

        # TERS YÖN
        wrong_way_check = IsWrongWayDriving(name="Ters Yön?", rdf_interface=self.rdf)
        replan = ReplanRoute(name="Yeniden Rota", ros_publisher_interface=self.ros_pub)
        wrong_way_sequence = py_trees.composites.Sequence(name="Ters Yön Kontrolü")
        wrong_way_sequence.add_children([wrong_way_check, replan])

        # PARK ALANI
        parking_check = IsParkingAreaAhead(name="Park Alanı Var mı?", rdf_interface=self.rdf)
        park_action = AllowParkingAction(name="Park Et", ros_publisher_interface=self.ros_pub)
        parking_sequence = py_trees.composites.Sequence(name="Park Kontrolü")
        parking_sequence.add_children([parking_check, park_action])

        # WAYPOINT kontrolü (hedef noktaya ulaşıldı mı?)
        at_waypoint = IsAtWaypoint(name="Hedef Noktadayız", rdf_interface=self.rdf)  # RDF'den hedefe ulaşıldı mı bilgisi alınır
        next_waypoint = UpdateToNextWaypoint(name="Sonraki Hedefe Geç", ros_publisher_interface=self.ros_pub)  # Yeni hedef belirlenir
        waypoint_sequence = py_trees.composites.Sequence(name="Waypoint Kontrolü")
        waypoint_sequence.add_children([at_waypoint, next_waypoint])

        # ÇEVRESEL DAVRANIŞLAR: Hepsi aynı anda kontrol edilmeli (engel, ışık, işaret, hız, ters yön, park)
        environment_parallel = py_trees.composites.Parallel(
            name="Çevresel Kontroller",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()  # Her çocuk çalışsın, uygulandıkça SUCCESS dönsün
        )

        environment_parallel.add_children([
            obstacle_sequence,
            traffic_light_sequence,
            traffic_sign_sequence,
            lane_check,
            speed_sequence,
            wrong_way_sequence,
            parking_sequence,
            waypoint_sequence  # EKLENDİ
        ])

        # AĞACIN KÖKÜ: Bu paralel yapı root olarak dönülür
        return environment_parallel

    def tick_tree(self):
        """
        Davranış ağacını bir kez çalıştırır (tick atar).
        Bu fonksiyon planner_node tarafından belirli aralıklarla çağrılır.
        """
        print("[BT] Davranış ağacı tick ediliyor...")
        self.behaviour_tree.tick_once()  # py_trees ile bir adım karar ver
