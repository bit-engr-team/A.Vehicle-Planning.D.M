import sys
import os
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from behavioral_planner.mock_rdf_interface_G import MockRDFInterface_G
from behavioral_planner.tree_handler_G import create_behavior_tree
import py_trees

def run_test_scenario(scenario_name, mock_rdf, expected_decisions, num_ticks=5):
    """
    Belirli bir test senaryosunu çalıştırır.
    
    Args:
        scenario_name (str): Senaryo adı
        mock_rdf (MockRDFInterface_G): Mock RDF arayüzü
        expected_decisions (list): Beklenen kararlar listesi
        num_ticks (int): Kaç kez çalıştırılacağı
    """
    print(f"\n[TEST] Senaryo: {scenario_name}")
    print("-" * 50)
    
    # Callback fonksiyonu çıktıyı toplamak için
    decisions = []
    def decision_callback(command):
        decisions.append(command)
        print(f"[CALLBACK] Karar: {command}")

    # Davranış ağacı oluşturuluyor
    root = create_behavior_tree(rdf_interface=mock_rdf, decision_callback=decision_callback)
    behaviour_tree = py_trees.trees.BehaviourTree(root)

    # Davranış ağacı belirtilen sayıda çalıştırılıyor
    for i in range(num_ticks):
        print(f"\n[TICK] {i+1}. adım")
        behaviour_tree.tick_tock(period_ms=100)
        time.sleep(0.1)  # Çıktıları okunabilir yapmak için kısa bekleme

    # Sonuçları kontrol et
    print("\n[TEST] Sonuçlar:")
    print(f"Beklenen kararlar: {expected_decisions}")
    print(f"Alınan kararlar: {decisions}")
    
    # Basit bir doğrulama
    if any(exp in decisions for exp in expected_decisions):
        print("[TEST] ✅ Senaryo başarılı!")
    else:
        print("[TEST] ❌ Senaryo başarısız!")

def main():
    # Senaryo 1: Trafik ışığı ve yaya geçidi
    mock_rdf1 = MockRDFInterface_G(
        detected_types=["TRAFFIC_LIGHT", "PEDESTRIAN_CROSSING"],
        obstacle_on_crosswalk=True,
        traffic_light_color="RED"
    )
    run_test_scenario(
        "Trafik Işığı ve Yaya Geçidi",
        mock_rdf1,
        ["dur", "yaya_varsa_dur"]
    )

    # Senaryo 2: Waypoint takibi
    mock_rdf2 = MockRDFInterface_G(
        detected_types=[],
        obstacle_on_crosswalk=False,
        traffic_light_color="GREEN"
    )
    mock_rdf2.set_vehicle_position((0, 0))
    mock_rdf2.set_next_waypoint_position((5, 5))
    run_test_scenario(
        "Waypoint Takibi",
        mock_rdf2,
        ["MOVE_TO_WAYPOINT"]
    )

    # Senaryo 3: Engel kontrolü
    mock_rdf3 = MockRDFInterface_G(
        detected_types=[],
        obstacle_on_crosswalk=False,
        traffic_light_color="GREEN"
    )
    run_test_scenario(
        "Engel Kontrolü",
        mock_rdf3,
        ["STOP", "SLOWDOWN"]
    )

    # Senaryo 4: Otobüs durağı
    mock_rdf4 = MockRDFInterface_G(
        detected_types=["BUS_STOP"],
        obstacle_on_crosswalk=False,
        traffic_light_color="GREEN"
    )
    run_test_scenario(
        "Otobüs Durağı",
        mock_rdf4,
        ["STOP_FOR_MISSION", "PICKUP_PASSENGER"]
    )

if __name__ == "__main__":
    main()
