from behavioral_planner.rdf_interface_G import RDFInterface_G
from behavioral_planner.tree_handler_G import create_behavior_tree
from behavioral_planner.perception_converter import PerceptionConverter

import py_trees

class BehavioralPlanner:
    def __init__(self):
        # RDF arayüzü başlatılır
        self.rdf_interface = RDFInterface_G()
        
        # Perception converter başlatılır
        self.perception_converter = PerceptionConverter()

        # Local planner'a aktarılacak kararları yakalamak için callback tanımlanır
        self.latest_decision = None
        def decision_callback(command):
            print(f"[CALLBACK] Local planner'a iletilen komut: {command}")
            self.latest_decision = command

        # Davranış ağacı oluşturulur
        self.root = create_behavior_tree(rdf_interface=self.rdf_interface,
                                         decision_callback=decision_callback)
        self.behaviour_tree = py_trees.trees.BehaviourTree(self.root)

    def update_from_perception(self, perception_data):
        """
        Perception modülünden gelen veriyi RDF formatına dönüştürüp günceller.
        
        Args:
            perception_data (dict): Perception modülünden gelen JSON formatındaki veri
        """
        try:
            # Perception verisini RDF'e dönüştür
            rdf_data = self.perception_converter.convert_perception_data(perception_data)
            
            # RDF verisini doğrula
            if self.perception_converter.validate_rdf(rdf_data):
                # RDF verisini güncelle
                self.update_rdf(rdf_data)
                return True
            else:
                print("[ERROR] Invalid RDF data generated from perception data")
                return False
        except Exception as e:
            print(f"[ERROR] Failed to update from perception data: {e}")
            return False

    def update_rdf(self, rdf_turtle_string):
        """
        Dışarıdan gelen Turtle formatındaki RDF verisi ile graf güncellenir.
        """
        self.rdf_interface.load_rdf_data(rdf_turtle_string)

    def tick(self):
        """
        Davranış ağacı bir adım (tick) çalıştırılır.
        """
        print("[TICK] Davranış ağacı çalıştırılıyor...")
        self.behaviour_tree.tick_once()
        return self.latest_decision  # Son karar çıktısı dışarıya döndürülür
