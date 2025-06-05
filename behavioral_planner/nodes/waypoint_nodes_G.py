import py_trees
from behavioral_planner_G.rdf_interface_G import RDFInterface_G

class IsWaypointAvailable_G(py_trees.behaviour.Behaviour):
    def __init__(self, rdf_interface, name="Waypoint var mı?"):
        super().__init__(name)
        self.rdf = rdf_interface

    def update(self):
        # SPARQL ile aktif bir waypoint olup olmadığını kontrol eder
        query = """
        ASK WHERE {
            ?v a :Vehicle ; :hasActiveWaypoint true .
        }
        """
        result = self.rdf.ask_query(query)
        if result:
            print("[WAYPOINT] Aktif waypoint bulundu.")
            return py_trees.common.Status.SUCCESS
        else:
            print("[WAYPOINT] Aktif waypoint bulunamadı.")
            return py_trees.common.Status.FAILURE

class RequestWaypointFromMissionPlanner_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Mission Planner ile waypoint getir", decision_callback=None): 
        super().__init__(name)
        self.callback = decision_callback  # Mission planner'a komut iletmek için callback fonksiyonu 

    def update(self):
        # Gerçek uygulamada bu kısım mission planner'dan veri çekme işlemidir
        # Şu anlık sahte bir örnek davranış: %70 ihtimalle başarılı
        success = random.random() < 0.7
        if success:
            print("[WAYPOINT] Mission planner'dan waypoint alındı.")
            if self.callback:
                self.callback("WAYPOINT_REQUEST_SUCCESS")  # Örnek: waypoint başarıyla alındı komutu
            return py_trees.common.Status.SUCCESS
        else:
            print("[WAYPOINT] Mission planner'dan waypoint alınamadı.")
            if self.callback:
                self.callback("WAYPOINT_REQUEST_FAIL")  # Örnek: başarısızlık durumu
            return py_trees.common.Status.FAILURE


class CreateTemporaryWaypoint_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Geçici waypoint oluştur", decision_callback=None):
        super().__init__(name)
        self.callback = decision_callback  

    def update(self):
        # Gerçek sistemde sabit/geçici bir koordinat RDF'e yazılabilir
        print("[WAYPOINT] Geçici waypoint oluşturuldu (örn. default hedef tanımlandı).")
        if self.callback:
            self.callback("TEMP_WAYPOINT_CREATED")  # Geçici hedef oluşturma bildirimi
        return py_trees.common.Status.SUCCESS

