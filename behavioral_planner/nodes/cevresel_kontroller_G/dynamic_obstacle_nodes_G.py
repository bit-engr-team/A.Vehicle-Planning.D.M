import py_trees
from behavioral_planner.rdf_interface_G import RDFInterface_G

class IsObstacleAhead_G(py_trees.behaviour.Behaviour):
    def __init__(self, rdf_interface, name="Engel var mı?"):
        super().__init__(name)  # py_trees sınıfına node ismini vererek üst sınıfı başlatır
        self.rdf = rdf_interface  # RDF verisini sorgulamak için gerekli arayüz

    def update(self):
        # RDF içinde herhangi bir engelin algılanıp algılanmadığını kontrol eden SPARQL sorgusu
        query = """
        ASK WHERE {
            ?o a :Obstacle ; :isDetected true .
        }
        """
        result = self.rdf.ask_query(query)  # SPARQL sorgusunu çalıştır ve sonucu al

        if result:
            print("[ENGEL] Engel algılandı.")  # Engel varsa çıktı ver
            return py_trees.common.Status.SUCCESS  # Engel varsa başarılı dön (ağaç devam eder)
        else:
            print("[ENGEL] Engel algılanmadı.")  # Engel yoksa çıktı ver
            return py_trees.common.Status.FAILURE  # Engel yoksa başarısız dön (ağaç bu dalı terk eder)


class IsObstacleClose_G(py_trees.behaviour.Behaviour):
    def __init__(self, rdf_interface, name="Engel yakın mı?", threshold_distance=5.0):
        super().__init__(name)  # py_trees sınıfına node ismini vererek üst sınıfı başlatır
        self.rdf = rdf_interface  # RDF verisini sorgulamak için gerekli arayüz
        self.threshold = threshold_distance  # Engel ne kadar yakında olursa "yakın" sayılacağı mesafe eşiği (biz aslinda mesafe degil ALAN KULLANICAZ, ALAN NE KADAR BUYUKSE O KADAR)

    def update(self):
        # Engel var mı ve mesafesi nedir diye SPARQL sorgusu oluşturuluyor
        query = """
        SELECT ?d WHERE {
            ?o a :Obstacle ; :distance ?d ; :isDetected true .
        } LIMIT 1
        """
        results = self.rdf.select_query(query)  # SPARQL sorgusunu çalıştır ve sonucu al

        for row in results:
            try:
                distance = float(row.d.toPython())  # RDF içindeki mesafeyi float türüne çevir
                if distance <= self.threshold:
                    print(f"[ENGEL] Engel mesafesi {distance} m, DUR.")  # Engel çok yakınsa uyarı ver
                    return py_trees.common.Status.SUCCESS  # Engel yakın: davranış başarılı (durulmalı)
                else:
                    print(f"[ENGEL] Engel mesafesi {distance} m, YAVAŞLA.")  # Engel var ama yakın değilse bu mesaj gösterilir.
                    # Bu durumda node FAILURE döndüreceği için fallback yolundaki SlowDown_G node'u otomatik çalışacaktır.
                    # Yani burada mesaj vermek zorunlu değildir ama anlaşılabilirlik ve debug için faydalıdır.  
                    return py_trees.common.Status.FAILURE  # Engel uzak: durma davranışı uygulanmaz
            except:
                pass  # Eğer dönüşümde hata olursa (örneğin boş veri) hiçbir şey yapma

        print("[ENGEL] Engel mesafesi alınamadı.")  # Sorgudan hiç veri gelmezse uyarı ver
        return py_trees.common.Status.FAILURE  # Varsayılan olarak başarısız dön (engel yakın mı bilinmiyor)


class StopForObstacle_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Dur (Engel)", decision_callback=None):
        super().__init__(name)
        self.callback = decision_callback  # Local planner'a karar iletmek için callback

    def update(self):
        print("[AKSİYON] Araç durduruluyor...")
        if self.callback:
            self.callback("STOP")  # Local planner'a STOP komutu gönder
        return py_trees.common.Status.SUCCESS


class SlowDownForObstacle_G(py_trees.behaviour.Behaviour):
    def __init__(self, name="Yavaşla", decision_callback=None):
        super().__init__(name)
        self.callback = decision_callback  # Local planner'a karar iletmek için callback

    def update(self):
        print("[AKSİYON] Araç yavaşlatılıyor...")
        if self.callback:
            self.callback("SLOWDOWN")  # Local planner'a SLOWDOWN komutu gönder
        return py_trees.common.Status.SUCCESS
