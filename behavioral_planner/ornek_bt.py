import py_trees  

class TrafikIsigiKarari(py_trees.behaviour.Behaviour):
    def __init__(self, name="Trafik Işığı Kararı"):
        super().__init__(name)  
        self.isik = "KIRMIZI"  

    def update(self):
        if self.isik == "KIRMIZI":  
            self.logger.info("KIRMIZI IŞIK → DUR")  
            return py_trees.common.Status.FAILURE  
        elif self.isik == "SARI":  
            self.logger.info("SARI IŞIK → YAVAŞLA")  
            return py_trees.common.Status.RUNNING  
        elif self.isik == "YEŞİL":  
            self.logger.info("YEŞİL IŞIK → DEVAM ET")  
            return py_trees.common.Status.SUCCESS  
        else:  
            self.logger.info("Işık Bilgisi Eksik → BEKLE")  
            return py_trees.common.Status.RUNNING  


class EngelKontrol(py_trees.behaviour.Behaviour):
    def __init__(self, name="Engel Kontrolü"):
        super().__init__(name)  
        self.engel_var = False  

    def update(self):
        if self.engel_var:  
            self.logger.info("ENGEL ALGILANDI → DUR")  
            return py_trees.common.Status.FAILURE  
        else:  # Engel yoksa
            self.logger.info("ENGEL YOK → DEVAM ET")  
            return py_trees.common.Status.SUCCESS  


def bt_olustur(isik_durumu, engel_durumu):
    # Paralel düğüm oluşturulur, tüm çocukların statusuna göre karar verir
    root = py_trees.composites.Parallel(
        name="Ana Paralel Kontrol",  
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)  # Her tickte child nodelar durumunu korusun sıfırlanmasın diye False
    )

    
    trafik_dugumu = TrafikIsigiKarari()
    trafik_dugumu.isik = isik_durumu  

    
    engel_dugumu = EngelKontrol()
    engel_dugumu.engel_var = engel_durumu  

    
    root.add_children([trafik_dugumu, engel_dugumu]) 

    return root  


if __name__ == "__main__": # BT çalıştırılır
    
    test_senaryolari = [
        ("KIRMIZI", True),
        ("YEŞİL", False),
        ("SARI", True),
        ("YEŞİL", True)
    ]

    for isik, engel in test_senaryolari:  
        print(f"Test Senaryosu → Işık: {isik}, Engel: {engel}")  
        bt = bt_olustur(isik, engel)  
        print(py_trees.display.ascii_tree(bt))  # BT ağacının metinsel görünümü yazdırılır
        bt.tick_once()  # BT bir adım çalıştırılır
        status = bt.status  # BT'nin genel karar sonucu alınır
        print(f"Karar Sonucu: {status.name}\n")  