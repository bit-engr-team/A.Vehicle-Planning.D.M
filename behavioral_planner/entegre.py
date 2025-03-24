import rclpy  
from rclpy.node import Node
from std_msgs.msg import String, Bool  
import py_trees  

# bu kod ne anlamadım

trafik_isigi_verisi = "YEŞİL"  
engel_var_mi = False  


class TrafikIsigiKarari(py_trees.behaviour.Behaviour):
    def __init__(self, name="Trafik Işığı Kararı"):
        super().__init__(name)

    def update(self):
        if trafik_isigi_verisi == "KIRMIZI":
            self.logger.info("KIRMIZI IŞIK → DUR")
            return py_trees.common.Status.FAILURE
        elif trafik_isigi_verisi == "SARI":
            self.logger.info("SARI IŞIK → YAVAŞLA")
            return py_trees.common.Status.RUNNING
        elif trafik_isigi_verisi == "YEŞİL":
            self.logger.info("YEŞİL IŞIK → DEVAM ET")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("Işık Bilgisi Eksik → BEKLE")
            return py_trees.common.Status.RUNNING


class EngelKontrol(py_trees.behaviour.Behaviour):
    def __init__(self, name="Engel Kontrolü"):
        super().__init__(name)

    def update(self):
        if engel_var_mi:
            self.logger.info("ENGEL ALGILANDI → DUR")
            return py_trees.common.Status.FAILURE
        else:
            self.logger.info("ENGEL YOK → DEVAM ET")
            return py_trees.common.Status.SUCCESS


def bt_olustur():
    root = py_trees.composites.Parallel(
        name="Paralel Kontrol",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    trafik_dugumu = TrafikIsigiKarari()
    engel_dugumu = EngelKontrol()
    root.add_children([trafik_dugumu, engel_dugumu])
    return root

# ROS 2 Node sınıfı: abone olarak veri alır ve BT çalıştırır
class BTAnaNode(Node):
    def __init__(self):
        super().__init__('bt_ana_node')
        self.create_subscription(String, 'trafik_isigi', self.trafik_callback, 10)
        self.create_subscription(Bool, 'engel_durumu', self.engel_callback, 10)
        self.bt = bt_olustur()  # BT oluşturulur
        self.timer = self.create_timer(1.0, self.bt_tick_et)  # Her 1 sn'de BT tick

    def trafik_callback(self, msg):
        global trafik_isigi_verisi
        trafik_isigi_verisi = msg.data  

    def engel_callback(self, msg):
        global engel_var_mi
        engel_var_mi = msg.data  

    def bt_tick_et(self):
        self.bt.tick_once()  
        print(py_trees.display.ascii_tree(self.bt))  
        print(f"BT Karar: {self.bt.status.name}\n")  

# Main fonksiyonu: ROS node başlatılır
def main(args=None):
    rclpy.init(args=args)
    node = BTAnaNode()
    try:
        rclpy.spin(node)  # Node sürekli çalışır
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
