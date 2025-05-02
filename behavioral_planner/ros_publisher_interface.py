#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String  # Karar mesajları basit string olarak yayınlanacak

class RosPublisherInterface:
    def __init__(self, node):
        self.node = node  # ROS2 node referansı alınır

        # Farklı karar çıktıları için ayrı topic'ler tanımlanır
        self.stop_pub = self.node.create_publisher(String, '/planner/stop', 10)
        self.slow_pub = self.node.create_publisher(String, '/planner/slow_down', 10)
        self.proceed_pub = self.node.create_publisher(String, '/planner/proceed', 10)
        self.speed_pub = self.node.create_publisher(String, '/planner/speed_adjust', 10)
        self.replan_pub = self.node.create_publisher(String, '/planner/replan', 10)
        self.waypoint_pub = self.node.create_publisher(String, '/planner/waypoint_update', 10)
        self.park_pub = self.node.create_publisher(String, '/planner/park', 10)

    def stop(self):
        """Aracı durdur komutu yayınlar."""
        msg = String()
        msg.data = "STOP"
        self.stop_pub.publish(msg)
        self.node.get_logger().info("[ROS PUB] STOP komutu yayınlandı.")

    def slow_down(self):
        """Yavaşla komutu yayınlar."""
        msg = String()
        msg.data = "SLOW_DOWN"
        self.slow_pub.publish(msg)
        self.node.get_logger().info("[ROS PUB] SLOW_DOWN komutu yayınlandı.")

    def proceed(self):
        """Devam et komutu yayınlar."""
        msg = String()
        msg.data = "PROCEED"
        self.proceed_pub.publish(msg)
        self.node.get_logger().info("[ROS PUB] PROCEED komutu yayınlandı.")

    def adjust_speed(self, value="AUTO"):
        """
        Hız ayarlama komutu yayınlar.
        value: 'AUTO' varsayılan hız önerisi ya da bir sayı olabilir.
        """
        msg = String()
        msg.data = f"SPEED_ADJUST:{value}"
        self.speed_pub.publish(msg)
        self.node.get_logger().info(f"[ROS PUB] SPEED_ADJUST komutu yayınlandı: {value}")

    def replan(self):
        """Yeniden rota belirleme komutu yayınlar."""
        msg = String()
        msg.data = "REPLAN"
        self.replan_pub.publish(msg)
        self.node.get_logger().info("[ROS PUB] REPLAN komutu yayınlandı.")

    def update_waypoint(self):
        """Bir sonraki hedef waypoint'e geçiş komutu yayınlar."""
        msg = String()
        msg.data = "NEXT_WAYPOINT"
        self.waypoint_pub.publish(msg)
        self.node.get_logger().info("[ROS PUB] NEXT_WAYPOINT komutu yayınlandı.")

    def park(self):
        """Park et komutu yayınlar."""
        msg = String()
        msg.data = "PARK"
        self.park_pub.publish(msg)
        self.node.get_logger().info("[ROS PUB] PARK komutu yayınlandı.")
