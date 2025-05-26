#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from behavioral_planner.tree_handler import TreeHandler #behavioral_planner modulu yok diyo, bak
from behavioral_planner.rdf_interface import RDFInterface
from behavioral_planner.ros_publisher_interface import RosPublisherInterface  

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')  # ROS2 node ismi

        self.rdf_interface = RDFInterface(self)  # RDF verisi buradan alınacak
        self.ros_publisher_interface = None  # Şimdilik yoksa None olarak bırakılır

        self.tree_handler = TreeHandler(
            rdf_interface=self.rdf_interface,
            ros_publisher_interface=self.ros_publisher_interface
        )

        timer_period = 0.1  # Her 100ms'de bir davranış ağacı tick edilecek
        self.timer = self.create_timer(timer_period, self.tick_behavior_tree)

    def tick_behavior_tree(self):
        self.tree_handler.tick_tree()  # TreeHandler içindeki ağacı bir adım çalıştır

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)  # ROS node’u çalıştır
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
