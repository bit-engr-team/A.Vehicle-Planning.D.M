import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Image
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge

DATA_FILE = "gnss_data.csv"
WAYPOINT_FILE = "waypoints.csv"
IMG_TOPIC = "/mapping/gps_map_image"

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.get_logger().info('Mapping node başlatıldı')
        self.gps_sub = self.create_subscription(NavSatFix, '/carla/gps', self.gps_callback, 10)
        self.img_pub = self.create_publisher(Image, IMG_TOPIC, 1)
        self.bridge = CvBridge()
        self.gps_points = []
        self.load_data()

    def load_data(self):
        if os.path.exists(DATA_FILE):
            with open(DATA_FILE, "r") as f:
                reader = csv.reader(f)
                for row in reader:
                    lat, lon, alt, stamp = map(float, row)
                    self.gps_points.append((lat, lon, alt, stamp))
            self.get_logger().info(f"{len(self.gps_points)} eski GNSS noktası yüklendi.")
    
    def save_point(self, lat, lon, alt, stamp):
        with open(DATA_FILE, "a") as f:
            writer = csv.writer(f)
            writer.writerow([lat, lon, alt, stamp])

    def gps_callback(self, msg):
        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gps_points.append((lat, lon, alt, stamp))
        self.save_point(lat, lon, alt, stamp)
        self.get_logger().info(f"GNSS kaydedildi: {lat:.6f}, {lon:.6f}, {alt:.2f}, {stamp:.2f}")
        self.publish_map_image()

    def compute_angle(self, p1, p2, p3):
        v1 = np.array([p1[0] - p2[0], p1[1] - p2[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
            return 180
        unit_v1 = v1 / np.linalg.norm(v1)
        unit_v2 = v2 / np.linalg.norm(v2)
        dot_product = np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)
        angle = np.degrees(np.arccos(dot_product))
        return angle

    def extract_waypoints(self, points, angle_threshold=30):
        sorted_points = sorted(points, key=lambda p: p[3])
        waypoints = []
        for i in range(5, len(sorted_points) - 5):
            p1, p2, p3 = sorted_points[i-5], sorted_points[i], sorted_points[i+5]
            angle = self.compute_angle(p1, p2, p3)
            if angle < (180 - angle_threshold):
                waypoints.append(p2)
        return waypoints

    def publish_map_image(self):
        if len(self.gps_points) < 3:
            return

        sorted_points = sorted(self.gps_points, key=lambda x: x[3])
        filtered_points = []
        last_lat, last_lon = None, None

        for lat, lon, alt, stamp in sorted_points:
            if last_lat is not None:
                dist = np.sqrt((lat - last_lat)**2 + (lon - last_lon)**2)
                if dist < 1e-6:
                    continue
            filtered_points.append((lat, lon, alt, stamp))
            last_lat, last_lon = lat, lon

        if len(filtered_points) < 3:
            return

        # Waypointleri çıkar
        waypoints = self.extract_waypoints(filtered_points)

        # Harita çizimi
        lats = [p[0] for p in filtered_points]
        lons = [p[1] for p in filtered_points]

        plt.figure(figsize=(6, 6))
        plt.plot(lons, lats, 'bo-', markersize=2, label='GNSS Yol')
        for wp in waypoints:
            plt.plot(wp[1], wp[0], 'ro', markersize=10)  # lon, lat
        plt.legend()
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title("GNSS Yol ve Waypoint Haritası")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("map.png")
        plt.close()

        # Görseli yayınla
        img = cv2.imread("map.png")
        if img is not None:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.img_pub.publish(img_msg)

        # Waypoint'leri kaydet
        with open(WAYPOINT_FILE, "w") as f:
            writer = csv.writer(f)
            for wp in waypoints:
                writer.writerow([wp[0], wp[1], wp[2], wp[3]])


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
