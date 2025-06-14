import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Image
import csv
import os
import matplotlib.pyplot as plt
import numpy as np
import cv2
from cv_bridge import CvBridge
from sklearn.cluster import DBSCAN

DATA_FILE = "gnss_data.csv"
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
            self.get_logger().info(f"{len(self.gps_points)} GNSS noktası yüklendi.")

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
        v1 = np.array([p2[1] - p1[1], p2[0] - p1[0]])
        v2 = np.array([p3[1] - p2[1], p3[0] - p2[0]])
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        return np.degrees(angle)

    def extract_waypoints(self, points, angle_threshold=30):
        sorted_points = sorted(points, key=lambda p: p[3])
        candidates = []
        for i in range(1, len(sorted_points) - 1):
            p1, p2, p3 = sorted_points[i - 1], sorted_points[i], sorted_points[i + 1]
            angle = self.compute_angle(p1, p2, p3)
            if angle < (180 - angle_threshold):
                candidates.append(p2)
        return candidates

    def cluster_waypoints(self, waypoints, eps=0.0001, min_samples=2):
        if not waypoints:
            return []
        coords = np.array([[wp[0], wp[1]] for wp in waypoints])
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(coords)
        labels = clustering.labels_

        clustered = []
        for label in set(labels):
            if label == -1:
                continue
            group = coords[labels == label]
            lat_avg = np.mean(group[:, 0])
            lon_avg = np.mean(group[:, 1])
            clustered.append((lat_avg, lon_avg))
        return clustered

    def publish_map_image(self):
        if len(self.gps_points) < 3:
            return

        lats = [p[0] for p in self.gps_points]
        lons = [p[1] for p in self.gps_points]

        waypoints = self.extract_waypoints(self.gps_points)
        clustered_waypoints = self.cluster_waypoints(waypoints)

        plt.figure(figsize=(6, 6))
        plt.plot(lons, lats, 'b-', linewidth=1, label="GNSS Yol")
        plt.scatter([w[1] for w in clustered_waypoints],
                    [w[0] for w in clustered_waypoints],
                    color='red', s=60, label="Waypoint")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title("GNSS Yol Haritası + Waypointler")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("map.png")
        plt.close()

        img = cv2.imread("map.png")
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.img_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    mapping_node = MappingNode()
    rclpy.spin(mapping_node)
    mapping_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
