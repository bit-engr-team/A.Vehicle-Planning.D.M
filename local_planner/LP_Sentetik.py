import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
import numpy as np
import math
import struct
import cv2
from cv_bridge import CvBridge
import collections
import json

class OGMNode(Node):
    def __init__(self):
        super().__init__('live_ogm_node')
        self.scale = 111000  # 1 derece lat/lon kaç metre
        self.resolution = 0.25
        self.grid_size = int(self.resolution * 1000)  # 200x200 grid
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)
        self.latest_gnss = None
        self.prev_gnss = None
        self.latest_heading = 0.0  # Radyan

        self.grid_center_x = self.grid_size // 2
        self.grid_center_y = self.grid_size // 2

        self.lidar_buffer = collections.deque(maxlen=8)  # Son 10 lidar verisi için buffer
        self.last_lane_lines = None  # Son başarılı lane çizgileri

        self.create_subscription(NavSatFix, '/carla/gps', self.gnss_callback, 10)
        self.create_subscription(PointCloud2, '/carla/lidar', self.lidar_callback, 10)
        self.lane_sub = self.create_subscription(String, '/perception/lane', self.lane_callback, 10)
        
        self.publisher_ = self.create_publisher(OccupancyGrid, '/ogm_map', 10)
        self.image_pub = self.create_publisher(Image, '/ogm_image', 10)
        self.bridge = CvBridge()

        #self.get_logger().info("OGMNode initialized and subscriptions created.")

    def gnss_callback(self, msg):
        if self.latest_gnss is not None:
            # Hareket doğrultusu (heading) hesapla
            dlat = (msg.latitude - self.latest_gnss.latitude) * self.scale
            dlon = (msg.longitude - self.latest_gnss.longitude) * self.scale
            if abs(dlat) > 1e-6 or abs(dlon) > 1e-6:
                self.latest_heading = math.atan2(dlon, dlat)  # Yani kuzey=0, doğu=+90°
        self.prev_gnss = self.latest_gnss
        self.latest_gnss = msg
        #self.get_logger().info(f"GNSS received: lat={msg.latitude}, lon={msg.longitude}, heading={math.degrees(self.latest_heading):.2f} deg")

    def lidar_callback(self, msg):
        # Son lidar mesajlarını buffer'a ekle
        self.lidar_buffer.append(msg)

        # OGM'yi sıfırla
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Son 10 lidar mesajını işle
        for lidar_msg in self.lidar_buffer:
            points = self.pointcloud2_to_xyz(lidar_msg)
            for x, y, z in points:
                distance = math.sqrt(x**2 + y**2 + z**2)
                if abs(z) > 1.5 or distance < 1.0:
                    continue
                gx = int(x / self.resolution) + self.grid_center_x
                gy = int(y / self.resolution) + self.grid_center_y
                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                    self.grid[gx, gy] = 100  # Occupied

        #self.get_logger().info("Occupancy grid güncellendi (son 10 lidar ile, 1m filtreli).")
        self.publish_ogm_image()

    def pointcloud2_to_xyz(self, cloud_msg):
        # PointCloud2 verisini xyz listesine çevirir (sadece x, y, z)
        fmt = 'fff'  # x, y, z
        step = cloud_msg.point_step
        data = cloud_msg.data
        points = []
        for i in range(0, len(data), step):
            x, y, z = struct.unpack_from(fmt, data, i)
            points.append((x, y, z))
        return points

    def publish_ogm_map(self, grid):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ogm"
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = -self.grid_center_x * self.resolution
        msg.info.origin.position.y = -self.grid_center_y * self.resolution
        msg.info.origin.position.z = 0.0
        msg.data = grid.flatten().tolist()
        self.publisher_.publish(msg)

    def publish_ogm_image(self, grid=None, ogm_lines=None):
        if grid is None:
            grid = self.grid
        img = np.zeros((640, 640), dtype=np.uint8)
        grid_resized = cv2.resize(grid, (640, 640), interpolation=cv2.INTER_NEAREST)
        # Y eksenine göre simetri (ayna)
        grid_resized = cv2.flip(grid_resized, 1)
        img[grid_resized == 100] = 255
        img[grid_resized == 200] = 180
        img[grid_resized == 0] = 128
        img[grid_resized == -1] = 0

        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if ogm_lines is not None:
            scale_x = 640 / self.grid_size
            scale_y = 640 / self.grid_size
            for line in ogm_lines:
                for i in range(1, len(line)):
                    # Y eksenine göre simetri için x'i ters çeviriyoruz
                    pt1 = (640 - int(line[i-1][0] * scale_x), int(line[i-1][1] * scale_y))
                    pt2 = (640 - int(line[i][0] * scale_x), int(line[i][1] * scale_y))
                    cv2.line(img_color, pt1, pt2, (0, 0, 255), 2)

        ros_img = self.bridge.cv2_to_imgmsg(img_color, encoding="bgr8")
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "Environment_map"
        self.image_pub.publish(ros_img)

    def lane_callback(self, msg):
        try:
            lane_data = json.loads(msg.data)
            lane_centers = lane_data.get("lane_centerlines", [])
            if not lane_centers or not all(lane.get("points", []) for lane in lane_centers):
                # Eğer yeni lane center yoksa, eskiyi kullan
                if self.last_lane_lines is not None:
                    ogm_lines = self.last_lane_lines
                else:
                    print("Hiç lane center verisi yok, çizim yapılmadı.")
                    return
            else:
                scale = 1 / 75.0  # 1 pixel = 1/75 metre
                ogm_lines = []
                for lane in lane_centers:
                    points = lane.get("points", [])
                    ogm_line = []
                    for px, py in points:
                        mx = -px * scale
                        my = -py * scale
                        gx = int(mx / self.resolution) + self.grid_center_x
                        gy = int(my / self.resolution) + self.grid_center_y
                        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                            ogm_line.append((gx, gy))
                    ogm_lines.append(ogm_line)
                self.last_lane_lines = ogm_lines  # Son başarılı lane çizgilerini kaydet
                self.get_logger().info("Yeni şerit merkez bilgileri başarıyla alındı ve işlendi.")

            ogm_with_lanes = self.grid.copy()
            for line in ogm_lines:
                for i in range(1, len(line)):
                    cv2.line(ogm_with_lanes, line[i-1], line[i], 200, 1)
            self.publish_ogm_map(ogm_with_lanes)
            self.publish_ogm_image(ogm_with_lanes, ogm_lines)
        except Exception as e:
            print(f"Lane JSON parse error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OGMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()