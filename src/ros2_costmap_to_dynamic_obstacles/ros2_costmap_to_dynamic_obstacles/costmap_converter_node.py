import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
from unique_identifier_msgs.msg import UUID
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray

class CostmapConverterNode(Node):
    def __init__(self):
        super().__init__('costmap_converter_node')

        # Subscribe đến costmap
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10
        )

        # Publisher để gửi danh sách vật thể động
        self.publisher = self.create_publisher(ObstacleArray, '/detection', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/detection_markers', 10)

        self.get_logger().info("Costmap Converter Node Initialized")

    def costmap_callback(self, msg):
        """ Xử lý dữ liệu costmap và phát hiện vật thể động """
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        # Chuyển đổi costmap thành ảnh OpenCV
        costmap_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        costmap_image = np.where(costmap_data == -1, 255, costmap_data).astype(np.uint8)

        # Phát hiện vật thể động
        obstacles = self.detect_dynamic_obstacles(costmap_image, resolution, origin)

        # Tạo ObstacleArray message
        obstacle_array_msg = ObstacleArray()
        obstacle_array_msg.header = msg.header
        obstacle_array_msg.obstacles = obstacles

        self.publisher.publish(obstacle_array_msg)
        self.publish_as_marker(msg.header.frame_id, obstacles)

    def detect_dynamic_obstacles(self, image, resolution, origin):
        # Phát hiện vật thể động trong costmap sử dụng OpenCV
        processed_image = cv2.GaussianBlur(image, (5, 5), 0)
        fg_mask = self.background_subtraction(processed_image)

        # Đảm bảo ảnh fg_mask là dạng uint8
        fg_mask = fg_mask.astype(np.uint8)

        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obstacles = []

        for contour in contours:
            if cv2.contourArea(contour) > 10:  # Ngưỡng diện tích
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Chuyển đổi pixel sang tọa độ thực tế
                    position = Point()
                    position.x = origin.x + cx * resolution
                    position.y = origin.y + cy * resolution
                    position.z = 0.0  # LiDAR chỉ có dữ liệu 2D

                    # Tạo UUID cho vật thể (danh tính duy nhất)
                    obstacle_id = UUID()
                    obstacle_id.uuid = [0] * 16  # Tạm thời đặt UUID là 16 số 0

                    # Khởi tạo velocity và size
                    velocity = Vector3()
                    size = Vector3()
                    size.x = size.y = 0.5  # Giả định kích thước vật thể

                    # Tạo Obstacle message
                    obstacle = Obstacle()
                    obstacle.header = Header()
                    obstacle.id = obstacle_id
                    obstacle.position = position
                    obstacle.velocity = velocity
                    obstacle.size = size

                    obstacles.append(obstacle)

        return obstacles

    def background_subtraction(self, image):
        # Phát hiện vật thể động bằng tách nền
        if not hasattr(self, 'previous_image'):
            self.previous_image = image.copy()
            return np.zeros_like(image, dtype=np.uint8)

        diff = cv2.absdiff(self.previous_image, image)
        _, fg_mask = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

        self.previous_image = image.copy()
        return fg_mask

    def publish_as_marker(self, frame_id, obstacles):
        marker_array = MarkerArray()
        
        # Xóa toàn bộ Marker cũ để tránh hiện tượng "bóng ma"
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "Obstacles Markers"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = obstacle.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(0.3, obstacle.size.x)  # Đảm bảo Marker không quá nhỏ
            marker.scale.y = max(0.3, obstacle.size.y)
            marker.scale.z = 0.2  # Độ cao Marker để dễ quan sát
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8  # Độ trong suốt phù hợp
            marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
