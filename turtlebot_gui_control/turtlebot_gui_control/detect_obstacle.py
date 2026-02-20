import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from turtlebot_gui_interfaces.msg import ObstacleDistance

class DetectObstacle(Node):
    def __init__(self):
        super().__init__('detect_obstacle')
        self.qos_profile = QoSProfile(depth=10)
        self.obstacle_pub = self.create_publisher(
            ObstacleDistance,
            '/obstacle_distance',
            self.qos_profile
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.scan_ranges = []

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        valid_ranges = [r for r in self.scan_ranges if math.isfinite(r) and r > 0.0]
        if not valid_ranges:
            return

        # 전방 섹터만 계산 (±20도, 인덱스 0 기준 ±20)
        # TurtleBot3: 360개 샘플, 인덱스 0 = 정면
        total_samples = len(self.scan_ranges)
        front_angle_range = 20  # degrees
        samples_per_degree = total_samples / 360
        front_samples = int(front_angle_range * samples_per_degree)

        # 전방: [0:front_samples] + [total_samples-front_samples:total_samples]
        front_indices = list(range(0, front_samples)) + list(range(total_samples - front_samples, total_samples))
        front_ranges = [self.scan_ranges[i] for i in front_indices if i < len(self.scan_ranges)]
        valid_front_ranges = [r for r in front_ranges if math.isfinite(r) and r > 0.0]

        if not valid_front_ranges:
            return

        min_range = min(valid_front_ranges)
        self.obstacle_pub.publish(ObstacleDistance(distance=min_range))


def main(args=None):
    rclpy.init(args=args)
    node = DetectObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt!!!!')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
  main()
