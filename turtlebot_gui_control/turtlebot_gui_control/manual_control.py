import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from turtlebot_gui_interfaces.msg import ObstacleDistance

class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        self.qos_profile = QoSProfile(depth=10)
        self.manual_publisher = self.create_publisher(Twist, '/cmd_vel', self.qos_profile)
        self.obstacle_sub = self.create_subscription(
            ObstacleDistance,
            '/obstacle_distance',
            self.obstacle_callback,
            self.qos_profile
        )
        self.publish_timer = self.create_timer(0.5, self._publish_cmd)
        self.velocity = 0.0
        self.angular = 0.0
        self.stop_distance = 0.6
        self._obstacle_near = False

    def move_forward(self):
        if self._obstacle_near:
            return
        self.velocity += 0.1
    def move_backward(self):
        self.velocity += -0.1
    def turn_left(self):
        self.angular += 0.1
    def turn_right(self):
        self.angular += -0.1
    def stop(self):
        self.velocity = 0.0
        self.angular = 0.0


    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular
        self.manual_publisher.publish(msg)
        self.get_logger().info(
            f'Manual Published message: velocity={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )

    def obstacle_callback(self, msg):
        is_near = msg.distance < self.stop_distance
        self._obstacle_near = is_near
        if is_near:
            # 전방 장애물 가까우면 linear velocity만 차단, angular 유지 (회전 허용)
            self.velocity = 0.0
            # self.angular는 0으로 설정 안 함 - 제자리 회전 가능
            self._publish_cmd()

def main(args=None):
    rclpy.init(args=args)
    manual_control_node = ManualControl()
    try:
      while rclpy.ok():
        rclpy.spin_once(manual_control_node)
    except KeyboardInterrupt:
        manual_control_node.get_logger().info('Keyboard interrupt!!!!')
    finally:
        manual_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
