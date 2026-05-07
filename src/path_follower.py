import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.path = []
        self.current_index = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.path_sub = self.create_subscription(Path, '/dp_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_index = 0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if not self.path or self.current_index >= len(self.path):
            return

        target = self.path[self.current_index].pose.position
        dx = target.x - self.x
        dy = target.y - self.y

        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.yaw

        # Normalize angle
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()

        if distance > 0.1:
            cmd.linear.x = 0.5
            cmd.angular.z = 1.0 * angle_error
        else:
            self.current_index += 1

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
