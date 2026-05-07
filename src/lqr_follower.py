import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np
from scipy.linalg import solve_continuous_are


class LQRFollower(Node):
    def __init__(self):
        super().__init__('lqr_follower')

        self.path = []
        self.current_index = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.path_sub = self.create_subscription(Path, '/dp_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.Q = np.diag([10.0, 10.0, 5.0])
        self.R = np.diag([1.0, 1.0])

    def path_callback(self, msg):
        self.path = msg.poses
        if self.current_index >= len(self.path):
            self.current_index = 0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def lqr_gain(self, A, B, Q, R):
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        return K

    def control_loop(self):
        if not self.path or self.current_index >= len(self.path):
            return

        target_pose = self.path[self.current_index].pose.position
        tx = target_pose.x
        ty = target_pose.y

        dx = tx - self.x
        dy = ty - self.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.15:
            self.current_index += 1
            if self.current_index >= len(self.path):
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                self.get_logger().info("Goal reached.")
                return
            return

        theta_ref = math.atan2(dy, dx)
        e_theta = math.atan2(math.sin(theta_ref - self.yaw), math.cos(theta_ref - self.yaw))

        ex = dx
        ey = dy
        v_ref = 0.4

        A = np.array([
            [0.0, 0.0, -v_ref * math.sin(self.yaw)],
            [0.0, 0.0,  v_ref * math.cos(self.yaw)],
            [0.0, 0.0, 0.0]
        ])

        B = np.array([
            [math.cos(self.yaw), 0.0],
            [math.sin(self.yaw), 0.0],
            [0.0, 1.0]
        ])

        state_error = np.array([[ex], [ey], [e_theta]])
        K = self.lqr_gain(A, B, self.Q, self.R)
        u = -K @ state_error

        v = v_ref + float(u[0, 0])
        w = float(u[1, 0])

        v = max(min(v, 0.6), -0.6)
        w = max(min(w, 1.5), -1.5)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LQRFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

