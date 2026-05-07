import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker


class DPPlannerNode(Node):
    def __init__(self):
        super().__init__('dp_planner_node')

        self.rows = 10
        self.cols = 10
        self.cell_size = 1.0

        # Grid initialization
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

        self.start = (0, 0)
        self.goal = (9, 9)

        # 🔥 Obstacles arranged to force diagonal path
        self.obstacles = [
            # Upper blocking
            (0, 2), (0, 3), (0, 4), (0, 5),
            (1, 3), (1, 4), (1, 5), (1, 6),
            (2, 4), (2, 5), (2, 6), (2, 7),
            (3, 5), (3, 6), (3, 7), (3, 8),

            # Lower blocking
            (2, 0), (3, 0), (4, 0),
            (3, 1), (4, 1), (5, 1),
            (4, 2), (5, 2), (6, 2),
            (5, 3), (6, 3), (7, 3)
        ]

        # ✅ 8-direction movement (diagonal enabled)
        self.moves = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        # Publishers
        self.path_pub = self.create_publisher(Path, '/dp_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/path_marker', 10)

        # Setup
        self.add_obstacles()
        self.compute_cost()
        self.path = self.extract_path()

        self.print_cost_map()
        self.print_path_grid()

        self.timer = self.create_timer(1.0, self.publish_all)

    # -----------------------------
    def add_obstacles(self):
        for r, c in self.obstacles:
            if 0 <= r < self.rows and 0 <= c < self.cols:
                self.grid[r][c] = 1

    # -----------------------------
    def is_valid(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r][c] == 0

    # -----------------------------
    def compute_cost(self):
        self.cost = [[math.inf]*self.cols for _ in range(self.rows)]
        self.cost[self.goal[0]][self.goal[1]] = 0

        changed = True
        while changed:
            changed = False
            for r in range(self.rows):
                for c in range(self.cols):
                    if self.grid[r][c] == 1:
                        continue

                    for dr, dc in self.moves:
                        nr, nc = r + dr, c + dc
                        if self.is_valid(nr, nc):
                            new_cost = 1 + self.cost[nr][nc]
                            if new_cost < self.cost[r][c]:
                                self.cost[r][c] = new_cost
                                changed = True

    # -----------------------------
    def extract_path(self):
        path = []
        current = self.start

        if self.cost[current[0]][current[1]] == math.inf:
            self.get_logger().info("No path found!")
            return path

        path.append(current)

        while current != self.goal:
            r, c = current
            best = current
            best_cost = self.cost[r][c]

            for dr, dc in self.moves:
                nr, nc = r + dr, c + dc
                if self.is_valid(nr, nc):
                    if self.cost[nr][nc] < best_cost:
                        best = (nr, nc)
                        best_cost = self.cost[nr][nc]

            current = best
            path.append(current)

        return path

    # -----------------------------
    def publish_path(self):
        msg = Path()
        msg.header.frame_id = "map"

        for r, c in self.path:
            pose = PoseStamped()
            pose.pose.position.x = float(c)
            pose.pose.position.y = float(r)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    # -----------------------------
    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.08

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        for r, c in self.path:
            p = Point()
            p.x = float(c)
            p.y = float(r)
            p.z = 0.05
            marker.points.append(p)

        self.marker_pub.publish(marker)

    # -----------------------------
    def publish_all(self):
        self.publish_path()
        self.publish_marker()
        self.get_logger().info(f"Published path with {len(self.path)} points")

    # -----------------------------
    def print_cost_map(self):
        self.get_logger().info("Cost Map:")
        for r in range(self.rows):
            row = ""
            for c in range(self.cols):
                if self.grid[r][c] == 1:
                    row += " XX "
                else:
                    row += f"{int(self.cost[r][c]):2d} "
            self.get_logger().info(row)

    # -----------------------------
    def print_path_grid(self):
        self.get_logger().info("DP Path:")
        path_set = set(self.path)

        for r in range(self.rows):
            row = ""
            for c in range(self.cols):
                if (r, c) == self.start:
                    row += " S "
                elif (r, c) == self.goal:
                    row += " G "
                elif self.grid[r][c] == 1:
                    row += " X "
                elif (r, c) in path_set:
                    row += " * "
                else:
                    row += " . "
            self.get_logger().info(row)


# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = DPPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
