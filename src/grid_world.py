import rclpy
from rclpy.node import Node


class GridWorldNode(Node):
    def __init__(self):
        super().__init__('grid_world_node')

        # Grid size
        self.rows = 10
        self.cols = 10

        # 0 = free cell, 1 = obstacle
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

        # Start and goal
        self.start = (0, 0)
        self.goal = (9, 9)

        # Obstacles
        self.obstacles = [
            (2, 2), (2, 3), (2, 4),
            (3, 4),
            (4, 4),
            (5, 4), (6, 4),
            (7, 4),
            (7, 5), (7, 6), (7, 7)
        ]

        self.add_obstacles()
        self.print_grid()

    def add_obstacles(self):
        for r, c in self.obstacles:
            if 0 <= r < self.rows and 0 <= c < self.cols:
                self.grid[r][c] = 1

    def print_grid(self):
        self.get_logger().info("Grid World:")
        for r in range(self.rows):
            row_str = ""
            for c in range(self.cols):
                if (r, c) == self.start:
                    row_str += " S "
                elif (r, c) == self.goal:
                    row_str += " G "
                elif self.grid[r][c] == 1:
                    row_str += " X "
                else:
                    row_str += " . "
            self.get_logger().info(row_str)


def main(args=None):
    rclpy.init(args=args)
    node = GridWorldNode()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
