import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import numpy as np
import time


class GridMapBuilder(Node):
    def __init__(self):
        super().__init__('grid_map_builder')
        self.subscription_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.listener_pose,
            10)
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/hokuyo',
            self.listener_scan,
            10)
        self.publisher_map = self.create_publisher(
            OccupancyGrid,
            '/occupancy_map',
            10)
        self.publisher_map_robot = self.create_publisher(
            Pose2D,
            '/occupancy_map/robot_pose',
            10)

        # Grid map parameters
        self.map_size_x = 10  # in meters
        self.map_size_y = 10  # in meters
        self.resolution = 0.01  # meters per cell
        self.grid_size_x = int(self.map_size_x / self.resolution)
        self.grid_size_y = int(self.map_size_y / self.resolution)
        self.grid_map = np.zeros((self.grid_size_x, self.grid_size_y), dtype=np.int8)

        # Current pose of the robot
        self.current_pose = Pose2D()

    def listener_pose(self, msg):
        self.current_pose = msg
        grid_x = int((self.current_pose.x / self.resolution) + (self.grid_size_x / 2))
        grid_y = int((self.current_pose.y / self.resolution) + (self.grid_size_y / 2))

        msg = Pose2D()
        msg.x = float(grid_x)
        msg.y = float(grid_y)
        msg.theta = self.current_pose.theta
        self.publisher_map_robot.publish(msg)

    def listener_scan(self, msg):
        # Calculate occupied points from laser scan
        valid_distances = np.array(msg.ranges) < msg.range_max
        distances = np.array(msg.ranges)[valid_distances]
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))[valid_distances]
        x_coords = distances * np.cos(angles + self.current_pose.theta) + self.current_pose.x
        y_coords = distances * np.sin(angles + self.current_pose.theta) + self.current_pose.y

        robot_grid_x = int((self.current_pose.x / self.resolution) + (self.grid_size_x / 2))
        robot_grid_y = int((self.current_pose.y / self.resolution) + (self.grid_size_y / 2))

        # Convert to grid coordinates
        grid_x = np.floor((x_coords / self.resolution) + (self.grid_size_x / 2)).astype(int)
        grid_y = np.floor((y_coords / self.resolution) + (self.grid_size_y / 2)).astype(int)

        for end_x, end_y in zip(grid_x, grid_y):
            # Use Bresenham's algorithm to update cells in line of sight
            for line_x, line_y in self.bresenham_line(robot_grid_x, robot_grid_y, end_x, end_y):
                # Check bounds and mark as free if within grid
                if (0 <= line_x < self.grid_map.shape[1]) and (0 <= line_y < self.grid_map.shape[0]):
                    self.grid_map[line_y, line_x] = 255

            # Mark the end point as occupied
            if (0 <= end_x < self.grid_map.shape[1]) and (0 <= end_y < self.grid_map.shape[0]):
                self.grid_map[end_y, end_x] = 100

        # Publish the occupancy grid
        self.publish_grid_map()

    def bresenham_line(self, x0, y0, x1, y1):
        """Generate coordinates of the line from (x0, y0) to (x1, y1) using Bresenham's algorithm."""
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def publish_grid_map(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"
        grid_msg.info = MapMetaData()
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_map.shape[1]
        grid_msg.info.height = self.grid_map.shape[0]
        grid_msg.info.origin = Pose()
        grid_msg.data = self.grid_map.ravel().tolist()
        self.publisher_map.publish(grid_msg)
        print('Occupancy map published')
        time.sleep(0.35)


def main(args=None):
    rclpy.init(args=args)
    grid_map_builder = GridMapBuilder()
    rclpy.spin(grid_map_builder)
    grid_map_builder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
