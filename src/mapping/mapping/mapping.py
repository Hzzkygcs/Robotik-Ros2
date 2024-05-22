import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import numpy as np
import time
import cv2

from std_msgs.msg import Empty

from models.mapconstants import ALGORITHM_RESOLUTION
from models.numpymap import NumpyMap, NumpyMapDisplayer
from models.exploration import ExplorationSteps, BfsToDestination, ExploreUnknownMaps

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
        self.subscription_goal_point_reached = self.create_subscription(
            Empty,
            '/goal_point/reached',
            self.goal_point_is_reached,
            10)
        self.subscription_goal_point_blocked = self.create_subscription(
            Empty,
            '/goal_point/blocked',
            self.goal_point_is_blocked,
            10)
        self.publisher_goal_point = self.create_publisher(
            Point,
            '/goal_point',
            10)
        self.publisher_map = self.create_publisher(
            OccupancyGrid,
            '/occupancy_map',
            10)
        self.publisher_map_robot = self.create_publisher(
            Pose2D,
            '/occupancy_map/robot_pose',
            10)

        self.map = NumpyMap()

        fig = plt.figure()
        ax1 = fig.add_subplot(211)
        ax2 = fig.add_subplot(212)
        self.displayer = NumpyMapDisplayer(self.map, fig, ax1)
        self.displayer_abstract = NumpyMapDisplayer(self.map.resize_dilated_but_efficient(ALGORITHM_RESOLUTION), fig, ax2)

        # Grid map parameters
        self.map_size_x = 10  # in meters
        self.map_size_y = 10  # in meters
        self.resolution = 0.01  # meters per cell
        self.grid_size_x = int(self.map_size_x / self.resolution)
        self.grid_size_y = int(self.map_size_y / self.resolution)
        self.grid_map = np.zeros((self.grid_size_x, self.grid_size_y), dtype=np.int8)
        # Current pose of the robot
        self.current_pose = None

        ### EXPLORATION
        self.bfs = ExplorationSteps(
            ExploreUnknownMaps(),
            BfsToDestination((6.275, -2.225)))

    def goal_point_is_reached(self, *msg):
        print(f"NEXT DESTINATION. current: {self.bfs.overall_destinations()}")
        result = self.bfs.move_on_to_next_destination()
        chance = 100 if result is None else 0
        self.broadcast_goal(self._resized_map, chance)

    def goal_point_is_blocked(self, *msg):
        print(f"BLCOKED. REPLANNING PATH. current: {self.bfs.overall_destinations()}")
        self.broadcast_goal(self._resized_map, 100)

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
        if self.current_pose is None:
            return

        # Calculate occupied points from laser scan
        valid_distances = np.array(msg.ranges) < msg.range_max
        distances = np.array(msg.ranges)[valid_distances]
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))[valid_distances]
        x_coords = distances * np.cos(angles + self.current_pose.theta) + self.current_pose.x
        y_coords = distances * np.sin(angles + self.current_pose.theta) + self.current_pose.y
        curr_pos = self.current_pose.x,self.current_pose.y

        for end_x, end_y in zip(x_coords, y_coords):
            self.map.add_raycast(curr_pos, (end_x, end_y))
        self.displayer.update_frame()
        self._resized_map = self.map.resize_dilated_but_efficient(ALGORITHM_RESOLUTION)
        self._resized_map.canvas = dilation(self._resized_map.canvas)
        self.displayer_abstract.set_new_map(self._resized_map).update_frame()
        self.broadcast_goal(self._resized_map)

        # Publish the occupancy grid
        self.publish_grid_map()

    def broadcast_goal(self, resized_map, chance=0):
        self.bfs.try_set_map(resized_map, (self.current_pose.x,self.current_pose.y), chance)
        x,y, _ = self.bfs.get_destination()

        _,_, final_dest = self.bfs.overall_destinations()[-1]
        if self._resized_map.canvas[final_dest[1]][final_dest[0]]:
            self.bfs.set_map(resized_map, (x,y))  # replan for new target
            print(f"Target block is no longer unknown. Replanning to be {self.bfs.overall_destinations()}")
        point = Point()
        point.x = x
        point.y = y
        self.publisher_goal_point.publish(point)
        print(f"Publishing {x},{y}   {self._resized_map.x_to_px(x)},{self._resized_map.y_to_px(y)}")



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
        return # TODO
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


def dilation(img):
    return img
    # kernel = np.ones((3,3),np.uint8)
    # return cv2.dilate(img,kernel,iterations = 1)



def main(args=None):
    rclpy.init(args=args)
    grid_map_builder = GridMapBuilder()
    rclpy.spin(grid_map_builder)
    grid_map_builder.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
