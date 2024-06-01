import itertools
import math
import warnings

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, Twist
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import numpy as np
import time
import cv2

from std_msgs.msg import String, Empty, Float32MultiArray
from models.mapconstants import ALGORITHM_RESOLUTION
from models.numpymap import NumpyMap, NumpyMapDisplayer
from models.exploration import ExplorationEvent, BfsToDestination, ExploreUnknownMaps
from models.mapconstants import *

class GridMapBuilder(Node):
    def __init__(self):
        super().__init__('grid_map_builder')
        self.subscription_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.listener_pose,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST
            ))
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/hokuyo',
            self.listener_scan,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST
            ))
        self.subscription_scan = self.create_subscription(
            Float32MultiArray,  # [x1,y1,x2,y2].
            '/goal_point/redo_bfs_if_blocked',
            self.redo_bfs_if_blocked,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST
            ))
        self.subscription_goal_point_reached = self.create_subscription(
            Empty,
            '/goal_point/reached',
            self.goal_point_is_reached,
            1)
        self.subscription_goal_point_blocked = self.create_subscription(
            Empty,
            '/goal_point/blocked',
            self.goal_point_is_blocked,
            1)
        self.subscription_goal_point_blocked = self.create_subscription(
            Empty,
            '/goal_point/redo_bfs',
            self.goal_point_redo_bfs,
            1)
        self.publisher_goal_point = self.create_publisher(
            Float32MultiArray,
            '/goal_point',
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST
            ))
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/robot_cmd_vel',
            self.robot_cmd_vel_update,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST
            ))
        self.publisher_map = self.create_publisher(
            OccupancyGrid,
            '/occupancy_map',
            10)
        self.publisher_map_robot = self.create_publisher(
            Pose2D,
            '/occupancy_map/robot_pose',
            10)

        self.map = NumpyMap()
        self._resized_map = None

        fig = plt.figure()
        ax1 = fig.add_subplot(211)
        size_divider = 3
        fig.set_figheight(20 / size_divider)
        fig.set_figwidth(15 / size_divider)
        ax2 = fig.add_subplot(212)
        self.displayer = NumpyMapDisplayer(self.map, fig, ax1)
        self.displayer.update_frame()
        self.displayer_abstract = NumpyMapDisplayer(self.map.resize_dilated_but_efficient(ALGORITHM_RESOLUTION), fig, ax2)
        self.pause_mapping = False
        self.start_time = None

        # Grid map parameters
        self.map_size_x = 10  # in meters
        self.map_size_y = 10  # in meters
        self.resolution = 0.01  # meters per cell
        self.grid_size_x = int(self.map_size_x / self.resolution)
        self.grid_size_y = int(self.map_size_y / self.resolution)
        self.grid_map = np.zeros((self.grid_size_x, self.grid_size_y), dtype=np.int8)
        # Current pose of the robot
        self.current_pose = None
        self.publish_goals_cooldown = 0
        self.publish_goals_queue = 0

        ### EXPLORATION
        self.bfs = ExplorationEvent(
            ExploreUnknownMaps(), lambda: self.set_bfs(ExplorationEvent(
                BfsToDestination((6.64, 2.8)), lambda: self.set_bfs(
                    ExplorationEvent(BfsToDestination((6.275, -2.225)), lambda: print("FINISHED!"))
            ))))

        self.is_processing = False

    def set_bfs(self, new_bfs):
        self.bfs = new_bfs

    def robot_cmd_vel_update(self, msg: Twist) -> None:
        rotation_speed = msg.angular.z
        if self.start_time is None:
            return
        if time.time() < self.start_time + 4:  # do not pause during the first 4 seconds
            return
        self.pause_mapping = abs(rotation_speed) > math.radians(35)  # max at 20 deg/sec


    def goal_point_is_reached(self, *msg):
        print(f"Reached destination, redo BFS. current: {self.bfs.overall_destinations()}")
        destinations = self.bfs.overall_destinations()
        if len(destinations) > 0:
            final_dest = destinations[-1]
            dx = self.current_pose.x - final_dest[0]
            dy = self.current_pose.y - final_dest[1]
            length = math.sqrt(dx * dx + dy * dy)
            if length > 0.5:
                warnings.warn("Cancelling... Was marked as Finished/Reached destination but its still far away from destination. Probably due to latency/delay")
                return
        self.broadcast_goal(self.map_for_bfs, 100)

    def goal_point_is_blocked(self, *msg):
        print(f"BLOCKED. REPLANNING PATH. current: {self.bfs.overall_destinations()}")
        # self.broadcast_goal(self._resized_map, 100)

    def goal_point_redo_bfs(self, *arg):
        print(f"Requested to redo BFS")
        # self.broadcast_goal(self._resized_map, 100)

    def redo_bfs_if_blocked(self, msg):
        x1, y1, x2, y2 = msg.data
        if self.bfs is not None:
            route = self.bfs.overall_destinations()

            # reset if waypoint stuck in wall(?)
            if len(route) > 0:
                curr_pos = self.map_for_bfs.actual_to_px((self.current_pose.x, self.current_pose.y))
                next_destination = self.bfs.overall_destinations()[0][2]
                line_to_next_dest = list(self.bresenham_line(curr_pos[0], curr_pos[1], next_destination[0], next_destination[1]))
                for route in itertools.chain(map(lambda x: x[2], route), line_to_next_dest):
                    px, py = route
                    if self.map_for_bfs.canvas[py][px] >= PATH_OBSTACLE_TRESHOLD:
                        self.bfs.try_set_map(self.map_for_bfs, (self.current_pose.x, self.current_pose.y), 100)
                        return


    @property
    def map_for_bfs(self):
        return self._resized_map

    def listener_pose(self, msg):
        self.current_pose = msg
        self.start_time = self.start_time if self.start_time is not None else time.time()
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
        # valid_distances = np.array(msg.ranges) < msg.range_max
        distances = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        x_coords = distances * np.cos(angles + self.current_pose.theta) + self.current_pose.x
        y_coords = distances * np.sin(angles + self.current_pose.theta) + self.current_pose.y
        curr_pos = self.current_pose.x, self.current_pose.y

        if not self.pause_mapping:
            for end_x, end_y, distance in zip(x_coords, y_coords, distances):
                self.map.add_raycast(curr_pos, (end_x, end_y), distance < msg.range_max)

        # update robot position on map - V
        self.map.robot_pos = self.current_pose

        # self._resized_map = self.map
        self._resized_map = self.map.resize_dilated_but_efficient(ALGORITHM_RESOLUTION)

        self.map_for_bfs.canvas = dilation(self.map_for_bfs.canvas)
        self.displayer_abstract.set_new_map(self.map_for_bfs).update_frame()
        self.displayer.update_frame()


        self.broadcast_goal(self._resized_map)
        # Publish the occupancy grid
        self.publish_grid_map()

    def broadcast_goal(self, resized_map, chance=0):

        if self.current_pose is None:
            return
        self.bfs.try_set_map(resized_map, (self.current_pose.x, self.current_pose.y), chance)
        self.displayer.set_destinations(self.bfs.overall_destinations())

        dirty_bit = self.bfs.tick_to_check_if_need_replan(resized_map, self.current_pose)

        if self.bfs.reset_dirty_bit() or dirty_bit:
            goal_points = self.bfs.overall_destinations()
            data = PointToRosSerializers().serialize(goal_points)
            self.publish_goals_queue = data
        if time.time() < self.publish_goals_cooldown:
            return
        if self.publish_goals_queue is not None:
            self.publisher_goal_point.publish(self.publish_goals_queue)
            self.publish_goals_queue = None
            print(f"Publishing. Current pose: {self._resized_map.actual_to_px((self.current_pose.x, self.current_pose.y))}")
            self.publish_goals_cooldown = time.time() + 2

    def bresenham_line(self, x0, y0, x1, y1):
        """Generate coordinates of the line from (x0, y0) to (x1, y1) using Bresenham's algorithm."""
        assert isinstance(x0, int)
        assert isinstance(y0, int)
        assert isinstance(x1, int)
        assert isinstance(y1, int)
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
        return  # TODO
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
    # kernel = np.ones((3, 3), np.uint8)
    # return cv2.dilate(img, kernel, iterations=1)


def main(args=None):
    rclpy.init(args=args)
    grid_map_builder = GridMapBuilder()
    rclpy.spin(grid_map_builder)
    grid_map_builder.destroy_node()
    rclpy.shutdown()



class PointToRosSerializers:
    def __init__(self):
        pass

    def serialize(self, data):
        ret = Float32MultiArray()
        ret.data = []
        for i in data:
            ret.data.append(i[0])
            ret.data.append(i[1])
        return ret



if __name__ == '__main__':
    main()
