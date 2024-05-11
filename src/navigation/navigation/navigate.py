import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, Twist
# from navigation.srv import SetGoal
from sensor_msgs.msg import LaserScan
import time

import numpy as np

from navigation.models.raycasthit import RayCastHit


class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/hokuyo',
            self.on_receive_laser_scan,
            10)
        self.subscriber_robot_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        self.subscriber_goal_point = self.create_subscription(
            Point,
            '/goal_point',
            self.goal_point_callback,
            10)
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/robot_cmd_vel',
            10)
        self.timer = self.create_timer(0.1, self.navigate)

        self.raycast = LaserScan()
        self.raycast_received_timestamp = 0
        self.raycast_expire_duration = 1000
        self._raycast_results = []

        self.robot_pose = Pose2D()
        self._goal_point = Point()
        self.robot_pose_received = False
        self.goal_point_received = False
        self.get_logger().info('Running')

    def on_receive_laser_scan(self, message: LaserScan):
        ret = []
        for index, range in enumerate(self.raycast.ranges):
            if range < self.raycast.range_min or range > self.raycast.range_max:
                continue  # simulate real world range limitation
            angle = message.angle_min + message.angle_increment*index
            ret.append(RayCastHit(range, angle))
        ret.sort(key=lambda x: x.range, reverse=True)
        self.raycast = message
        self.raycast_received_timestamp = int(time.time())
        self._raycast_results = ret
        self.goal_point_received_timestamp = 0


    @property
    def raycast_results(self) -> list[RayCastHit]:
        if int(time.time()) - self.raycast_received_timestamp > self.raycast_expire_duration:
            return []
        return self._raycast_results


    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True
    
    def goal_point_callback(self, msg: Point):
        self._goal_point = msg
        self.goal_point_received = True
        self.goal_point_received_timestamp = int(time.time())

    @property
    def goal_point(self) -> Point:
        dx = self._goal_point.x - self.robot_pose.x
        dy = self._goal_point.y - self.robot_pose.y
        distance = np.sqrt(dx**2 + dy**2)
        if distance < 0.1:
            return None
        return self._goal_point


    def navigate(self):
        distance = float('inf')
        if self.goal_point is None:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.publisher_cmd_vel.publish(cmd_vel)
            return True

        dx = self.goal_point.x - self.robot_pose.x
        dy = self.goal_point.y - self.robot_pose.y
        distance = np.sqrt(dx**2 + dy**2)
        goal_angle = np.arctan2(dy, dx)
        theta = goal_angle - self.robot_pose.theta

        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi

        cmd_vel = Twist()
        cmd_vel.linear.y = np.min([0.2 * distance, 0.2])
        cmd_vel.angular.z = 2.0 * theta
        self.publisher_cmd_vel.publish(cmd_vel)
        return True

    def obstacle_is_near(self):
        robot_body_radius = 0.3
        for obstacle in self.raycast_results:
            pass







def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()