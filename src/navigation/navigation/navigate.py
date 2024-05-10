import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, Twist
# from navigation.srv import SetGoal

import numpy as np

class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
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

        self.robot_pose = Pose2D()
        self.goal_point = Point()
        self.robot_pose_received = False
        self.goal_point_received = False

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True
    
    def goal_point_callback(self, msg):
        self.goal_point = msg
        self.goal_point_received = True

    def navigate(self):
        distance = float('inf')

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

        if distance > 0.1:
            cmd_vel.linear.y = np.min([0.2 * distance, 0.2])
            cmd_vel.angular.z = 2.0 * theta
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        self.publisher_cmd_vel.publish(cmd_vel)
        
        return True

def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()