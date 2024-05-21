import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Point, Twist
# from navigation.srv import SetGoal
from std_msgs.msg import String

import numpy as np

from navigation.models.movement_override import BackwardMovementOverride, ForwardMovementOverride, MovementOverride, \
    NopMovementOverride, RotateInPlace

TICK_RATE = 0.01  # in seconds
FALLBACK_DISTANCE = 0.66
AVOID_DISTANCE = 0.33
MAX_ANGLE_DEGREE_TOWARD_GOAL = 10

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
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/hokuyo',
            self.listener_scan,
            10)
        self.subscription_user_input = self.create_subscription(
            String,
            '/user_string_input',  # for debugging purpose only
            self.user_input,
            10
        )

        self.timer = self.create_timer(TICK_RATE, self.navigate)

        self.laser_scan = LaserScan()
        self.laser_scan_received = False

        self.robot_pose = Pose2D()
        self.robot_pose_received = False

        self.goal_point = Point()
        self.goal_point_received = False
        self.movement_override = NopMovementOverride()

    def listener_scan(self, msg):
        self.laser_scan = msg
        self.laser_scan_received = True

    def user_input(self, msg):
        self.movement_override = RotateInPlace(3, 2)


    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True

    def goal_point_callback(self, msg):
        self.goal_point = msg
        self.goal_point_received = True

    def obstacle_avoidance(self):
        short_distance = float('inf')
        predicted_goal_angle = 0

        if not self.laser_scan_received:  # Obstacle Avoidance
            return False

        distances = np.array(self.laser_scan.ranges)
        angles = np.linspace(self.laser_scan.angle_min, self.laser_scan.angle_max, len(self.laser_scan.ranges))
        shortest = distances.argmin()
        short_angle = angles[shortest]  # Left of robot is positive, right of robot is negative, 0 is front
        short_distance = distances[shortest]

        shortest_direction_absolute = (self.robot_pose.theta + short_angle) % 6.28
        predicted_goal_angle = shortest_direction_absolute
        if predicted_goal_angle > 3.14:  # Direction relative to robot closest to obstacle
            predicted_goal_angle += -6.28
        obstacle_on_left = short_angle > 0
        avoid_distance = 0.33
        too_close = False

        if short_distance > avoid_distance:
            return False
        print(f"Too close!: {predicted_goal_angle}")
        self.robot_go_bakcward(obstacle_on_left)
        return True


    def robot_go_bakcward(self, obstacle_left):
        self.get_logger().info(f"GOING BACKWARD. obstacle at the {'left' if obstacle_left else 'right'}")

        expire_duration = 0
        goal_is_in_left_func = lambda: self.goal_angle() > 0
        self.movement_override = MovementOverride.chain(
            BackwardMovementOverride(obstacle_left, expire_duration:= expire_duration + 1.5),
            ForwardMovementOverride(lambda: not obstacle_left, 0.5, expire_duration:= expire_duration + 0.5),
            ForwardMovementOverride(goal_is_in_left_func, 1, expire_duration:= expire_duration + 0.5),
        )
        return True


    def goal_angle(self, relative_to_pov=True, angle_range_start=-math.pi, angle_range_end=math.pi + 0.001):
        dx = self.goal_point.x - self.robot_pose.x
        dy = self.goal_point.y - self.robot_pose.y
        goal_angle = np.arctan2(dy, dx)
        if relative_to_pov:
            # goal_angle -= math.pi/2
            goal_angle -= self.robot_pose.theta
        while goal_angle < angle_range_start:
            goal_angle += math.pi*2
        while goal_angle > angle_range_end:
            goal_angle -= math.pi*2
        return goal_angle

    def navigate(self):
        # print(f"Received goal point: {self.goal_point}")
        twist = self.movement_override.twist
        if twist is not None:
            self.publisher_cmd_vel.publish(twist)
            return
        if  self.goal_distance() < 0.3:
            self.publisher_cmd_vel.publish(Twist())
            return
        goal_angle_deg = math.degrees(self.goal_angle())
        if abs(goal_angle_deg) > MAX_ANGLE_DEGREE_TOWARD_GOAL:
            def rotation():
                deg = math.degrees(self.goal_angle())
                if abs(deg) <= MAX_ANGLE_DEGREE_TOWARD_GOAL:
                    return None
                return math.copysign(deg/14, deg)
            print(f"Rotating... {goal_angle_deg}")
            self.movement_override = RotateInPlace(rotation)

        # if self.obstacle_avoidance():
        #     return

        dx = self.goal_point.x - self.robot_pose.x
        dy = self.goal_point.y - self.robot_pose.y
        distance = self.goal_distance()

        # Direction where we should face, same as theta but quadrant 3-4 is negative (-3.14->-0), 4 max is -0
        goal_angle = np.arctan2(dy, dx)
        print(f"Robot's heading to: {self.goal_point.x, self.goal_point.y, goal_angle}")

        # quadrant 1 is 0-1.5, 2 is 1.6-3.14, 3 is 3.15-4.6~, 4 is 4.6-6.28
        # print(f"Robot's facing: {self.robot_pose.theta}")
        theta = goal_angle - self.robot_pose.theta  # Adjusted direction based on where the robot's facing
        goal_angle_adjusted = goal_angle
        if goal_angle_adjusted < 0:
            goal_angle_adjusted = goal_angle % math.pi

        while theta > np.pi:
            theta -= 2 * np.pi
        while theta < -np.pi:
            theta += 2 * np.pi

        cmd_vel = Twist()

        if distance > 0.3:
            cmd_vel.linear.y = 0.6

            print(goal_angle_adjusted, self.robot_pose.theta)
            # if abs(goal_angle_adjusted - self.robot_pose.theta) < 0.2:  # Only move when facing the correct direction
            #     print("Facing correctly")
            #     cmd_vel.linear.y = 0.6
            cmd_vel.angular.z = 2.0 * theta
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        self.publisher_cmd_vel.publish(cmd_vel)

        return True
    def goal_distance(self):
        dx = self.goal_point.x - self.robot_pose.x
        dy = self.goal_point.y - self.robot_pose.y
        return np.sqrt(dx ** 2 + dy ** 2)


def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()