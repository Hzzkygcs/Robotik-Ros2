import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, Twist
# from navigation.srv import SetGoal
from sensor_msgs.msg import LaserScan
import time
from std_msgs.msg import String
import numpy as np

from navigation.models.raycasthit import RayCastHit


ROBOT_BODY_RADIUS = 0.9

# in POV from left (positive degree) to right (negative degree)
ROBOT_SENSOR_LIMIT_START = math.pi/2
ROBOT_SENSOR_LIMIT_END = -math.pi/2


class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
        self.raycast = LaserScan()
        self.raycast_received_timestamp = 0
        self.raycast_expire_duration = 1000
        self._raycast_results = []

        self.robot_pose = Pose2D()
        self._goal_point = Point()
        self.robot_pose_received = False
        self.goal_point_received = False
        self.get_logger().info('Running')

        self.subscription_scan = self.create_subscription(
            String,
            '/user_string_input',  # debugging purpose only
            self.user_input,
            10)
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

    def on_receive_laser_scan(self, message: LaserScan):  # angle in rads, 0=front, -90=right, +90=left
        ret = []
        for index, range in enumerate(self.raycast.ranges):
            range -= ROBOT_BODY_RADIUS
            angle = message.angle_min + message.angle_increment*index
            if range < self.raycast.range_min or range < 0:
                ret.append(RayCastHit(0, angle, self.robot_pose))
                continue
            if range > self.raycast.range_max:
                ret.append(RayCastHit(message.range_max, angle, self.robot_pose))
                continue
            ret.append(RayCastHit(range, angle, self.robot_pose))
        # ret.sort(key=lambda x: x.range)
        ret.reverse()  # so that index 0 is the one on the left-most, and last index is the one on the right-most
        self.raycast = message
        self.raycast_received_timestamp = int(time.time())
        self._raycast_results = ret
        self.goal_point_received_timestamp = 0

        # self.get_best_angle()


    def get_raycast_index_from_relative_angle(self, _relative_angle):  # 0 means the one in front of the robot. - in the right, + in the left
        relative_angle = _relative_angle
        while relative_angle < self.raycast.angle_min:
            relative_angle += 2*math.pi
        while relative_angle > self.raycast.angle_max:
            relative_angle -= 2*math.pi

        raycast_non_reversed_index = (relative_angle - self.raycast.angle_min)/self.raycast.angle_increment
        raycast_non_reversed_index = round(raycast_non_reversed_index)
        return len(self.raycast_results) - raycast_non_reversed_index

    def get_relative_angle_from_raycast_index(self, raycast_index):  # 0 means the one in front of the robot. - in the right, + in the left
        raycast_non_reversed_index = len(self.raycast_results) - raycast_index
        return self.raycast.angle_min + raycast_non_reversed_index*self.raycast.angle_increment

    def user_input(self, *args):
        hits = self.get_hits_bool_array()
        hits.insert(len(hits)//2, '9')

        if self.raycast_received:
            goal_angle = self.goal_angle()
            goal_angle_array_index = self.get_raycast_index_from_relative_angle(goal_angle)
            hits.insert(goal_angle_array_index, "D")

        recommended_angle = self.get_best_angle()
        if (self.raycast_received and recommended_angle is not None and
                self.raycast.angle_min < recommended_angle < self.raycast.angle_max):
            recommended_angle_index = self.get_raycast_index_from_relative_angle(recommended_angle)
            hits.insert(recommended_angle_index, "R")

        # TODO mark goal angle
        a = list(map(lambda x: str({True: 1, False: 0}.get(x, x)), hits))
        print("".join(a))
        print(f"{self.raycast.angle_min + self.raycast.angle_increment*len(hits)}  {self.raycast.angle_max}")


    def get_best_angle(self, *args):  # None means you should go backward
        if self.goal_is_reachable_without_obstacle_around:
            return None
        hits = self.get_hits_bool_array()
        goal_angle = self.goal_angle()
        if not self.raycast_received or not self.robot_pose_received:
            return None
        goal_is_in_left = goal_angle > math.pi/17
        goal_is_in_right = goal_angle < math.pi/17

        goal_angle_array_index = self.get_raycast_index_from_relative_angle(goal_angle)
        if goal_angle_array_index < 0 or goal_angle_array_index >= len(hits):
            if random.randint(1, 100) < 15:  # so it's not too laggy
                self.get_logger().info(f'angle none 1 - {goal_angle_array_index}/{len(hits)} {math.degrees(goal_angle)} {math.degrees(self.raycast.angle_min)}'
                                       f' {math.degrees(self.raycast.angle_max)}')
            return None  # go backward, the goal is not in raycast angle radius
        recommended_angle_index = find_closest_value(hits, goal_angle_array_index, False)
        angle = None
        if recommended_angle_index is not None:
            angle = recommended_angle_index[0]
            middle_angle, unbounded_left, unbounded_right, start, end = get_the_middle_index(hits, angle, False)

            obstacle_left = self.raycast_results[start].relative_position(self.robot_pose)
            obstacle_right = self.raycast_results[end].relative_position(self.robot_pose)

            space_width = obstacle_left.distance_to(obstacle_right)
            if not unbounded_left and not unbounded_right:
                # if the space is enclosed by obstacle on the left and the right and not wide, then
                # pick the middle. (Because if it's wide enough, then just go to the goal_point)
                angle = middle_angle
                if goal_is_in_left:
                    angle = start+(end-start)*4/5 + 8  # +x untuk jaga2
                elif goal_is_in_right:
                    angle = start+(end-start)*1/5 - 8
            elif goal_is_in_left and angle+5<len(hits) and hits[angle+5] is not True:  # is not pure True
                angle += 5  # go to slightly right because big chances obstacle are at the left
            elif goal_is_in_right and angle-5>=0 and hits[angle-5] is not True:
                angle -= 5  # go to slightly left



        if angle is None:
            return None
        ret = self.get_relative_angle_from_raycast_index(angle)

        if random.randint(1, 100) < 15:  # so it's not too laggy
            self.get_logger().info(f'Recommended angle: {math.degrees(ret)}:{recommended_angle_index} '
                                   f'goal={math.degrees(goal_angle)}:{goal_angle_array_index}'
                                   f' len={len(hits)}')
        return ret

    def get_hits_bool_array(self) -> list[bool]:
        ret = []
        if len(self.raycast_results) == 0:
            return ret
        prev_raycast = self.raycast_results[0]

        # any False will be non-pure True if they're within `dilation_radius`-radius to a pure-True (pure object hit)
        dilation_radius = 0
        for index,raycast in enumerate(self.raycast_results):
            is_hit = self.obstacle_will_hit(raycast)
            value = is_hit
            if is_hit and index-1 >= 0 and ret[index-1] is not True:
                for i in range(1, dilation_radius+1):
                    target_index_to_be_false = index - i
                    if target_index_to_be_false >= 0:
                        ret[target_index_to_be_false] = ret[target_index_to_be_false] or NonPureTrue.value
            elif is_hit is not True and any(filter(lambda x: x is True, ret[index-dilation_radius:index])):  # if any of previous are PURE true
                # become true if any of the previous n-item (n=dilation_radius) is true
                value = NonPureTrue.value
            ret.append(value)
            assert prev_raycast.angle >= raycast.angle
            prev_raycast = raycast
        sensor_start_index = self.get_raycast_index_from_relative_angle(ROBOT_SENSOR_LIMIT_START)
        sensor_end_index = self.get_raycast_index_from_relative_angle(ROBOT_SENSOR_LIMIT_END)
        for i in range(sensor_start_index):
            ret[i] = NonPureTrue('|')
        for i in range(sensor_end_index, len(ret)):
            ret[i] = NonPureTrue('|')
        return ret



    # obstacle angle is in radians, starts from 0 for right, pi/2 for straight in front of the robot, and pi for left
    def obstacle_will_hit(self, raycasthit: RayCastHit):
        obstacle_angle, obstacle_distance = raycasthit.angle, raycasthit.range
        goal_angle = self.goal_angle()
        x_relative = obstacle_distance * math.cos(obstacle_angle - goal_angle + math.pi/2)
        y_relative = obstacle_distance * math.sin(obstacle_angle - goal_angle + math.pi/2)
        y_target_relative = self.goal_distance()

        if y_relative < -ROBOT_BODY_RADIUS or y_relative > raycasthit.range+ROBOT_BODY_RADIUS:
            return False
        if math.fabs(x_relative) <= ROBOT_BODY_RADIUS:
            return True
        return False

    @property
    def goal_is_reachable_without_obstacle_around(self):
        if not self.robot_pose_received or not self.raycast_received:
            return True
        goal_angle_relative_to_robot_pov = self.goal_angle(True)

        if goal_angle_relative_to_robot_pov < self.raycast.angle_min or goal_angle_relative_to_robot_pov > self.raycast.angle_max:
            return False
        return self.angle_is_reachable_without_obstacle_around(goal_angle_relative_to_robot_pov,
                                                               self.goal_distance())

    def angle_is_reachable_without_obstacle_around(self, goal_angle, goal_distance, check_degree_to_left_right=10):
        if not self.raycast_received:
            return True
        target_angle_index = self.get_raycast_index_from_relative_angle(goal_angle)
        rads_degree = math.radians(check_degree_to_left_right)
        angle_index = math.ceil(rads_degree / self.raycast.angle_increment)
        raycasts = self.raycast_results
        for index in range(max(0,target_angle_index-angle_index), min(target_angle_index+angle_index, len(raycasts))):
            if raycasts[index].range <= goal_distance:
                return False
        return True



    @property
    def raycast_received(self) -> bool:
        if int(time.time()) - self.raycast_received_timestamp > self.raycast_expire_duration:
            return False
        return True

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

    def goal_distance(self):  # goal angle relative to current robot position
        dx = self._goal_point.x - self.robot_pose.x
        dy = self._goal_point.y - self.robot_pose.y
        gdistance = np.sqrt(dx**2 + dy**2)
        return gdistance


    def goal_angle(self, relative_to_pov=True, angle_range_start=-math.pi, angle_range_end=math.pi + 0.001):
        dx = self._goal_point.x - self.robot_pose.x
        dy = self._goal_point.y - self.robot_pose.y
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
        distance = float('inf')
        if self.goal_point is None:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.publisher_cmd_vel.publish(cmd_vel)
            return True
        best_angle = self.get_best_angle()
        if random.randint(1,100) < 10:
            self.user_input()
        # best_angle = None

        dx = self.goal_point.x - self.robot_pose.x
        dy = self.goal_point.y - self.robot_pose.y
        distance = np.sqrt(dx**2 + dy**2)
        goal_angle = np.arctan2(dy, dx)

        theta = best_angle
        if best_angle is None:
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

def find_closest_value(array: list, start_index: int, value_to_find) -> Optional[list[int]]:
    assert not start_index < 0 or start_index >= len(array)

    left_exist = True
    right_exist = True
    distance = 3
    ret = []
    while left_exist and right_exist:
        if start_index-distance < 0:
            left_exist = False
        if start_index+distance >= len(array):
            right_exist = False
        if left_exist and array[start_index-distance] == value_to_find:
            ret.append(start_index-distance)
        if right_exist and array[start_index+distance] == value_to_find:
            ret.append(start_index+distance)
        if len(ret) != 0:
            return ret
        distance += 1
    return None


def get_the_middle_index(array: list, start_index: int, value_to_find):
    assert array[start_index] == value_to_find
    start = start_index
    while start-1 >= 0 and array[start-1] == value_to_find:
        start -= 1
    touch_left_index = (start == 0)
    end = start_index
    while end+1 < len(array) and array[end+1] == value_to_find:
        end += 1
    touch_right_index = (end == len(array)-1)
    mid = (start+end)//2
    return mid, touch_left_index, touch_right_index, start, end



class NonPureTrue:
    def __init__(self, repr='-'):
        self.repr = repr

    def __bool__(self):
        return True
    def __repr__(self):
        return self.repr
NonPureTrue.value = NonPureTrue()


def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()