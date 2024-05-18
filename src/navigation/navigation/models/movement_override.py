from __future__ import annotations

import time, math
from geometry_msgs.msg import Twist


class MovementOverride:
    def __init__(self, end_duration_relative, next_override: MovementOverride=None):
        self.end_duration = int(time.time()) + end_duration_relative
        self.next_override = next_override

    @property
    def is_valid(self):
        return int(time.time()) < self.end_duration

    @property
    def get_twist(self):
        raise NotImplementedError()

    def get_twist_of_next_override(self):
        return self.next_override.get_twist if self.next_override is not None else None

    @staticmethod
    def chain(*movement_overrides: MovementOverride):
        if len(movement_overrides) == 0:
            return None
        for i in range(len(movement_overrides)):
            if i+1 >= len(movement_overrides):
                break
            movement_overrides[i].next_override = movement_overrides[i+1]
        return movement_overrides[0]


class NopMovementOverride(MovementOverride):
    def __init__(self, _ignored=None, next_override: MovementOverride=None):
        super().__init__(-1)
    @property
    def get_twist(self):
        return

DEGREE = 7
MOVEMENT_SPEED = 0.4

class BackwardMovementOverride(MovementOverride):
    def __init__(self, obstacle_on_left: bool, expire_duration=3, next_override: MovementOverride=None):
        super().__init__(expire_duration)
        self.next_override = next_override
        self.obstacle_at_left = obstacle_on_left

    @property
    def get_twist(self):
        if not self.is_valid:
            return self.get_twist_of_next_override()
        ret = Twist()
        theta = math.radians(-DEGREE if self.obstacle_at_left else DEGREE) * 3
        ret.linear.y = -MOVEMENT_SPEED
        ret.angular.z = 2.0 * theta
        return ret


class ForwardMovementOverride(MovementOverride):
    def __init__(self, go_to_left_func, angle_multiplier, end_duration_relative=3, next_override: MovementOverride=None):
        super().__init__(end_duration_relative)
        self.next_override = next_override
        self.go_to_left_func = go_to_left_func
        self.angle_multiplier = angle_multiplier

    @property
    def get_twist(self):
        if not self.is_valid:
            return self.get_twist_of_next_override()
        ret = Twist()
        theta = math.radians(DEGREE if self.go_to_left_func() else -DEGREE)*self.angle_multiplier
        ret.linear.y = MOVEMENT_SPEED
        ret.angular.z = 2.0 * theta
        return ret


class RotateTowardGoalOverride(MovementOverride):
    def __init__(self, get_robot_pose_func, get_goal_angle_func, publish_result, end_duration_relative=3, next_override: MovementOverride=None):
        super().__init__(end_duration_relative, next_override)
        self.get_robot_pose_func = get_robot_pose_func
        self.get_goal_angle_func = get_goal_angle_func
        self.publish_result = publish_result



    @property
    def get_twist(self):
        if not self.is_valid:
            return self.get_twist_of_next_override()
        theta = self.get_goal_angle_func() - self.get_robot_pose_func().theta

        while theta > math.pi:
            theta -= 2*math.pi
        while theta < -math.pi:
            theta += 2*math.pi

        cmd_vel = Twist()
        cmd_vel.linear.y = 0.4
        cmd_vel.angular.z = 2.0 * theta
        return cmd_vel