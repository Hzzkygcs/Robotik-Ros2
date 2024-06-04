from __future__ import annotations

import time, math

import numpy as np
from geometry_msgs.msg import Twist


class MovementOverride:
    def __init__(self, end_duration_relative, next_override: MovementOverride=None):
        self.end_duration = int(time.time()) + end_duration_relative
        self.next_override = next_override
        self.is_valid_override = None

    @property
    def is_valid(self):
        if self.is_valid_override is not None:
            return self.is_valid_override
        return int(time.time()) < self.end_duration

    @is_valid.setter
    def is_valid(self, value):
        self.is_valid_override = value


    @property
    def twist(self):
        raise NotImplementedError()

    def _get_twist_of_next_override(self):
        return self.next_override.twist if self.next_override is not None else None

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
    def __init__(self, _ignored=None, next_override: MovementOverride=None, event_callback=None):
        super().__init__(-1)
        self.next_override = next_override
        self.event_callback = event_callback

    @property
    def twist(self):
        if self.event_callback is not None:
            self.event_callback()
            self.event_callback = None
        return self.next_override.twist if self.next_override is not None else None

DEGREE = 10
MOVEMENT_SPEED = 2.5

class BackwardMovementOverride(MovementOverride):
    def __init__(self, obstacle_on_left: bool, get_robot_position, on_obstacle_hit, expire_duration=3, speed_multiplier=1,
                 next_override: MovementOverride=None):
        super().__init__(expire_duration)
        self.get_robot_position = get_robot_position
        self.on_obstacle_hit = on_obstacle_hit
        self.start_time = time.time()
        self.previous_position = None
        self.prev_frame_time = None

        self.next_override = next_override
        self.obstacle_at_left = obstacle_on_left
        self.speed_multiplier = speed_multiplier

    @property
    def twist(self):
        if not self.is_valid:
            return self._get_twist_of_next_override()
        curr_pos = self.get_robot_position()
        if self.previous_position is not None and (time.time() - self.start_time) > 0.5:
            dx = self.previous_position.x - curr_pos.x
            dy = self.previous_position.y - curr_pos.y
            vel = np.hypot(dx, dy) / (time.time() - self.prev_frame_time)
            if vel < (MOVEMENT_SPEED * self.speed_multiplier) / 2:
                self.on_obstacle_hit()
                self.is_valid = False  # robot is not moving
        self.previous_position = curr_pos
        self.prev_frame_time = time.time()
        ret = Twist()
        theta = math.radians(-DEGREE if self.obstacle_at_left else DEGREE) * 3
        ret.linear.y = -MOVEMENT_SPEED * self.speed_multiplier
        ret.angular.z = 2.0 * theta
        return ret



class ForwardMovementOverride(MovementOverride):
    def __init__(self, go_to_left_func, angle_multiplier, end_duration_relative=3, next_override: MovementOverride=None):
        super().__init__(end_duration_relative)
        self.next_override = next_override
        self.go_to_left_func = go_to_left_func
        self.angle_multiplier = angle_multiplier

    @property
    def twist(self):
        if not self.is_valid:
            return self._get_twist_of_next_override()
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
    def twist(self):
        if not self.is_valid:
            return self._get_twist_of_next_override()
        theta = self.get_goal_angle_func() - self.get_robot_pose_func().theta

        while theta > math.pi:
            theta -= 2*math.pi
        while theta < -math.pi:
            theta += 2*math.pi

        cmd_vel = Twist()
        cmd_vel.linear.y = 0.4
        cmd_vel.angular.z = 2.0 * theta
        return cmd_vel

class RotateInPlace(MovementOverride):
    def __init__(self, angular_velocity_func, end_duration_relative=3, next_override: MovementOverride=None):
        super().__init__(end_duration_relative, next_override)
        self.angular_velocity_func = angular_velocity_func

    @property
    def twist(self):
        if not self.is_valid:
            return self._get_twist_of_next_override()
        cmd_vel = Twist()
        cmd_vel.linear.y = 0.0

        angular_velocity = self.angular_velocity_func()
        if angular_velocity is None:
            self.end_duration = 0  # set as no longer valid
            return self._get_twist_of_next_override()
        cmd_vel.angular.z = float(angular_velocity)
        return cmd_vel


