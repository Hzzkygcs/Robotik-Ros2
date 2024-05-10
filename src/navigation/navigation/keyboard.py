import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, Twist
# from navigation.srv import SetGoal
import curses
import numpy as np
import time


class Keyboard(Node):
    def __init__(self):
        super().__init__('keyboard')
        self.subscriber_robot_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        self.publisher_goal_point = self.create_publisher(
            Point,
            '/goal_point',
            10)
        timer_period_in_seconds = 0.2
        self.timer = self.create_timer(timer_period_in_seconds, self.update_goal_point_from_keyboard)
        self.current_goal_point = None
        self.robot_is_moving = False
        self.keyboard_input_stdscr = screen
        self.keyboard_input_stdscr.addstr(2, 20, "")
        self.already_stopped = False
        self.movement_distance = 0.1
        self.exit_signal = False

    def robot_pose_callback(self, robotPoseMsg):
        self.position = robotPoseMsg
        if self.current_goal_point is not None:
            return
        self.robot_is_moving = False
        self.current_goal_point = [self.position.x, self.position.y]
        self.keyboard_input_stdscr.addstr(1, 20, 'current_goal_point intiialized')

    def update_goal_point_from_keyboard(self):
        if self.current_goal_point is None:
            self.keyboard_input_stdscr.addstr(2, 20, 'current_goal_point not yet initialized. Keyboard input will be ignored')
            return
        key = self.keyboard_input_stdscr.getch()
        self.keyboard_input_stdscr.clrtoeol()
        stop = False

        if key == ord('w'):
            self.current_goal_point[1] += self.movement_distance
            self.keyboard_input_stdscr.addstr(2, 20, "Up")
        elif key == ord('q'):
            curses.endwin()
            self.keyboard_input_stdscr.addstr(2, 20, "Stopping...")
            self.exit_signal = True
        elif key == ord('s'):
            self.current_goal_point[1] -= self.movement_distance
            self.keyboard_input_stdscr.addstr(2, 20, "Down")
        elif key == ord('a'):
            self.current_goal_point[0] -= self.movement_distance
            self.keyboard_input_stdscr.addstr(2, 20, "Left")
        elif key == ord('d'):
            self.current_goal_point[0] += self.movement_distance
            self.keyboard_input_stdscr.addstr(2, 20, "Right")
        else:
            stop = True
            self.publish_goal_point()
            self.keyboard_input_stdscr.addstr(2, 20, "Stop")
            self.current_goal_point = [self.position.x, self.position.y]
            self.already_stopped = True
        if not stop:
            self.already_stopped = False
            self.publish_goal_point()

        curses.flushinp()
        self.keyboard_input_stdscr.clrtoeol()
        self.keyboard_input_stdscr.addstr(3, 20, str(int(time.time())) + " " + str(key))
        self.keyboard_input_stdscr.refresh()

    def publish_goal_point(self):
        msg = Point()
        msg.x = float(self.current_goal_point[0])
        msg.y = float(self.current_goal_point[1])
        self.publisher_goal_point.publish(msg)
        if self.exit_signal:
            self.destroy_node()
            raise SystemExit




def main(args=None):
    global screen  # TODO move to constructor params but idk how
    screen = curses.initscr()
    curses.cbreak()
    screen.keypad(1)


    rclpy.init(args=args)
    keyboard = Keyboard()
    rclpy.spin(keyboard)
    keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()