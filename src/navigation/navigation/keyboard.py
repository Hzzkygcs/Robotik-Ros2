import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, Twist
# from navigation.srv import SetGoal
import curses
import numpy as np
import time
from std_msgs.msg import String


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

        self.publisher_user_string_input = self.create_publisher(
            String,
            '/user_string_input',  # for debugging purpose only
            10)
        timer_period_in_seconds = 0.2
        self.timer = self.create_timer(timer_period_in_seconds, self.update_goal_point_from_keyboard)
        self.current_goal_point = None
        self.robot_is_moving = False
        self.keyboard_input_stdscr = screen
        self.set_stdscr_timeout()
        self.keyboard_input_stdscr.addstr(1, 0, "Press W to go Up, S to go down, A to go left, D to go right, ")
        self.keyboard_input_stdscr.addstr(2, 0, "or Q to quit. Press i for something special")
        self.set_status("")
        self.already_stopped = False
        self.movement_distance = 0.3
        self.exit_signal = False
        self.start_time = int(time.time())
        self.user_input_mode = False

    def set_stdscr_timeout(self):
        self.keyboard_input_stdscr.timeout(400)

    def robot_pose_callback(self, robotPoseMsg):
        self.position = robotPoseMsg
        if self.current_goal_point is not None:
            return
        self.robot_is_moving = False
        self.current_goal_point = [self.position.x, self.position.y]
        self.keyboard_input_stdscr.addstr(4, 20, 'current_goal_point intiialized')

    def update_goal_point_from_keyboard(self):
        if self.current_goal_point is None:
            self.keyboard_input_stdscr.addstr(5, 20, 'current_goal_point not yet initialized. Keyboard input will be ignored')
            return
        if self.user_input_mode:
            return
        key = self.keyboard_input_stdscr.getch()
        self.keyboard_input_stdscr.clrtoeol()
        stop = False

        if key == ord('w'):
            self.current_goal_point[1] += self.movement_distance
            self.set_status("Up")
        elif key == ord('q'):
            curses.endwin()
            self.set_status("Stopping...")
            self.exit_signal = True
        elif key == ord('s'):
            self.current_goal_point[1] -= self.movement_distance
            self.set_status("Down")
        elif key == ord('a'):
            self.current_goal_point[0] -= self.movement_distance
            self.set_status("Left")
        elif key == ord('d'):
            self.current_goal_point[0] += self.movement_distance
            self.set_status("Right")
        elif key == ord('i'):
            self.manual_input_mode(self.manual_input_mode_callback, msg="Set target goal relative to current robot in `x,y` format")
            stop = True
            self.already_stopped = True  # do not send "stop goal_point" until the next keypress
        elif key == ord('p'):
            def func(data: bytes):
                ret = String()
                ret.data = data.decode()
                self.publisher_user_string_input.publish(ret)
            self.manual_input_mode(func, msg="Type your input (max 15 chars)")
            stop = True
        else:
            stop = True
            self.stop()
        if not stop:
            self.already_stopped = False
            self.publish_goal_point()

        curses.flushinp()
        self.keyboard_input_stdscr.clrtoeol()
        # self.keyboard_input_stdscr.addstr(8, 20, str(int(time.time()) - self.start_time) + " " + str(key))
        self.keyboard_input_stdscr.refresh()

    def manual_input_mode(self, callback, msg=''):
        self.user_input_mode = True
        self.keyboard_input_stdscr.timeout(-1)
        self.keyboard_input_stdscr.clear()
        self.keyboard_input_stdscr.addstr(3, 0, msg)
        user_input = self.keyboard_input_stdscr.getstr(0,0, 15)
        self.keyboard_input_stdscr.clear()
        callback(user_input)
        self.set_stdscr_timeout()
        self.user_input_mode = False

    def manual_input_mode_callback(self, _user_input: bytes):
        user_input = _user_input.decode()
        self.keyboard_input_stdscr.addstr(9, 20, f"result: {user_input}")
        if ',' not in user_input:
            self.keyboard_input_stdscr.addstr(9, 20, f"User input not valid. Should be `int,int` format")
            return
        x,y = user_input.split(",")
        try:
            x = float(x.strip())
            y = float(y.strip())
        except:
            self.keyboard_input_stdscr.addstr(9, 20, f"X or Y is not valid. Both should be in float format")
            return
        self.current_goal_point = [self.position.x + x, self.position.y + y]
        self.publish_goal_point()
        self.set_status(f"visiting user's defined position, rel={[x,y]} abs={self.current_goal_point}")




    def stop(self):
        if self.already_stopped:
            return
        self.publish_goal_point()
        self.set_status("Stop")
        self.already_stopped = True
        pos = [self.position.x, self.position.y]
        for i in range(2):
            self.current_goal_point[i] += pos[i]*2
            self.current_goal_point[i] /= 3

    def set_status(self, status):
        self.keyboard_input_stdscr.addstr(5, 20, f"Current status: {status}")

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