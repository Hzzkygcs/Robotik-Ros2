import rclpy
import keyboard
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point

TICK_RATE = 0.7

class KeyboardControls(Node):
    def __init__(self):
        super().__init__('keyboardcontrols')
        self.subscriber_robot_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.robot_pose_callback,
            qos_profile=10)
        self.publisher_goal_point = self.create_publisher(
            Point,
            '/goal_point',
            10)

        self.timer = self.create_timer(TICK_RATE, self.keyboardcontrols)

        self.robot_pose = Pose2D()
        self.robot_pose_received = False
        self.direction_length = 4
        self.is_moving = False

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True

    def new_goal_point(self, x, y, z=0):
        new_goal_point = Point()
        new_goal_point.x = x
        new_goal_point.y = y
        new_goal_point.z = z
        return new_goal_point

    def move(self, direction: str, x, y, z=0):
        print(f"Moving {direction}!")
        dx = self.robot_pose.x + x
        dy = self.robot_pose.y + y
        new_goal_point = self.new_goal_point(dx, dy)
        self.publisher_goal_point.publish(new_goal_point)
        self.is_moving = True

    def stop(self):
        self.publisher_goal_point.publish(self.new_goal_point(self.robot_pose.x, self.robot_pose.y,0))
        print("Stopped moving.")
        self.is_moving = False

    def keyboardcontrols(self):
        direction = ""
        x = 0
        y = 0
        if keyboard.is_pressed("up"):
            direction += "up "
            y += self.direction_length
        if keyboard.is_pressed('down'):
            direction += "down "
            y += -self.direction_length
        if keyboard.is_pressed('left'):
            direction += "left "
            x += -self.direction_length
        if keyboard.is_pressed('right'):
            direction += "right "
            x += self.direction_length

        if direction != "":
            self.move(direction, x, y, 0)
        elif self.is_moving:
            self.stop()

        return True


def main(args=None):
    rclpy.init(args=args)
    keyboardcontrols = KeyboardControls()
    rclpy.spin(keyboardcontrols)
    keyboardcontrols.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
