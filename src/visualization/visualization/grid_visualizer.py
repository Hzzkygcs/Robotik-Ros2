import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
import cv2
import numpy as np

class GridMapVisualizer(Node):
    def __init__(self):
        super().__init__('opencv_grid_visualizer')
        self.subscriber_grid_map = self.create_subscription(
            OccupancyGrid,
            '/occupancy_map',
            self.occupancy_callback,
            10)
        self.subscriber_robot_pose = self.create_subscription(
            Pose2D,
            '/occupancy_map/robot_pose',
            self.robot_pose_callback,
            10)
        self.robot_pose = Pose2D()

    def occupancy_callback(self, msg):
        # self.get_logger().info('Received occupancy grid')
        # Convert OccupancyGrid data to a NumPy array
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

        # Scale the values to 0 - 255 for visualization (0=free, 100=occupied, -1=unknown)
        image = np.zeros_like(grid, dtype=np.uint8)
        image[grid == -1] = 255  # Free spaces to white
        image[grid == 100] = 0  # Occupied spaces to black
        image[grid == 0] = 127  # Unknown spaces to gray

        # Draw robot pose
        cv2.circle(image, (int(self.robot_pose.x), int(self.robot_pose.y)), 3, (0, 255, 0), -1)
        # Draw robot direction
        direction = np.array([np.cos(self.robot_pose.theta), np.sin(self.robot_pose.theta)])
        cv2.arrowedLine(image, (int(self.robot_pose.x), int(self.robot_pose.y)), (int(self.robot_pose.x + direction[0] * 10), int(self.robot_pose.y + direction[1] * 10)), (0, 0, 255), 2)

        # flip y axis
        flipped_image = np.flipud(image)

        # Display the image using OpenCV
        cv2.imshow("Occupancy Grid", flipped_image)
        cv2.waitKey(1)

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

def main(args=None):
    rclpy.init(args=args)
    visualizer = GridMapVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    cv2.destroyAllWindows()  # Make sure to destroy all OpenCV windows
    rclpy.shutdown()

if __name__ == '__main__':
    main()
