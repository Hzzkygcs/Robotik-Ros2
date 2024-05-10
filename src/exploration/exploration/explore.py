import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose2D

import cv2

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.subscription_pose = self.create_subscription(
            Pose2D,
            '/occupancy_map/robot_pose',
            self.robot_pose_callback,
            10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/occupancy_map',
            self.map_callback,
            10)
        # self.publisher = self.create_publisher(
        #     Path,
        #     '/exploration_path',
        #     10)
        self.publisher_goal = self.create_publisher(
            Point,
            '/goal_point',
            10)
        self.current_map = OccupancyGrid()
        self.current_pose = Pose2D()
        self.directions = [(-10, 0), (10, 0), (0, -10), (0, 10), (-10, -10), (-10, 10), (10, -10), (10, 10)]

        self.debug_img = None

    def map_callback(self, msg):
        self.current_map = msg
        self.explore_frontier(msg)

    def explore_frontier(self, map):
        if map is None:
            return
        
        grid = np.array(map.data).reshape((map.info.height, map.info.width))

        # Debug
        self.debug_img = np.zeros_like(grid, dtype=np.uint8)
        self.debug_img[grid == -1] = 255  # Free spaces to white
        self.debug_img[grid == 100] = 0  # Occupied spaces to black
        self.debug_img[grid == 0] = 127  # Unknown spaces to gray

        frontiers = self.detect_frontiers(grid)
        self.get_logger().info(f'Found {len(frontiers)} frontiers')
        if not frontiers:
            self.get_logger().info('No frontiers found')
            return

        start_position = (int(self.current_pose.x), int(self.current_pose.y))
        self.get_logger().info(f'Starting exploration from: {start_position}')
        target = self.select_closest_frontier(frontiers, start_position)
        self.get_logger().info(f'Target: {target}')

        paths = self.plan_path(grid, start_position, target)

        img = np.copy(self.debug_img)
        past_path = paths[0]
        for path in paths:
            cv2.line(img, (past_path[0], past_path[1]), (path[0], path[1]), (200, 0, 0), 2)
            past_path = path
        cv2.imshow('Debug', img)
        cv2.waitKey(0)

        self.get_logger().info(f'Found {len(paths)} paths')
        self.publish_path(paths)

    def detect_frontiers(self, grid):
        frontiers = []
        rows, cols = grid.shape
        
        for r in range(1, rows-1):
            for c in range(1, cols-1):
                if grid[r, c] == -1:  # Check if the cell is free
                    for dr, dc in self.directions:
                        rr, cc = r + dr, c + dc
                        
                        if(rr < 0 or rr >= rows or cc < 0 or cc >= cols): continue

                        if grid[rr, cc] == 0:  # Check if neighboring cell is unknown
                            frontiers.append((r, c))
                            break
        return frontiers


    def robot_pose_callback(self, msg):
        self.current_pose = msg

    def select_closest_frontier(self, frontiers, start_position):
        closest = None
        min_distance = float('inf')
        start_x, start_y = start_position
        for frontier in frontiers:
            fx, fy = frontier
            distance = np.sqrt((fx - start_x) ** 2 + (fy - start_y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest = frontier
        return closest


    def plan_path(self, grid, start, goal):
        import heapq

        def heuristic(a, b):
            return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(start, goal)}
        oheap = []

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]
            # print(f'Exploring: {current}', end='\r')
            # self.get_logger().info(f'Exploring: {current}')

            # calculate distance
            dis = np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)
            # self.get_logger().info(f'Distance: {dis}')
            if dis < 10:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data

            close_set.add(current)
            for i, j in self.directions:
                neighbor = current[0] + i, current[1] + j

                tentative_g_score = gscore[current] + heuristic(current, neighbor)
                if 0 <= neighbor[0] < grid.shape[0]:
                    if 0 <= neighbor[1] < grid.shape[1]:
                        if grid[neighbor[0], neighbor[1]] == 100 or neighbor in close_set:
                            continue
                    else:
                        continue
                else:
                    continue

                if neighbor not in oheap or tentative_g_score < gscore.get(neighbor, 0):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
            
            img = np.copy(self.debug_img)
            # print('===========================================')
            # print(len(came_from))
            # while current in came_from:
            #     pos = came_from[current]
            #     print(pos)
            #     cv2.circle(img, (pos[0], pos[1]), 1, (200, 0, 0), 5)
            # cv2.circle(img, (current[0], current[1]), 1, (200, 0, 0), 5)

            # flipped_debug_img = np.flipud(img)
            # cv2.imshow("Debug Explore", flipped_debug_img)
            # cv2.waitKey(1)
        
        return []

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for point in path:
            print(point)
        #     pose = Point()
        #     pose.header = path_msg.header
        #     pose.pose.position.x = point[1] * self.current_map.info.resolution
        #     pose.pose.position.y = point[0] * self.current_map.info.resolution
        #     path_msg.poses.append(pose)
        # self.publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    explorer = Explorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
