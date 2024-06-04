from __future__ import annotations
import abc
import collections
import math
import random

from .direction import *
from .mapconstants import *
from .numpymap import NumpyMap


class ExplorationBase(abc.ABC):
    def __init__(self):
        pass

    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple, redo_bfs=True):
        pass

    def get_destination(self):
        pass

    def move_on_to_next_destination(self):
        pass

    def overall_destinations(self):
        pass

    def redo_bfs(self):
        raise NotImplementedError

    def tick_to_check_if_need_replan(self, updated_map, robot_pose) -> bool:
        """ Return true if replan is done, and false otherwise """
        raise NotImplementedError


class ExplorationEvent(ExplorationBase):
    def __init__(self, bfsStrategy: ExplorationBase, onComplete):
        self.chains: list = [bfsStrategy]
        self.onComplete = onComplete
        self.numpyMap = None
        self.currentActualPos = None
        self.dirty_bit = True
        self.finished = False

    # Used to update routing path but don't do it on every mapping update, only occasionally (for performance reason).
    # Will definitely update routing path if not yet intiialized
    def try_set_map(self, numpyMap: NumpyMap, currentActualPos: tuple, chance: int):
        assert 0 <= chance <= 100
        dont_redo_bfs = (random.randint(0, 100) > chance and self.numpyMap is not None and len(self.overall_destinations()) > 0)
        ret = self.set_map(numpyMap, currentActualPos, redo_bfs=not dont_redo_bfs)
        if dont_redo_bfs:
            return True
        print(self.overall_destinations())
        return ret

    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple, redo_bfs=True):
        if self.finished:
            return True
        self.numpyMap = numpyMap
        self.currentActualPos = currentActualPos
        chain = self.chains[0]
        success = chain.set_map(numpyMap, currentActualPos, redo_bfs=redo_bfs)
        if not success:
            self.finished = True
            self.onComplete()
            return False
        self.dirty_bit = redo_bfs
        return True

    def get_destination(self):
        raise NotImplementedError  # it's deprecated
        # if self.finished:
        #     return True
        # chain: ExplorationBase = self.chains[0]
        # curr_destination = chain.get_destination()
        # if curr_destination is None:
        #     curr_destination = self.move_on_to_next_destination()
        # if curr_destination is not None:
        #     return curr_destination
        # self.finished = True
        # self.onComplete()
        # # self.chains.pop(0)  # would be better to use queue tho


    # def move_on_to_next_destination(self):
    #     first_item: ExplorationBase = self.chains[0]
    #     result = first_item.move_on_to_next_destination()
    #     return result

    def overall_destinations(self):
        return self.chains[0].overall_destinations()

    def tick_to_check_if_need_replan(self, updated_map, robot_pose):
        return self.chains[0].tick_to_check_if_need_replan(updated_map, robot_pose)

    def reset_dirty_bit(self):
        ret = self.dirty_bit
        self.dirty_bit = False
        return ret

class ExploreUnknownMaps(ExplorationBase):
    def __init__(self, max_num_of_compressed):
        self.bfs = DoBfs(self.stoppingCondition, lambda x, y, value: value >= PATH_OBSTACLE_TRESHOLD, max_num_of_compressed)
        self.numpyMap = None
        self.canvas = None
        self.currentActualPos = None

    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple, redo_bfs=True):
        self.numpyMap = numpyMap
        self.canvas = numpyMap.canvas
        self.currentActualPos = currentActualPos
        return self.bfs.set_map(numpyMap, currentActualPos, redo_bfs=redo_bfs)

    def redo_bfs(self):
        self.bfs.set_map(self.numpyMap, self.currentActualPos)

    def stoppingCondition(self, node_info: NodeInformation):
        x = node_info.x
        y = node_info.y
        return self.canvas[y][x] == PATH_UNKNOWN_VALUE

    def move_on_to_next_destination(self):
        return self.bfs.move_on_to_next_destination()

    def get_destination(self):
        return self.bfs.get_destination()

    def overall_destinations(self):
        return self.bfs.overall_destinations()

    def tick_to_check_if_need_replan(self, updated_map, robot_pose):
        temp = self.bfs.overall_destinations()
        if temp is None or len(temp) == 0:
            self.bfs.set_map(updated_map, (robot_pose.x, robot_pose.y), redo_bfs=True)
            return True
        _, _, final_dest = temp[-1]
        if updated_map.canvas[final_dest[1]][final_dest[0]] and robot_pose is not None:
            self.bfs.set_map(updated_map, (robot_pose.x, robot_pose.y), redo_bfs=True)
            return True
        return False


class BfsToDestination(ExplorationBase):
    def __init__(self, destination_actual_coordinate: tuple, max_num_of_compressed):
        self.bfs = DoBfs(self.stoppingCondition, lambda x, y, value: value >= PATH_OBSTACLE_TRESHOLD, max_num_of_compressed)
        self.numpyMap = None
        self.canvas = None
        self.arrived = False
        self.destination_actual_coordinate = destination_actual_coordinate

    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple, redo_bfs=True):
        if self.arrived:
            return False
        self.numpyMap = numpyMap
        self.destination_px_coord = numpyMap.actual_to_px(self.destination_actual_coordinate)
        self.canvas = numpyMap.canvas
        return self.bfs.set_map(numpyMap, currentActualPos, redo_bfs=redo_bfs)

    def stoppingCondition(self, node_info: NodeInformation):
        x = node_info.x
        y = node_info.y
        return self.destination_px_coord[0] == x and self.destination_px_coord[1] == y

    def move_on_to_next_destination(self):
        return self.bfs.move_on_to_next_destination()

    def get_destination(self):
        return self.bfs.get_destination()

    def overall_destinations(self):
        return self.bfs.overall_destinations()

    def tick_to_check_if_need_replan(self, updated_map, robot_pose):
        dx = self.destination_actual_coordinate[0] - robot_pose.x
        dy = self.destination_actual_coordinate[1] - robot_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        if distance < 0.3:
            self.arrived = True
        return False

    def reset_dirty_bit(self):
        pass


class DoBfs(ExplorationBase):
    def __init__(self, stopping_condition, wall_condition, max_num_of_compressed):
        self.numpyMap = None
        self.currentActualPos = None
        self.shortest_maps = []
        self.routes = collections.deque()
        self.stopping_condition = stopping_condition
        self.wall_condition = wall_condition
        self.max_num_of_compressed = max_num_of_compressed

    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple, redo_bfs=True):
        self.numpyMap = numpyMap
        self.currentActualPos = currentActualPos
        self.currentPos = numpyMap.actual_to_px(self.currentActualPos)
        if not redo_bfs:
            return True
        self.routes.clear()
        self.initialPositionIsWall = self.wall_condition(self.currentPos[0], self.currentPos[1],
                                                         numpyMap.canvas[self.currentPos[1]][self.currentPos[0]])
        self.shortest_maps = [[NodeInformation(x, y) for x in range(self.numpyMap.px_width)]
                              for y in range(self.numpyMap.px_height)]
        final_node = self.bfs_find_route(self.stopping_condition)
        if final_node is None:
            return False
        self.final_node = final_node  # debugging purpose only
        origin_coord = None
        for pixel_coord in self.backtrack_routes(final_node, self.max_num_of_compressed):
            act_x, act_y = self.numpyMap.px_to_actual_rect(pixel_coord).mid
            self.routes.appendleft((act_x, act_y, pixel_coord))
            origin_coord = pixel_coord
        self.origin_coord = origin_coord  # debugging purpose only
        return True
        # final_node = self.bfs_find_route(self.stopping_condition)
        # self.final_node = final_node  # debugging purpose only
        # origin_coord = None
        #
        # last_direction = None
        # for node_x, node_y, node_direction_to_source in self.backtrack_routes(final_node):
        #     if node_direction_to_source == last_direction:
        #         continue
        #     last_direction = node_direction_to_source
        #     pixel_coord = (node_x, node_y)
        #     act_x, act_y = self.numpyMap.px_to_actual_rect(pixel_coord).mid
        #     self.routes.appendleft((act_x, act_y, pixel_coord))
        #     origin_coord = pixel_coord
        # self.origin_coord = origin_coord  # debugging purpose only

    def get_destination(self) -> tuple:
        if len(self.routes) == 0:
            return self.move_on_to_next_destination()
        return self.routes[0]

    def move_on_to_next_destination(self):
        if len(self.routes) == 0:
            return None
        self.routes.popleft()
        return self.get_destination()

    def backtrack_routes(self, final_node: NodeInformation, maximum_compress):
        if final_node is None:
            return
        curr_node = final_node
        last_direction = False
        num_of_compressed = 0
        while True:
            is_origin_point = (curr_node.shortest_distance == 0)
            if is_origin_point:
                yield curr_node.x, curr_node.y
                return
            if curr_node.direction_to_here != last_direction or num_of_compressed >= maximum_compress:
                yield curr_node.x, curr_node.y # , curr_node.direction_to_source
                last_direction = curr_node.direction_to_here
                num_of_compressed = 0
            backtrack_direction = curr_node.direction_to_source
            dx = DIR_X[backtrack_direction]
            dy = DIR_Y[backtrack_direction]
            curr_node = self.shortest_maps[curr_node.y + dy][curr_node.x + dx]

    def bfs_find_route(self, stopping_condition, empty_value=None):
        numpyMap = self.numpyMap
        if numpyMap is None:
            return
        self.routes.clear()
        queue = BfsQueue()
        initial_origin: NodeInformation = self.shortest_maps[self.currentPos[1]][self.currentPos[0]]
        direction = South
        queue.push(direction, initial_origin)
        queue.new_distance(initializeOnly=True)  # starts from 0 distance for the origin itself
        allow_wall = self.initialPositionIsWall

        while True:
            if queue.is_current_distance_empty():
                queue.new_distance()
                queue.to_next_distance()
            curr_node = queue.pop_item_at_current_distance(direction)
            if curr_node is None:  # queue empty, no more BFS
                return empty_value
            node_info: NodeInformation = curr_node[0]
            direction: int = curr_node[1]
            distance = queue.currentDistance + (0.5 if is_diagonal(direction) else 0)  # to prioritize non-diagonal when possible
            if node_info.shortest_distance <= distance:
                continue
            if allow_wall:  # allow wall will be set to False at the first time a non-wall cell is met
                allow_wall = self.wall_condition(*node_info.xy, self.numpyMap.get_px(node_info.xy))
            node_info.set_shortest_distance(distance, direction)
            if stopping_condition(node_info):
                return node_info
            for next_dir in range(TOTAL_DIRECTION):
                dx = DIR_X[next_dir]
                dy = DIR_Y[next_dir]
                y = node_info.y + dy
                x = node_info.x + dx
                if not (0 <= x < self.numpyMap.px_width) or not (0 <= y < self.numpyMap.px_height):
                    continue
                if self.wall_condition(x, y, self.numpyMap.canvas[y][x]) and not allow_wall:
                    continue
                next_node_info: NodeInformation = self.shortest_maps[y][x]
                queue.push(next_dir, next_node_info)

    def overall_destinations(self):
        return self.routes

    def tick_to_check_if_need_replan(self, *_):
        return False

class BfsQueue:
    def __init__(self):
        self.queue = collections.deque()
        self.currentDistance = 0
        self.new_distance(initializeOnly=True)
        self.currentItem: list = self.queue.popleft()

    def pop_item_at_current_distance(self, index):
        for i in range(TOTAL_DIRECTION // 2 + 1):
            curr_direction = (index + i) % TOTAL_DIRECTION
            curr_item: list = self.currentItem[curr_direction]
            if len(curr_item) > 0:
                return curr_item.pop(), curr_direction
            curr_direction = (index - i) % TOTAL_DIRECTION
            curr_item: list = self.currentItem[curr_direction]
            if len(curr_item) > 0:
                return curr_item.pop(), curr_direction
        return None

    def to_next_distance(self):
        self.currentItem = self.queue.popleft()  # possibly error but let the caller handle it

    def push(self, index, item):
        self.lastItem[index].append(item)

    def new_distance(self, initializeOnly=False):
        self.lastItem = [[] for i in range(TOTAL_DIRECTION)]
        self.queue.append(self.lastItem)
        if not initializeOnly:
            self.currentDistance += 1

    def is_current_distance_empty(self):
        return all(map(lambda x: len(x) == 0, self.currentItem))


class NodeInformation:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.shortest_distance = float('inf')
        self.direction_to_here = None
        self.shortest_distance_initialized = False

    @property
    def direction_to_source(self):
        return INVERSE[self.direction_to_here]

    def set_shortest_distance(self, shortest_distance, direction_to_here):
        assert shortest_distance < self.shortest_distance
        self.shortest_distance = shortest_distance
        self.direction_to_here = direction_to_here
        self.shortest_distance_initialized = True

    @property
    def xy(self):
        return self.x, self.y