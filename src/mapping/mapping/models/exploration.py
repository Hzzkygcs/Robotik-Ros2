from __future__ import annotations
import abc
import collections
import random

from .direction import *
from .mapconstants import *
from .numpymap import NumpyMap


class ExplorationBase(abc.ABC):
    def __init__(self):
        pass

    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple):
        pass

    def get_destination(self):
        pass

    def move_on_to_next_destination(self):
        pass

    def overall_destinations(self):
        pass


class ExplorationSteps(ExplorationBase):
    def __init__(self, *chains):
        self.chains: list = list(chains)
        self.numpyMap = None
        self.currentActualPos = None

    # Used to update routing path but don't do it on every mapping update, only occasionally (for performance reason).
    # Will definitely update routing path if not yet intiialized
    def try_set_map(self, numpyMap: NumpyMap, currentActualPos: tuple, chance: int):
        assert 0 <= chance <= 100
        if not random.randint(0, 100) <= chance and self.numpyMap is not None:
            return
        self.set_map(numpyMap, currentActualPos)
        print(self.overall_destinations())


    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple):
        self.numpyMap = numpyMap
        self.currentActualPos = currentActualPos
        chain = self.chains[0]
        chain.set_map(numpyMap, currentActualPos)

    def get_destination(self):
        while True:
            chain: ExplorationBase = self.chains[0]
            curr_destination = chain.get_destination()
            if curr_destination is not None:
                return curr_destination
            self.chains.pop(0)  # would be better to use queue tho
            chain: ExplorationBase = self.chains[0]
            chain.set_map(self.numpyMap, self.currentActualPos)

    def move_on_to_next_destination(self):
        first_item: ExplorationBase = self.chains[0]
        result = first_item.move_on_to_next_destination()
        if result is not None:
            return
        self.chains.pop(0)
        chain: ExplorationBase = self.chains[0]
        chain.set_map(self.numpyMap, self.currentActualPos)

    def overall_destinations(self):
        return self.chains[0].overall_destinations()


class ExploreUnknownMaps(ExplorationBase):
    def __init__(self):
        self.bfs = DoBfs(self.stoppingCondition)
        self.numpyMap = None
        self.canvas = None

    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple):
        self.numpyMap = numpyMap
        self.canvas = numpyMap.canvas
        self.bfs.set_map(numpyMap, currentActualPos)

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



class BfsToDestination(ExplorationBase):
    def __init__(self, destination_actual_coordinate):
        self.bfs = DoBfs(self.stoppingCondition)
        self.numpyMap = None
        self.canvas = None
        self.destination_actual_coordinate = destination_actual_coordinate

    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple):
        self.numpyMap = numpyMap
        self.destination_px_coord = numpyMap.actual_to_px(self.destination_actual_coordinate)
        self.canvas = numpyMap.canvas
        self.bfs.set_map(numpyMap, currentActualPos)

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



class DoBfs(ExplorationBase):
    def __init__(self, stopping_condition):
        self.numpyMap = None
        self.currentActualPos = None
        self.shortest_maps = []
        self.routes = collections.deque()
        self.stopping_condition = stopping_condition

    def set_map(self, numpyMap: NumpyMap, currentActualPos: tuple):
        self.routes.clear()
        self.numpyMap = numpyMap
        self.currentActualPos = currentActualPos
        self.currentPos = numpyMap.actual_to_px(self.currentActualPos)
        self.shortest_maps = [[NodeInformation(x, y) for x in range(self.numpyMap.px_width)]
                              for y in range(self.numpyMap.px_height)]
        final_node = self.bfs_find_route(self.stopping_condition)
        self.final_node = final_node  # debugging purpose only
        for pixel_coord in self.backtrack_routes(final_node):
            act_x, act_y = self.numpyMap.px_to_actual_rect(pixel_coord).mid
            self.routes.appendleft((act_x, act_y))
            origin_coord = pixel_coord
        self.origin_coord = origin_coord  # debugging purpose only

    def get_destination(self) -> tuple:
        if len(self.routes) == 0:
            return self.move_on_to_next_destination()
        return self.routes[0]

    def move_on_to_next_destination(self):
        if len(self.routes) == 0:
            return None
        self.routes.popleft()
        return self.get_destination()

    def backtrack_routes(self, final_node: NodeInformation):
        curr_node = final_node
        while True:
            yield curr_node.x, curr_node.y
            is_origin_point = not curr_node.shortest_distance_initialized
            if is_origin_point:
                return
            backtrack_direction = curr_node.direction_to_source
            dx = DIR_X[backtrack_direction]
            dy = DIR_Y[backtrack_direction]
            curr_node = self.shortest_maps[curr_node.y+dy][curr_node.x+dx]

    def bfs_find_route(self, stopping_condition, empty_value=None):
        numpyMap = self.numpyMap
        self.routes.clear()
        if numpyMap is None:
            return
        queue = BfsQueue()
        initial_origin: NodeInformation = self.shortest_maps[self.currentPos[1]][self.currentPos[0]]
        direction = West
        queue.push(direction, initial_origin)
        queue.new_distance(initializeOnly=True)  # starts from 0 distance for the origin itself

        while True:
            if queue.is_current_distance_empty():
                queue.new_distance()
                queue.to_next_distance()
            curr_node = queue.pop_item_at_current_distance(direction)
            if curr_node is None:  # queue empty, no more BFS
                return empty_value
            node_info: NodeInformation = curr_node[0]
            direction: int = curr_node[1]
            if node_info.shortest_distance <= queue.currentDistance:
                continue
            node_info.set_shortest_distance(queue.currentDistance, direction)
            if stopping_condition(node_info):
                return node_info
            for next_dir in range(TOTAL_DIRECTION):
                dx = DIR_X[next_dir]
                dy = DIR_Y[next_dir]
                next_node_info: NodeInformation = self.shortest_maps[node_info.y+dy][node_info.x+dx]
                queue.push(next_dir, next_node_info)

    def overall_destinations(self):
        return self.routes








class BfsQueue:
    def __init__(self):
        self.queue = collections.deque()
        self.currentDistance = 0
        self.new_distance(initializeOnly=True)
        self.currentItem: list = self.queue.popleft()


    def pop_item_at_current_distance(self, index):
        for i in range(TOTAL_DIRECTION//2 + 1):
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
        return all(map(lambda x: len(x)==0, self.currentItem))

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

