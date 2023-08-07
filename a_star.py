from typing import List, Optional, Tuple
from node import Node, Edge, NodeType
import math


class AStar:
    """Searches a path from a start point to an end point in a 2D array without crossing obstacles using the A* algorithm and the euclidean distance.

    2D array elements:
    0 = blank fields
    1 = start (needed)
    2 = end (needed)
    3 = obstacles

    Example:
    ```python

    # init
    a_star = AStar([
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 3, 0, 0, 0],
        [0, 1, 0, 3, 0, 2, 0],
        [0, 0, 0, 3, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0]
    ])

    # calculate path
    a_star.search()

    # print path
    print(a_star.get_result()) # [(2, 2), (3, 2), (4, 2), (4, 3), (4, 4), (3, 4), (3, 5)]
    ```
    """

    # The initial maze layout converted into the internal node structure.
    __layout: List[List[Node]] = None

    # The start point (number code = 3).
    __node_start: Node = None
    # The end point (number code = 2).
    __node_end: Node = None

    # Nodes that have to be considered for the final path, e.g. neighbors of start and have to be checked further.
    __open_list: List[Node] = []

    # Nodes that have been in openList previously but are now done with processing. These dont have to be checked again.
    __closed_list: List[Node] = []

    def __init__(self, arr2d: List[List[int]]):
        xLen, yLen = (len(arr2d), len(arr2d[0]))
        self.__layout = [[Node.empty() for _ in range(yLen)]
                         for _ in range(xLen)]

        for x in range(xLen):
            for y in range(yLen):
                node = Node(x, y, NodeType(NodeType.from_value(arr2d[x][y])))
                self.__layout[x][y] = node
                if node.is_start():
                    self.__node_start = node
                if node.is_end():
                    self.__node_end = node

        if self.__node_start is None:
            raise InvalidLayoutInitializationException('No start found.')
        if self.__node_end is None:
            raise InvalidLayoutInitializationException('No end found.')

    def get_result(self) -> List[Tuple[int, int]]:
        """Gets the resulting path from start to end.

        :return: A list of coordinates which represents the path between start and end.
        :raises PathNotFoundException: If no path could be found.
        """
        if not self.__node_end.has_edge_to_parent():
            raise PathNotFoundException('Call search first.')

        result: List[Tuple[int, int]] = list()
        parent = self.__node_end.edge_to_parent.target

        # working backwards from the target square until you reach the starting square
        while parent.has_edge_to_parent() and not parent.is_start():
            result.append((parent.x, parent.y))
            parent = parent.edge_to_parent.target

        # order start to end
        result.reverse()
        return result

    def get_len_x(self) -> int:
        return len(self.__layout)

    def get_len_y(self) -> int:
        return len(self.__layout[0])

    def search(self) -> bool:
        self.__node_end.set_edge_to_parent(None)
        self.__open_list.clear()
        self.__closed_list.clear()
        self.__open_list.append(self.__node_start)
        return self.__do_search()

    def __do_search(self) -> bool:
        if len(self.__open_list) <= 0:
            # nothing todo and no path was found
            return False

        # find lowest F score in openList
        current: Node = min(
            self.__open_list, key=lambda x: self.__get_f_or_zero_if_start(x))

        # switch lowest F to the closed list
        self.__open_list.remove(current)
        self.__closed_list.append(current)

        if current.is_end():
            # we have found the end node
            # update the member end
            self.__node_end = current
            return True

        for (neighbour, is_diagonal) in self.__get_neighbours(current):
            # We dont want obstacles and closed neighbours.
            if neighbour.is_obstacle() or neighbour in self.__closed_list:
                continue

            neighbour_with_edge = neighbour.set_edge_to_parent(Edge(current, self.__calc_g(
                current, is_diagonal), self.__calc_h(neighbour), is_diagonal))

            if neighbour_with_edge not in self.__open_list:
                self.__open_list.append(neighbour_with_edge)
            else:
                # We already have this neighbour in list.
                index_of_other = self.__open_list.index(neighbour_with_edge)
                # Is this path (with current as parent) better than the on already in the list?
                if neighbour_with_edge.edge_to_parent.g < self.__open_list[index_of_other].edge_to_parent.g:
                    # Yes, the path for the current parent is better!
                    # Change it.
                    self.__open_list[index_of_other] = neighbour_with_edge

        return self.__do_search()

    def __get_f_or_zero_if_start(self, node: Node) -> float:
        if node.node_type == NodeType.START:
            return 0.0
        return node.edge_to_parent.get_f()

    def __calc_g(self, parent: Node, is_diagonal: bool):
        """Calculates the G cost for an edge based on the parent edge G cost.

        :param parent: The parent of the edge we want to calc the G cost for.
        :param is_diagonal: Whether the edge we want to calcalate the G cost for has a diagonal or orthogonal neighbourship relation to its parent.
        :return: The G cost.
        """
        # Note: Since the start has no parent we are going to return 0.
        if not parent.has_edge_to_parent() and parent.node_type == NodeType.START:
            return 0
        elif not parent.has_edge_to_parent():
            raise Exception('No edge found.')

        # Define how the path looks like by setting weights for orthogonal and diagonal edges.
        # https://stackoverflow.com/questions/36493642/a-pathfinding-calculating-g-cost
        value: float = 10.0
        if is_diagonal:
            value = 14.0

        return parent.edge_to_parent.g + value

    def __calc_h(self, node: Node) -> float:
        """Euclidean distance (h) to the end point.

        :param node: Start point.
        :return: The Distance.
        """
        dx = self.__node_end.x - node.x
        dy = self.__node_end.y - node.y
        return math.sqrt((dx * dx) + (dy * dy))

    def __get_neighbours(self, node: Node) -> List[Tuple[Node, bool]]:
        """Gets all neighbours around a specific node.

        :param node: The node to get the naighbours for.
        :return: The list of neighbours.
        """

        result: List[Tuple[Node, bool]] = list()
        aligned_coordinates: List[Tuple[int, int, bool]] = list([
            (node.x - 1, node.y + 1, True),  # TL
            (node.x, node.y + 1, False),  # TC
            (node.x + 1, node.y + 1, True),  # TR
            (node.x + 1, node.y, False),  # CR
            (node.x + 1, node.y - 1, True),  # BR
            (node.x, node.y - 1, False),  # BC
            (node.x - 1, node.y - 1, True),  # BL
            (node.x - 1, node.y, False),  # CL

        ])

        for element in aligned_coordinates:
            (x, y, a) = element
            neighbour = self.__get_node_or_none(x, y)
            if neighbour != None:
                result.append((neighbour, a))

        return result

    def __get_node_or_none(self, x: int, y: int) -> Optional[Node]:
        if self.__is_in_bounds(x, y):
            return self.__layout[x][y]
        return None

    def __is_in_bounds(self, x: int, y: int) -> bool:
        if x < 0 or x > self.get_len_x() - 1 or y < 0 or y > self.get_len_y() - 1:
            return False
        return True


class InvalidLayoutInitializationException(Exception):
    "Raised when the initial maze doesnt follow the rules."
    pass


class PathNotFoundException(Exception):
    "Raised when there was no connection found from start to end."
    pass
