from typing import Optional
from enum import Enum
from dataclasses import dataclass, replace


class NodeType(Enum):
    BLANK = 0
    START = 1
    END = 2
    OBSTACLE = 3

    @staticmethod
    def from_value(value: int) -> 'NodeType':
        maxValue = max([e.value for e in NodeType])
        if value > maxValue:
            return NodeType(maxValue)
        return NodeType(value)


@dataclass(frozen=True, eq=False)
class Node:
    x: int
    y: int
    node_type: NodeType
    edge_to_parent: Optional['Edge'] = None

    @staticmethod
    def empty() -> 'Node':
        """Creates an empty node.

        :return: An emty node.
        """
        return Node(0, 0, NodeType(0))

    def is_start(self) -> bool:
        """Checks whether this node is a START.

        :return: True if its a START node, else False.
        """
        if self.node_type == NodeType.START:
            return True
        return False

    def is_end(self) -> bool:
        """Checks whether this node is an END.

        :return: True if its an END node, else False.
        """
        if self.node_type == NodeType.END:
            return True
        return False

    def is_obstacle(self) -> bool:
        """Checks whether this node is an OBSTACLE.

        :return: True if its an OBSTACLE node, else False.
        """
        if self.node_type == NodeType.OBSTACLE:
            return True
        return False

    def has_edge_to_parent(self) -> bool:
        """Checks whether an edge to a parent was set.

        :return: True if an edge to a parent was set, else False.
        """
        if self.edge_to_parent is None:
            return False
        return True

    def set_edge_to_parent(self, edge:  Optional['Edge']) -> 'Node':
        return replace(self, edge_to_parent=edge)

    def __eq__(self, other):
        if not isinstance(other, Node):
            return NotImplemented
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


@dataclass(frozen=True)
class Edge:
    target: 'Node'
    g: float
    h: float
    is_diagonal: bool

    def get_f(self) -> float:
        """The overall score. The lesser, the better!

        :return: g + h.
        """
        return self.g + self.h
