from dataclasses import dataclass
from edge import Node


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
