from planning.classic_based.ChainOfEdge import ChainOfEdge
from planning.classic_based.PreVertex import PreVertex
from planning.classic_based.Point import Point


class PathFromTarget:
    def __init__(self) -> None:
        self.visible = Point()
        self.chain = ChainOfEdge()
        self.preVt = PreVertex()