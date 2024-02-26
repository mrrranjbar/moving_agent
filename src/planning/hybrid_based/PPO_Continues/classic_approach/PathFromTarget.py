from PPO_Continues.classic_approach.ChainOfEdge import ChainOfEdge
from PPO_Continues.classic_approach.PreVertex import PreVertex
from PPO_Continues.classic_approach.Point import Point


class PathFromTarget:
    def __init__(self) -> None:
        self.visible = Point()
        self.chain = ChainOfEdge()
        self.preVt = PreVertex()