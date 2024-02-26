from dataclasses import dataclass
# from PreVertex import PreVertex
from typing import List
from PPO_Continues.classic_approach.Point import Point

@dataclass
class Vertex:
    def __init__(self):
        self.point = Point()
        self.PreObstacleVertex = Point()
        self.pre = []
        self.pre.clear()
