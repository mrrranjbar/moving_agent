from math import sqrt
from typing import List
from PPO_Continues.classic_approach.Point import Point

class ChainOfEdge:
    def __init__(self):
        self.id = 0
        self.minAngle = 0.0
        self.maxAngle = 0.0
        self.points : List[Point] = []
        self.points.clear()
        self.start = Point()
        self.end = Point()

    def Length(self):
        length = 0.0
        for i in range(len(self.points) - 1):
            length += sqrt((self.points[i + 1].x - self.points[i].x) ** 2 + (self.points[i + 1].y - self.points[i].y) ** 2)
        return length
