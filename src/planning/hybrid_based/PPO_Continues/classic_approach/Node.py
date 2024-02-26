from PPO_Continues.classic_approach.Point import Point

class Node:
    def __init__(self, pt: Point):
        self.op : Point = pt
        self.Next : Node = None
        self.Prev: Node = None