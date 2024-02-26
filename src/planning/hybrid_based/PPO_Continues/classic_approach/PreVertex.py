from PPO_Continues.classic_approach.Vertex import Vertex
from PPO_Continues.classic_approach.ChainOfEdge import ChainOfEdge
from dataclasses import dataclass

@dataclass
class PreVertex:
    def __init__(self):
        self.chain = ChainOfEdge()
        self.path = ChainOfEdge()
        self.dist=100000000
        self.vt = Vertex()