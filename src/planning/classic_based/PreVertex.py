from planning.classic_based.Vertex import Vertex
from planning.classic_based.ChainOfEdge import ChainOfEdge
from dataclasses import dataclass

@dataclass
class PreVertex:
    def __init__(self):
        self.chain = ChainOfEdge()
        self.path = ChainOfEdge()
        self.dist=100000000
        self.vt = Vertex()