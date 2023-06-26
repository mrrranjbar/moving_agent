from planning.classic_based.Node import Node
from planning.classic_based.Point import Point

class LinkList:
    def __init__(self):
        self.head = None
    
    def insert(self, p: Point):
        node = Node(p)
        if self.head is None:
            self.head = node
            node.Next = self.head
            node.Prev = self.head
            return
        
        temp = self.head
        while temp.Next != self.head:
            temp = temp.Next
        temp.Next = node
        node.Next = self.head
        node.Prev = temp
        self.head.Prev = node