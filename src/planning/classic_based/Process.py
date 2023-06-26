from planning.classic_based.LinkList import LinkList
from planning.classic_based.ChainOfEdge import ChainOfEdge
from planning.classic_based.LinkList import LinkList
from planning.classic_based.Node import Node
from planning.classic_based.PathFromTarget import PathFromTarget
from planning.classic_based.PreVertex import PreVertex
from planning.classic_based.Vertex import Vertex
from planning.classic_based.Point import Point
import time
import math
from typing import List
import sys
import copy


class Process:
    rings: LinkList = []
    source = Vertex()
    AllVertexes = []
    ObstaclePoint = []
    Paths = []
    Length = 50.0
    Alpha = 30.0
    width = 800
    hight = 600

    ############### methods ##################
    def __init__(self, Rings: List[LinkList], src):
        self.rings = Rings
        v = Vertex()
        v.point = src
        pr = PreVertex()
        pr.dist = 0
        pr.chain.points.append(src)
        pr.chain.points.append(src)
        pr.chain.id = 0
        pr.chain.minAngle = 0
        pr.chain.maxAngle = 359.99
        pr.vt = v
        v.pre.append(pr)
        self.source = v
        self.AllVertexes.append(self.source)
        for ring in self.rings:
            temp = ring.head
            while temp.Next != ring.head:
                vt = Vertex()
                vt.point = temp.op
                vt.PreObstacleVertex = temp.Prev.op
                self.ObstaclePoint.append(temp.op)
                self.AllVertexes.append(vt)
                temp = temp.Next
            vt1 = Vertex()
            vt1.point = temp.op
            vt1.PreObstacleVertex = temp.Prev.op
            self.ObstaclePoint.append(temp.op)
            self.AllVertexes.append(vt1)

    def MainLoop(self):
        startTime = time.time_ns()
        # FindVisibleVertexesFromVertex(source, ObstaclePoint)  # for test
        for v in self.AllVertexes:
            for vt in self.FindVisibleVertexesFromVertex(v):
                self.FillVertexInfo(v, vt, self.ccw(v.point, vt.point, vt.PreObstacleVertex))

        endTime = time.time_ns()
        duration = (endTime - startTime)  # divide by 1000000 to get milliseconds
        print("That took", duration / 1000000.0, "milliseconds")

    def FindVisibleVertexesFromVertex(self,vt):
        visibles = []
        for v in self.AllVertexes:
            if not v.point == self.source.point:
                if not v.point == vt.point:
                    if self.isVisible(vt, v, True) or self.isVisible(vt, v, False):
                        visibles.append(v)
                        # GUI.tempdraw.append(v.point)  # for test MRR_PY
        return visibles

    def isVisible(self, v1, v2, IsCCW):
        if not self.IntersectionWithObstacle(v1.point, v2.point):
            degree = self.IntersectionDegree(v1.point, v2.point)
            for i in range(len(v1.pre)):
                if self.angle_is_between_angles(degree, v1.pre[i].chain.minAngle, v1.pre[i].chain.maxAngle):
                    return True
        sign = 1 if IsCCW else -1
        AB = 0
        Gama = math.degrees(math.atan2(v2.point.y - v1.point.y, v2.point.x - v1.point.x))
        Teta = 0
        m = 1
        while True:
            ch = ChainOfEdge()
            Teta = 0.5 * (self.Alpha * m)
            Beta = Gama - (self.Alpha + Teta) * sign
            ResultSin = 0
            ResultCos = 0
            for i in range(1, m + 2):
                degree = (i * self.Alpha * sign) + Beta
                ResultCos += math.cos(math.radians(degree))
            try:
                AB = (v2.point.x - v1.point.x) / ResultCos
            except ZeroDivisionError:
                AB = sys.maxsize

            if AB < self.Length:
                break
            ResultCos = 0
            ResultSin = 0
            EdgeIntersect = False
            ch.points.append(v1.point)
            for i in range(1, m + 1):
                degree = (i * self.Alpha * sign) + Beta
                ResultCos += math.cos(math.radians(degree))
                ResultSin += math.sin(math.radians(degree))
                np = Point() #MRR_PY
                np.x = int((AB * ResultCos) + v1.point.x)
                np.y = int((AB * ResultSin) + v1.point.y)
                ch.points.append(np)
            ch.points.append(v2.point)
            for i in range(len(ch.points) - 1):
                if self.IntersectionWithObstacle(ch.points[i], ch.points[i + 1]):
                    EdgeIntersect = True
            if not EdgeIntersect:
                # GUI.tempCh.append(ch)  # for test MRR_PY
                degree = self.IntersectionDegree(ch.points[0], ch.points[1])
                for i in range(len(v1.pre)):
                    if self.angle_is_between_angles(degree, v1.pre[i].chain.minAngle, v1.pre[i].chain.maxAngle):
                        return True
            m += 1
        return False

    def IntersectionWithObstacle(self,p1, p2):
        for ring in self.rings:
            temp = ring.head
            while temp.Next != ring.head:
                if p2 != temp.op and p2 != temp.Next.op:
                    if self.Intersection(p1, p2, temp.op, temp.Next.op):
                        return True
                temp = temp.Next
        return False

    def FillVertexInfo(self, p1, p2, IsCCW):
        sign = 0
        if IsCCW == 1:
            sign = 1
        else:
            sign = -1
        
        Gama = math.degrees(math.atan2(p2.point.y - p1.point.y, p2.point.x - p1.point.x))
        Teta = 0
        StartDegree = 0
        m = 0
        id = 0
        finished_condition = False
        
        while True:
            Teta = 0.5 * (self.Alpha * m)
            Beta = Gama - (self.Alpha + Teta) * sign
            ResultSin = 0
            ResultCos = 0
            
            for i in range(1, m + 2):
                degree = (i * self.Alpha * sign) + Beta
                ResultCos += math.cos(math.radians(degree))
                ResultSin += math.sin(math.radians(degree))
            try:
                AB = (p2.point.x - p1.point.x + p2.point.y - p1.point.y) / (ResultCos + ResultSin)
            except ZeroDivisionError:
                AB = sys.maxsize
            
            if AB < self.Length:
                break
            
            chain_of_edge = ChainOfEdge()
            EdgeIntersect = False
            chain_of_edge.id = id
            id += 1
            chain_of_edge.points.append(p1.point)
            chain_of_edge.start = p1.point
            chain_of_edge.end = p2.point
            ResultCos = 0
            ResultSin = 0
            
            for i in range(1, m + 1):
                degree = (i * self.Alpha * sign) + Beta
                ResultCos += math.cos(math.radians(degree))
                ResultSin += math.sin(math.radians(degree))
                np = Point() # MRR_PY
                np.x = int((AB * ResultCos) + p1.point.x)
                np.y = int((AB * ResultSin) + p1.point.y)
                chain_of_edge.points.append(np)
            
            chain_of_edge.points.append(p2.point)
            
            if m == 0:
                if IsCCW == 1:
                    chain_of_edge.minAngle = self.IntersectionDegree(p1.point, p2.point)
                    StartDegree = chain_of_edge.minAngle
                    chain_of_edge.maxAngle = self.Alpha + chain_of_edge.minAngle
                    
                    if chain_of_edge.maxAngle > 360:
                        chain_of_edge.maxAngle -= 360
                else:
                    chain_of_edge.minAngle = self.IntersectionDegree(p1.point, p2.point)
                    chain_of_edge.maxAngle = chain_of_edge.minAngle - self.Alpha
                    
                    if chain_of_edge.maxAngle < 0:
                        chain_of_edge.maxAngle += 360
                    
                    temp = chain_of_edge.minAngle
                    chain_of_edge.minAngle = chain_of_edge.maxAngle
                    chain_of_edge.maxAngle = temp
                    StartDegree = chain_of_edge.maxAngle
            else:
                if IsCCW == 1:
                    chain_of_edge.minAngle = Teta + StartDegree + (self.Alpha / 2)
                    
                    if chain_of_edge.minAngle > 360:
                        chain_of_edge.minAngle -= 360
                    
                    chain_of_edge.maxAngle = (self.Alpha / 2) + chain_of_edge.minAngle
                    
                    if chain_of_edge.maxAngle > 360:
                        chain_of_edge.maxAngle -= 360
                else:
                    chain_of_edge.minAngle = StartDegree - Teta - (self.Alpha / 2)
                    
                    if chain_of_edge.minAngle < 0:
                        chain_of_edge.minAngle += 360
                    
                    chain_of_edge.maxAngle = chain_of_edge.minAngle - (self.Alpha / 2)
                    
                    if chain_of_edge.maxAngle < 0:
                        chain_of_edge.maxAngle += 360
                    
                    temp = chain_of_edge.minAngle
                    chain_of_edge.minAngle = chain_of_edge.maxAngle
                    chain_of_edge.maxAngle = temp
                
                degree = self.IntersectionDegree(p2.point, p2.PreObstacleVertex)
                
                if self.angle_is_between_angles(degree, chain_of_edge.minAngle, chain_of_edge.maxAngle):
                    if IsCCW == 1:
                        chain_of_edge.maxAngle = degree
                    else:
                        chain_of_edge.minAngle = degree
                    finished_condition = True
            
            for i in range(len(chain_of_edge.points) - 1):
                if self.IntersectionWithObstacle(chain_of_edge.points[i], chain_of_edge.points[i + 1]):
                    EdgeIntersect = True
            
            if not EdgeIntersect:
                degree = self.IntersectionDegree(chain_of_edge.points[0], chain_of_edge.points[1])
                
                for i in range(len(p1.pre)):
                    if chain_of_edge.minAngle != chain_of_edge.maxAngle and self.angle_is_between_angles(degree, p1.pre[i].chain.minAngle, p1.pre[i].chain.maxAngle):
                        self.Relax(p1, p2, chain_of_edge)
            
            if finished_condition:
                break
            
            m += 1
        
        for vt in self.AllVertexes:
            if vt.point == p2.point:
                vt.PreObstacleVertex = p2.PreObstacleVertex
                vt.point = p2.point
                vt.pre = p2.pre
                break

    def Intersection(self, line1V1, line1V2, line2V1, line2V2):
        if (
            (self.ccw(line1V1, line1V2, line2V1) * self.ccw(line1V1, line1V2, line2V2) <= 0)
            and (self.ccw(line2V1, line2V2, line1V1) * self.ccw(line2V1, line2V2, line1V2) <= 0)
        ):
            return True
        else:
            return False

    def ccw(self, p, q, r):
        if self.turn(p, q, r) > 0:
            return 1
        else:
            return -1

    def turn(self, p, q, r):
        result = (r.x - q.x) * (p.y - q.y) - (r.y - q.y) * (p.x - q.x)
        if result < 0:
            return -1  # P->Q->R is a right turn
        if result > 0:
            return 1  # P->Q->R is a left turn
        return 0  # P->Q->R is a straight line, i.e. P, Q, R are collinear

    def IntersectionDegree(self, p1, p2):
        # Calculating degree between p1 & p2 , zero is in p1
        degree = math.degrees(math.atan2(p2.y - p1.y, p2.x - p1.x))
        if degree < 0:
            degree += 360
        return degree

    def angle_1to360(self, angle):
        angle = ((int(angle) % 360) + (angle - int(angle)))  # converts angle to range -360 + 360
        if angle > 0.0:
            return angle
        else:
            return angle + 360.0

    def angle_is_between_angles(self, N, a, b):
        N = self.angle_1to360(N)  # normalize angles to be 1-360 degrees
        a = self.angle_1to360(a)
        b = self.angle_1to360(b)

        if a < b:
            return a <= N <= b
        return a <= N or N <= b

    def add_chain_to_path(self, chain, pre_path, path):
        for i in range(len(pre_path.points)):
            path.points.append(pre_path.points[i])
        for i in range(len(chain.points)):
            path.points.append(chain.points[i])

    def isBetweenInterval(self, pv, chain):
        # max [()] | , [()|] , [(|)] , [|()]  min
        if (
            (
                (pv.chain.maxAngle > pv.chain.minAngle)
                and (chain.maxAngle > chain.minAngle)
                and (pv.chain.maxAngle >= chain.maxAngle)
                and (chain.maxAngle >= pv.chain.minAngle)
                and (pv.chain.maxAngle >= chain.minAngle)
                and (chain.minAngle >= pv.chain.minAngle)
            )
            or (
                (pv.chain.maxAngle < pv.chain.minAngle)
                and (chain.maxAngle > chain.minAngle)
                and (pv.chain.maxAngle >= chain.maxAngle)
                and (chain.maxAngle <= pv.chain.minAngle)
                and (pv.chain.maxAngle >= chain.minAngle)
                and (chain.minAngle <= pv.chain.minAngle)
            )
            or (
                (pv.chain.maxAngle < pv.chain.minAngle)
                and (chain.maxAngle > chain.minAngle)
                and (pv.chain.maxAngle >= chain.maxAngle)
                and (chain.maxAngle <= pv.chain.minAngle)
                and (pv.chain.maxAngle <= chain.minAngle)
                and (chain.minAngle >= pv.chain.minAngle)
            )
            or (
                (pv.chain.maxAngle < pv.chain.minAngle)
                and (chain.maxAngle > chain.minAngle)
                and (pv.chain.maxAngle <= chain.maxAngle)
                and (chain.maxAngle >= pv.chain.minAngle)
                and (pv.chain.maxAngle <= chain.minAngle)
                and (chain.minAngle >= pv.chain.minAngle)
            )
        ):
            return True
        # max ([])| , ([]|) , ([|]) , (|[])  min
        elif ( 
            (
                (chain.maxAngle > chain.minAngle)
                and (pv.chain.maxAngle > pv.chain.minAngle)
                and (chain.maxAngle >= pv.chain.maxAngle)
                and (pv.chain.maxAngle >= chain.minAngle)
                and (chain.maxAngle >= pv.chain.minAngle)
                and (pv.chain.minAngle >= chain.minAngle)
            )
            or (
                (chain.maxAngle < chain.minAngle)
                and (pv.chain.maxAngle > pv.chain.minAngle)
                and (chain.maxAngle >= pv.chain.maxAngle)
                and (pv.chain.maxAngle <= chain.minAngle)
                and (chain.maxAngle >= pv.chain.minAngle)
                and (pv.chain.minAngle <= chain.minAngle)
            )
            or (
                (chain.maxAngle < chain.minAngle)
                and (pv.chain.maxAngle < pv.chain.minAngle)
                and (chain.maxAngle >= pv.chain.maxAngle)
                and (pv.chain.maxAngle <= chain.minAngle)
                and (chain.maxAngle <= pv.chain.minAngle)
                and (pv.chain.minAngle >= chain.minAngle)
            )
            or (
                (chain.maxAngle < chain.minAngle)
                and (pv.chain.maxAngle > pv.chain.minAngle)
                and (chain.maxAngle <= pv.chain.maxAngle)
                and (pv.chain.maxAngle >= chain.minAngle)
                and (chain.maxAngle <= pv.chain.minAngle)
                and (pv.chain.minAngle >= chain.minAngle)
            )
        ):
            return True
        # max [ ( ] |   ,   [ ( | ]  ,  [ | ( ] min
        elif (
            (pv.chain.maxAngle > pv.chain.minAngle)
            and (pv.chain.maxAngle >= chain.maxAngle)
            and (chain.maxAngle >= pv.chain.minAngle)
        ) or (
            (pv.chain.maxAngle < pv.chain.minAngle)
            and (pv.chain.maxAngle >= chain.maxAngle)
            and (chain.maxAngle <= pv.chain.minAngle)
        ) or (
            (pv.chain.maxAngle < pv.chain.minAngle)
            and (pv.chain.maxAngle <= chain.maxAngle)
            and (chain.maxAngle >= pv.chain.minAngle)
        ):
            return True
        # max [ ) ] |   ,   [ ) | ]  ,  [ | ) ] min
        elif (
            (pv.chain.maxAngle > pv.chain.minAngle)
            and (pv.chain.maxAngle >= chain.minAngle)
            and (chain.minAngle >= pv.chain.minAngle)
        ) or (
            (pv.chain.maxAngle < pv.chain.minAngle)
            and (pv.chain.maxAngle >= chain.minAngle)
            and (chain.minAngle <= pv.chain.minAngle)
        ) or (
            (pv.chain.maxAngle < pv.chain.minAngle)
            and (pv.chain.maxAngle <= chain.minAngle)
            and (chain.minAngle >= pv.chain.minAngle)
        ):
            return True
        return False

    def Relax(self, v1, v2, chain: ChainOfEdge):
        distance = 0
        is_empty_again = False
        pv_v1_path : ChainOfEdge = None
        chain_minAngle = chain.minAngle
        chain_maxAngle = chain.maxAngle
        pre_temps = []
        pre_temps_tmp = []
        degree = self.IntersectionDegree(chain.points[0], chain.points[1])
        for pv_v1 in v1.pre:
            if self.angle_is_between_angles(degree, pv_v1.chain.minAngle, pv_v1.chain.maxAngle):
                distance = pv_v1.dist
                pv_v1_path = pv_v1.path
                break
        distance += chain.Length()
        # finding interval of v2.pre that match by minAngle & maxAngle of chain
        for pv in v2.pre:
            if self.isBetweenInterval(pv, chain):
                pre_temps.append(pv)
        if len(pre_temps) == 0:
            pv_temp = PreVertex()
            pv_temp.dist = distance
            pv_temp.chain = copy.deepcopy(chain)
            # pv_temp.chain.id = chain.id
            # pv_temp.chain.end = chain.end
            # pv_temp.chain.maxAngle = chain.maxAngle
            # pv_temp.chain.minAngle = chain.minAngle
            # pv_temp.chain.points = chain.points
            # pv_temp.chain.start = chain.start
            pv_temp.vt = v1
            self.add_chain_to_path(chain, pv_v1_path, pv_temp.path)
            if pv_temp.chain.maxAngle != pv_temp.chain.minAngle:
                v2.pre.append(pv_temp)
        else:
            is_empty_again = True
            pre_temps_tmp.clear()
            for pv in pre_temps:
                pre_temps_tmp.append(pv)
            ch_min_changed = -1
            ch_max_changed = -1
            for pv in pre_temps_tmp:
                if ((pv.chain.maxAngle > pv.chain.minAngle) and (chain.maxAngle > chain.minAngle)
                        and (pv.chain.maxAngle >= chain.maxAngle) and (chain.maxAngle >= pv.chain.minAngle)
                        and (pv.chain.maxAngle >= chain.minAngle) and (chain.minAngle >= pv.chain.minAngle)) or \
                        ((pv.chain.maxAngle < pv.chain.minAngle) and (chain.maxAngle > chain.minAngle)
                        and (pv.chain.maxAngle >= chain.maxAngle) and (chain.maxAngle <= pv.chain.minAngle)
                        and (pv.chain.maxAngle >= chain.minAngle) and (chain.minAngle <= pv.chain.minAngle)) or \
                        ((pv.chain.maxAngle < pv.chain.minAngle) and (chain.maxAngle < chain.minAngle)
                        and (pv.chain.maxAngle >= chain.maxAngle) and (chain.maxAngle <= pv.chain.minAngle)
                        and (pv.chain.maxAngle <= chain.minAngle) and (chain.minAngle >= pv.chain.minAngle)) or \
                        ((pv.chain.maxAngle < pv.chain.minAngle) and (chain.maxAngle > chain.minAngle)
                        and (pv.chain.maxAngle <= chain.maxAngle) and (chain.maxAngle >= pv.chain.minAngle)
                        and (pv.chain.maxAngle <= chain.minAngle) and (chain.minAngle >= pv.chain.minAngle)):
                    if distance < pv.dist:
                        pv_temp1 = PreVertex()
                        pv_temp1.dist = distance
                        pv_temp1.chain = copy.deepcopy(chain)
                        # pv_temp1.chain.id = chain.id
                        # pv_temp1.chain.end = chain.end
                        pv_temp1.chain.maxAngle = chain_maxAngle
                        pv_temp1.chain.minAngle = chain_minAngle
                        # pv_temp1.chain.points = chain.points
                        # pv_temp1.chain.start = chain.start
                        pv_temp1.vt = v1
                        self.add_chain_to_path(chain, pv_v1_path, pv_temp1.path)
                        if pv_temp1.chain.maxAngle != pv_temp1.chain.minAngle:
                            v2.pre.append(pv_temp1)
                        pv_temp2 = PreVertex()
                        pv_temp2.dist = distance
                        pv_temp2.chain = copy.deepcopy(chain)
                        # pv_temp2.chain.id = chain.id
                        # pv_temp2.chain.end = chain.end
                        pv_temp2.chain.maxAngle = chain_maxAngle
                        pv_temp2.chain.minAngle = chain_minAngle
                        # pv_temp2.chain.points = chain.points
                        # pv_temp2.chain.start = chain.start
                        pv_temp2.vt = v1
                        pv_temp2.chain.minAngle = v2.pre[self.get_index_of_obj_in_list(v2.pre,pv)].chain.minAngle
                        pv_temp2.chain.maxAngle = chain_minAngle
                        self.add_chain_to_path(chain, pv_v1_path, pv_temp2.path)
                        if pv_temp2.chain.maxAngle != pv_temp2.chain.minAngle:
                            v2.pre.append(pv_temp2)
                        v2.pre[self.get_index_of_obj_in_list(v2.pre,pv)].chain.minAngle = chain_maxAngle
                        # pre_temps.remove(pv)
                        pre_temps.pop(self.get_index_of_obj_in_list(pre_temps,pv))
                        ch_max_changed = ch_min_changed = chain.maxAngle
                        if v2.pre[self.get_index_of_obj_in_list(v2.pre,pv)].chain.minAngle == v2.pre[self.get_index_of_obj_in_list(v2.pre,pv)].chain.maxAngle:
                            # v2.pre.remove(pv)
                            v2.pre.pop(self.get_index_of_obj_in_list(v2.pre, pv))
                    else:
                        ch_max_changed = ch_min_changed = chain.maxAngle
                        # pre_temps.remove(pv)
                        pre_temps.pop(self.get_index_of_obj_in_list(pre_temps,pv))
                elif ((chain.maxAngle > chain.minAngle) and (pv.chain.maxAngle > pv.chain.minAngle)
                        and (chain.maxAngle >= pv.chain.maxAngle) and (pv.chain.maxAngle >= chain.minAngle)
                        and (chain.maxAngle >= pv.chain.minAngle) and (pv.chain.minAngle >= chain.minAngle)) or \
                        ((chain.maxAngle < chain.minAngle) and (pv.chain.maxAngle > pv.chain.minAngle)
                        and (chain.maxAngle >= pv.chain.maxAngle) and (pv.chain.maxAngle <= chain.minAngle)
                        and (chain.maxAngle >= pv.chain.minAngle) and (pv.chain.minAngle <= chain.minAngle)) or \
                        ((chain.maxAngle < chain.minAngle) and (pv.chain.maxAngle < pv.chain.minAngle)
                        and (chain.maxAngle >= pv.chain.maxAngle) and (pv.chain.maxAngle <= chain.minAngle)
                        and (chain.maxAngle <= pv.chain.minAngle) and (pv.chain.minAngle >= chain.minAngle)) or \
                        ((chain.maxAngle < chain.minAngle) and (pv.chain.maxAngle > pv.chain.minAngle)
                        and (chain.maxAngle <= pv.chain.maxAngle) and (pv.chain.maxAngle >= chain.minAngle)
                        and (chain.maxAngle <= pv.chain.minAngle) and (pv.chain.minAngle >= chain.minAngle)):
                    if distance < pv.dist:
                        # v2.pre.remove(pv)
                        v2.pre.pop(self.get_index_of_obj_in_list(v2.pre, pv))
                        # pre_temps.remove(pv)
                        pre_temps.pop(self.get_index_of_obj_in_list(pre_temps,pv))
                elif ((pv.chain.maxAngle > pv.chain.minAngle) and (pv.chain.maxAngle >= chain.maxAngle)
                        and (chain.maxAngle >= pv.chain.minAngle)) or \
                        ((pv.chain.maxAngle < pv.chain.minAngle) and (pv.chain.maxAngle >= chain.maxAngle)
                        and (chain.maxAngle <= pv.chain.minAngle)) or \
                        ((pv.chain.maxAngle < pv.chain.minAngle) and (pv.chain.maxAngle <= chain.maxAngle)
                        and (chain.maxAngle >= pv.chain.minAngle)):
                    if distance < pv.dist:
                        v2.pre[self.get_index_of_obj_in_list(v2.pre,pv)].chain.minAngle = chain.maxAngle
                        # pre_temps.remove(pv)
                        pre_temps.pop(self.get_index_of_obj_in_list(pre_temps,pv))
                    else:
                        if ch_max_changed != chain.maxAngle:
                            ch_max_changed = pv.chain.minAngle
                        # pre_temps.remove(pv)
                        pre_temps.pop(self.get_index_of_obj_in_list(pre_temps,pv))
                elif ((pv.chain.maxAngle > pv.chain.minAngle) and (pv.chain.maxAngle >= chain.minAngle)
                        and (chain.minAngle >= pv.chain.minAngle)) or \
                        ((pv.chain.maxAngle < pv.chain.minAngle) and (pv.chain.maxAngle >= chain.minAngle)
                        and (chain.minAngle <= pv.chain.minAngle)) or \
                        ((pv.chain.maxAngle < pv.chain.minAngle) and (pv.chain.maxAngle <= chain.minAngle)
                        and (chain.minAngle >= pv.chain.minAngle)):
                    if distance < pv.dist:
                        v2.pre[self.get_index_of_obj_in_list(v2.pre,pv)].chain.maxAngle = chain.minAngle
                        # pre_temps.remove(pv)
                        pre_temps.pop(self.get_index_of_obj_in_list(pre_temps,pv))
                    else:
                        if ch_min_changed != chain.maxAngle:
                            ch_min_changed = pv.chain.maxAngle
                        # pre_temps.remove(pv)
                        pre_temps.pop(self.get_index_of_obj_in_list(pre_temps,pv))

            if ch_max_changed != -1:
                chain_maxAngle = ch_max_changed
            if ch_min_changed != -1:
                chain_minAngle = ch_min_changed



        if  pre_temps:
            # is_empty_again = True
            pre_temps_tmp = pre_temps[:]
            pre_temps_tmp.sort(key=lambda pv: pv.chain.minAngle)
            for i in range(len(pre_temps_tmp) - 1):
                pv_temp = PreVertex()
                pv_temp.dist = distance
                pv_temp.chain = copy.deepcopy(chain)
                # pv_temp.chain.id = chain.id
                # pv_temp.chain.end = chain.end
                pv_temp.chain.maxAngle = chain_maxAngle
                pv_temp.chain.minAngle = chain_minAngle
                # pv_temp.chain.points = chain.points
                # pv_temp.chain.start = chain.start
                pv_temp.vt = v1
                pv_temp.chain.minAngle = pre_temps_tmp[i].chain.maxAngle
                pv_temp.chain.maxAngle = pre_temps_tmp[i + 1].chain.minAngle
                self.add_chain_to_path(chain, pv_v1_path, pv_temp.path)
                if pv_temp.chain.maxAngle != pv_temp.chain.minAngle:
                    v2.pre.append(pv_temp)

            pv_temp1 = PreVertex()
            pv_temp1.dist = distance
            pv_temp1.chain = copy.deepcopy(chain)
            # pv_temp1.chain.id = chain.id
            # pv_temp1.chain.end = chain.end
            pv_temp1.chain.maxAngle = chain_maxAngle
            pv_temp1.chain.minAngle = chain_minAngle
            # pv_temp1.chain.points = chain.points
            # pv_temp1.chain.start = chain.start
            pv_temp1.vt = v1
            pv_temp1.chain.maxAngle = pre_temps_tmp[0].chain.minAngle
            self.add_chain_to_path(chain, pv_v1_path, pv_temp1.path)
            if pv_temp1.chain.maxAngle != pv_temp1.chain.minAngle:
                v2.pre.append(pv_temp1)

            pv_temp2 = PreVertex()
            pv_temp2.dist = distance
            pv_temp2.chain = copy.deepcopy(chain)
            # pv_temp2.chain.id = chain.id
            # pv_temp2.chain.end = chain.end
            pv_temp2.chain.maxAngle = chain_maxAngle
            pv_temp2.chain.minAngle = chain_minAngle
            # pv_temp2.chain.points = chain.points
            # pv_temp2.chain.start = chain.start
            pv_temp2.vt = v1
            pv_temp2.chain.minAngle = pre_temps_tmp[-1].chain.maxAngle
            pv_temp2.chain.maxAngle = chain_maxAngle
            self.add_chain_to_path(chain, pv_v1_path, pv_temp2.path)
            if pv_temp2.chain.maxAngle != pv_temp2.chain.minAngle:
                v2.pre.append(pv_temp2)
        elif is_empty_again and not pre_temps and chain_maxAngle != chain_minAngle:
            pv_temp = PreVertex()
            pv_temp.dist = distance
            pv_temp.chain = copy.deepcopy(chain)
            # pv_temp.chain = chain
            pv_temp.chain.minAngle = chain_minAngle
            pv_temp.chain.maxAngle = chain_maxAngle
            pv_temp.vt = v1
            self.add_chain_to_path(chain, pv_v1_path, pv_temp.path)
            v2.pre.append(pv_temp)

    #####target section##########
    def GetChainsFromTargetToVertexes(self, p1):
        sign = 1
        id = 0
        Info = []
        for k in range(2):
            for p2 in self.AllVertexes:
                if not self.IntersectionWithObstacle(p1, p2.point):
                    degree = self.IntersectionDegree(p2.point, p1)
                    for i in range(len(p2.pre)):
                        if self.angle_is_between_angles(degree, p2.pre[i].chain.minAngle, p2.pre[i].chain.maxAngle):
                            tmp_chain = ChainOfEdge()
                            tmp_chain.points.append(p1)
                            tmp_chain.points.append(p2.point)
                            tmp_p = PathFromTarget()
                            tmp_p.chain = tmp_chain
                            tmp_p.visible = p2.point
                            tmp_p.preVt = p2.pre[i]
                            Info.append(tmp_p)
                            break

                AB = 0
                Gama = math.degrees(math.atan2(p2.point.y - p1.y, p2.point.x - p1.x))
                Teta = 0
                m = 1
                while True:
                    Teta = 0.5 * (self.Alpha * m)
                    Beta = Gama - (self.Alpha + Teta) * sign
                    ResultSin = 0
                    ResultCos = 0
                    for i in range(1, m + 2):
                        degree = (i * self.Alpha * sign) + Beta
                        ResultCos += math.cos(math.radians(degree))
                    try:
                        AB = (p2.point.x - p1.x) / ResultCos
                    except ZeroDivisionError:
                        AB = sys.maxsize

                    if AB < self.Length:
                        break

                    chain_of_edge = ChainOfEdge()
                    EdgeIntersect = False
                    chain_of_edge.id = id
                    id += 1
                    chain_of_edge.points.append(p1)
                    chain_of_edge.start = p1
                    chain_of_edge.end = p2.point
                    ResultCos = 0
                    ResultSin = 0
                    for i in range(1, m + 1):
                        degree = (i * self.Alpha * sign) + Beta
                        ResultCos += math.cos(math.radians(degree))
                        ResultSin += math.sin(math.radians(degree))
                        np = Point()
                        np.x = int((AB * ResultCos) + p1.x)
                        np.y = int((AB * ResultSin) + p1.y)
                        chain_of_edge.points.append(np)
                    chain_of_edge.points.append(p2.point)
                    for i in range(len(chain_of_edge.points) - 1):
                        if self.IntersectionWithObstacle(chain_of_edge.points[i], chain_of_edge.points[i + 1]):
                            EdgeIntersect = True
                    if not EdgeIntersect:
                        degree = self.IntersectionDegree(chain_of_edge.points[-1], chain_of_edge.points[-2])
                        for i in range(len(p2.pre)):
                            if self.angle_is_between_angles(degree, p2.pre[i].chain.minAngle, p2.pre[i].chain.maxAngle):
                                tmp = PathFromTarget()
                                tmp.chain = chain_of_edge
                                tmp.visible = p2.point
                                tmp.preVt = p2.pre[i]
                                Info.append(tmp)
                                break
                    m += 1

            sign = -1

        return Info

    def MainFunction(self, target):
        start_time = time.time()

        shortest_path = []
        paths = self.GetChainsFromTargetToVertexes(target)

        if paths:
            min_path = PathFromTarget()
            for path in paths:
                if min_path is None or (path.chain.Length() + path.preVt.dist < min_path.chain.Length() + min_path.preVt.dist):
                    min_path = path
            for pt in min_path.preVt.path.points:
                shortest_path.append(pt)
            for i in range(len(min_path.chain.points) - 1, -1, -1):
                shortest_path.append(min_path.chain.points[i])

        end_time = time.time()
        duration = (end_time - start_time) * 1000  # convert to milliseconds
        print("Query Time is:", duration, "milliseconds")

        length1 = 0
        for i in range(len(shortest_path) - 1):
            length1 += math.sqrt((shortest_path[i + 1].x - shortest_path[i].x) ** 2 + (shortest_path[i + 1].y - shortest_path[i].y) ** 2)
        print("Shortest Path Length is:", length1)


        filtered_points = []
        prev_point = None

        for point in shortest_path:
            if point != prev_point:
                filtered_points.append(point)
            prev_point = point

        return filtered_points
    
    def get_index_of_obj_in_list(self, lst, target_obj):
        for i in range(len(lst)):
            if(id(target_obj) == id(lst[i])):
                return i
        return -1