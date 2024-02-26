import os
# import exceptiongroup
from PPO_Continues.classic_approach.Process import Process
from typing import List
from PPO_Continues.classic_approach.LinkList import LinkList
from PPO_Continues.classic_approach.Point import Point
# import matplotlib.pyplot as plt

class Preparation:
    def __init__(self):
        self.rings: List[LinkList] = []
        self.Source = Point()
        self.Target = Point()

    def CreatRing(self, points: List[Point]):
        ring = LinkList()
        for pt in points:
            ring.insert(pt)
        self.rings.append(ring)

    def ReadFromFile(self):
        lines = ""
        # working_directory = os.getcwd() temp mrr
        # file_path = os.path.join(working_directory, "test_file.txt") temp mrr
        file_path = os.path.join("/home/mohammad/catkin_ws/src/moving_agent/src/planning/hybrid_based/PPO_Continues/classic_approach", "test_file.txt")
        with open(file_path, "r") as file:
            lines = file.readlines()
            # everything = "-".join(lines)
        return lines

    def Initialize(self, input_str = ""):
        string = ""
        # try:
        if(input_str == ""):
            string = self.ReadFromFile()
        else:
            string = input_str.splitlines()
        # except exceptiongroup as ex:
        #     print("can not read from file. exception occured.")
        tmp_points : List[Point] = []
        parts = string
        self.Source.x = int(parts[0])
        self.Source.y = int(parts[1])
        i = 2
        while i < len(parts):
            if parts[i] == "ring" or parts[i] == "ring\n":
                self.CreatRing(tmp_points)
                tmp_points.clear()
                i += 1
            elif not parts[i].startswith("target"):
                pttt = Point(int(parts[i]), int(parts[i + 1]))
                tmp_points.append(pttt)
                i += 2
            if parts[i].startswith("target"): #parts[i] == "target" or parts[i] == "target\n":
                self.Target.x = int(parts[i + 1])
                self.Target.y = int(parts[i + 2])
                break

    # def draw_path_and_obstacles(self, points, rings):
    #     for ring in rings:
    #         temp = ring.head
    #         while True:
    #             plt.plot([temp.op.x, temp.Next.op.x], [temp.op.y, temp.Next.op.y], marker='s', color='red', linewidth=5)
    #             temp = temp.Next
    #             if temp == ring.head:
    #                 break

    #     x_values = [point.x for point in points]
    #     y_values = [point.y for point in points]

    #     plt.plot(x_values, y_values, marker='o', color='blue')
    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #     plt.title('Lines and Obstacles')
    #     plt.show()
    
    def main(self, min_leg_length, max_turning_angle, width, height):
        preparation = Preparation()
        preparation.Initialize()

        pr = Process()
        pr.initialize(preparation.rings, preparation.Source, min_leg_length, max_turning_angle, width, height)
        # pr = Process(preparation.rings, preparation.Source, min_leg_length, max_turning_angle, width, height)
        pr.MainLoop()
        path_temp = pr.MainFunction(preparation.Target)
        # print("result: ")
        # for pt in path_temp:
        #     print("X:", pt.x, "Y:", pt.y)
        return path_temp, preparation.rings, preparation.Source, preparation.Target
        
        #drawing
        # preparation.draw_path_and_obstacles(path_temp, preparation.rings)


# if __name__ == "__main__":
#     preparation = Preparation()
#     preparation.Initialize()
#     pr = Process(preparation.rings, preparation.Source)
#     pr.MainLoop()
#     path_temp = pr.MainFunction(preparation.Target)
#     print("result: ")
#     for pt in path_temp:
#         print("X:", pt.x, "Y:", pt.y)
    
#     #drawing
#     preparation.draw_path_and_obstacles(path_temp, preparation.rings)
    


