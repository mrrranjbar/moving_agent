#!/usr/bin/env python

import random
import math
import numpy as np
from PPO_Continues.classic_approach.Point import Point
from PPO_Continues.classic_approach.LinkList import LinkList
from typing import List


class CustomShortPathEnv:
    def __init__(self, state_dim, action_dim, max_episode_steps, max_turning_angle, min_leg_length):
        self.state_dim = state_dim # 3:current (x,y,angle) + 2: target(x,y) + {todo: 2: pos of nearest obstacle} = 5
        self.action_dim = action_dim # 2: continuous (l,alpha) + 1: discreat (0: cw, 1:ccw) = 3
        self.max_episode_steps = max_episode_steps # (dist target and start) / min l * 50
        self.current_step = 0
        # self.state = None
        self._max_turning_angle = max_turning_angle 
        self._min_leg_length = min_leg_length 
        self.Width = 100
        self.Height = 100
        self._max_leg_length = self.Width / 2 #math.sqrt(self.Width * self.Width + self.Height * self.Height) # 141
        self.total_steps = 0
        self.GOAL = 2
        self.HAS_INTERSECTION = -2
        self.OUT_OF_ENVIRONMENT = -2
        self.MAX_EPISODE_STEP = -2

        # Define action space (assuming continuous action space)
        self.action_space = np.zeros(action_dim) # np.arange(-max_action_value, max_action_value, 0.1)
        self.action_space_dim = self.action_space.shape[0]

        # Define observation space (assuming continuous state space)
        self.observation_space = np.zeros(state_dim)

        # related to problem
        self.MaxNumberOfObastacles = 0
        self.rings: List[LinkList] = []
        #
        x1 = random.randint(0,20)
        y1 = random.randint(0,20)
        x2 = random.randint(80,self.Width)
        y2 = random.randint(80,self.Height)
        self.Start = Point(x1, y1)
        self.Target = Point(x2, y2)
        self.CurrentPos = Point(x1, y1)
        self.StartAngle = self.calc_start_angle()
        self.CurrentPosAngle = self.StartAngle
        self.generate_random_obstacles(random.randint(0,self.MaxNumberOfObastacles))
        self.Path: List[Point] = []
        self.DistanceToTarget = round(self.calc_distance(self.CurrentPos, self.Target), 2)
        self.s_value_for_ray = 500
        self.AngleToTarget = self.find_angle(self.CurrentPos, self.Target)
        self.number_of_goals_happened = 0
        self.number_of_reset_happened = 0
        self.minimum_distance_from_nearest_obatacle = self.find_minimum_distance_from_nearest_obatacle()

    
    def generate_random_obstacles(self, number_of_obstacles):
        self.rings.clear()
        # for _ in range(number_of_obstacles):
        #     ring = LinkList()
        #     p1 = Point(random.randint(0,self.Width), random.randint(0,self.Height))
        #     p2 = Point(random.randint(0,self.Width), random.randint(0,self.Height))
        #     while(self.is_intersect_with_obatacles(p1, p2)):
        #         p1 = Point(random.randint(0,self.Width), random.randint(0,self.Height))
        #         p2 = Point(random.randint(0,self.Width), random.randint(0,self.Height))
        #     ring.insert(p1)
        #     ring.insert(p2)
        #     self.rings.append(ring)
        
        # obs1
        ring1 = LinkList()
        ring1.insert(Point(random.randint(25,50), random.randint(80,100)))
        ring1.insert(Point(random.randint(25,50), random.randint(80,100)))
        self.rings.append(ring1)

        # obs2
        ring2 = LinkList()
        ring2.insert(Point(random.randint(50,75), random.randint(50,75)))
        ring2.insert(Point(random.randint(50,75), random.randint(50,75)))
        self.rings.append(ring2)

        # obs3
        ring3 = LinkList()
        ring3.insert(Point(random.randint(75,100), random.randint(20,30)))
        ring3.insert(Point(random.randint(75,100), random.randint(20,30)))
        self.rings.append(ring3)

        # obs4
        ring4 = LinkList()
        ring4.insert(Point(random.randint(50,60), random.randint(10,20)))
        ring4.insert(Point(random.randint(10,20), random.randint(40,49)))
        self.rings.append(ring4)

        # obs5
        # ring5 = LinkList()
        # ring5.insert(Point(random.randint(10,40), random.randint(50,75)))
        # ring5.insert(Point(random.randint(10,40), random.randint(50,75)))
        # self.rings.append(ring5)

        # obs6
        # ring6 = LinkList()
        # ring6.insert(Point(random.randint(40,60), random.randint(40,60)))
        # ring6.insert(Point(random.randint(40,60), random.randint(40,60)))
        # self.rings.append(ring6)


        # ring.insert(Point(200, 300))
        # ring.insert(Point(300, 200))
        
    
    def find_angle(self, p1:Point, p2:Point):
        # Calculating degree between p1 & p2 , zero is in p1
        degree = math.degrees(math.atan2(p2.y - p1.y, p2.x - p1.x))
        if degree < 0:
            degree += 360
        return degree
    
    def find_length(self, p1:Point, p2:Point):
        return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)

    def show(self):
        print("Custom Short Path Environment:")
        print(f"Width: {self.Width}, Height: {self.Height}")
        
        # Print obstacles
        print("Obstacles:")
        for idx, ring in enumerate(self.rings, start=1):
            points = [ring.head.op, ring.head.Next.op]
            obstacle_str = f"Obstacle {idx}:"
            for p in points:
                obstacle_str += f" ({p.x}, {p.y})"
            print(obstacle_str)
        
        # Print the path
        print("Path:")
        for step, point in enumerate(self.Path, start=1):
            print(f"Step {step}: ({point.x}, {point.y})")

        # Print start and target points
        print(f"Start Point: ({self.Start.x}, {self.Start.y})")
        print(f"Target Point: ({self.Target.x}, {self.Target.y})")

    def step(self, action, is_run = False, solved_by_classic_method = False):
        done = False
        # Implement your custom step logic here.
        # scaled_value = ((tanh_output + 1) / 2) * (max_range - min_range) + min_range
        l = action[0] # [-1,1] => L
        l = ((l + 1) / 2) * (self._max_leg_length - self.min_leg_length) + self.min_leg_length # (minL, minL * 4): int
        alpha = action[1] # [-1,1] => alpha
        alpha = ((alpha + 1) / 2) * (self.max_turning_angle - (-self.max_turning_angle)) + (-self.max_turning_angle) # (-maxAlpha, +maxAlpha) : int
        # pre_Pos = Point(self.CurrentPos.x, self.CurrentPos.y)
        is_intersect_with_obstacle, is_out_of_environment = self.run(l, alpha)
        self.Path.append(Point(self.CurrentPos.x, self.CurrentPos.y))

        self.DistanceToTarget = round(self.calc_distance(self.CurrentPos, self.Target), 2)
        
        # Compute the next state (s_), reward (r), and done flag (done) based on the given action.
        ####state######
        # s_ = np.zeros(self.state_dim)
        # s_[0] = self.CurrentPosAngle
        # s_[1] = self.AngleToTarget
        # s_[2] = self.DistanceToTarget
        # s_[3] = self.find_minimum_distance_from_nearest_obatacle()
        # # print(f"minimum distance from nearest obatacle: {s_[4]}")
        # # s_[3] = self.Target.x
        # # s_[4] = self.Target.y
        self.minimum_distance_from_nearest_obatacle = self.find_minimum_distance_from_nearest_obatacle()
        s_ = self.get_states()

        self.current_step += 1
        self.total_steps = self.total_steps + 1

        ### Circle Radius ######
        circ_radius = self.min_leg_length
        # circ_radius = self.min_leg_length * 3
        # if self.number_of_goals_happened > 200 and self.number_of_goals_happened < 400 :
        #     circ_radius = self.min_leg_length * 2
        # elif self.number_of_goals_happened >= 400 :
        #     circ_radius = self.min_leg_length
        
        if is_run:
            circ_radius = self.min_leg_length
        
        # path_length = self.calc_distance(self.CurrentPos, self.Start)
        #### reward ######
        # note: reward = -exp(distance) - exp(steps)
        
        # r = -distance - self.current_step
        diometer_of_env = math.sqrt(self.Width * self.Width + self.Height * self.Height) # 141
        temp_coef = 1 if self.minimum_distance_from_nearest_obatacle <= 1 else self.minimum_distance_from_nearest_obatacle
        inverse_distance_from_nearest_obs = (1.0 / temp_coef) * diometer_of_env

        
        angle_error = self.find_angle_error()
        # w_distance_to_target = 1.0
        w_current_step = 1.0
        w_angle_error = 1.0
        w_inverse_distance_from_nearest_obs = 1.0
        sum =  (
            # self.min_max_scale_and_weight(self.DistanceToTarget, 0, diometer_of_env, w_distance_to_target) +
               self.min_max_scale_and_weight(self.current_step, 0, self.max_episode_steps, w_current_step) +
               self.min_max_scale_and_weight(angle_error, 0, 180, w_angle_error) +
               self.min_max_scale_and_weight(inverse_distance_from_nearest_obs, 0, diometer_of_env, w_inverse_distance_from_nearest_obs))
        # r = - (sum / (w_distance_to_target + w_current_step + w_angle_error + w_inverse_distance_from_nearest_obs))
        # r = - (sum / (w_distance_to_target + w_angle_error + w_inverse_distance_from_nearest_obs))
        r = - (sum / (w_angle_error + w_inverse_distance_from_nearest_obs + w_current_step))

        # log rewards
        # file_path = '/home/mohammad/catkin_ws/src/moving_agent/src/planning/hybrid_based/PPO_Continues/log_rewards.txt'
        # with open(file_path, 'a') as file:
        #     # Write content to the file
        #     log_content = F'=========== New Step ===========================\n'
        #     log_content += F'Distance to target = {self.min_max_scale_and_weight(self.DistanceToTarget, 0, diometer_of_env, w_distance_to_target)}\n'
        #     # log_content += F'Current Step = {self.min_max_scale_and_weight(self.current_step, 0, self.max_episode_steps, w_current_step)}\n'
        #     log_content += F'Angle Error = {self.min_max_scale_and_weight(angle_error, 0, 180, w_angle_error)}\n'
        #     log_content += F'Inv dist from obs = {self.min_max_scale_and_weight(inverse_distance_from_nearest_obs, 0, diometer_of_env, w_inverse_distance_from_nearest_obs)}\n'
        #     log_content += F'reward = {r}\n'

        #     file.write(log_content)

        


        if is_intersect_with_obstacle and not solved_by_classic_method:
            done = True
            # r = -distance - self.max_episode_steps - 100 
            r = self.HAS_INTERSECTION
            print(F"HAS INTERSECTION! Reward= {r}, classic={solved_by_classic_method}, Steps = {self.current_step}, radius = {circ_radius}")
            return s_, r, done, {}
        
        if is_out_of_environment:
            done = True
            # r = -distance - self.max_episode_steps - 100 
            r = self.OUT_OF_ENVIRONMENT
            print(F"OUT OF ENVIRONMENT! Reward= {r}, classic={solved_by_classic_method}, Steps = {self.current_step}, radius = {circ_radius}")
            return s_, r, done, {}
        
        if self.current_step >= self.max_episode_steps:
            done = True
            r = self.MAX_EPISODE_STEP
            print(F"Max Episode Steps is happened! Reward= {r}, classic={solved_by_classic_method}, Steps = {self.current_step}, radius = {circ_radius}")
            return s_, r, done, {}
            # raise Exception("Episode has already ended. Call reset() to start a new episode.")

        
        # print(F"Normal Reward= {r}")

        ### Goals ###
        if(self.is_point_in_circle(self.CurrentPos, self.Target, circ_radius)):
            # self.is_point_on_line(self.Target, pre_Pos, self.CurrentPos) ):
            # int(self.CurrentPos.x) >= int(self.Target.x - 10) and int(self.CurrentPos.x) <= int(self.Target.x) and 
        #    int(self.CurrentPos.y) >= int(self.Target.y - 10) and int(self.CurrentPos.y) <= int(self.Target.y)
        #    ):
            done = True
            r = self.GOAL
            # self.number_of_goals_happened += 1
            print(F"GOALLLLLLLL!!!!!!!!!!!!!!!!!!!!!!!!!!!!, classic={solved_by_classic_method}, Reward= {r}, Steps = {self.current_step}, radius = {circ_radius}, goals={self.number_of_goals_happened}")
            
            if is_run :
                return s_, r, done, {}

            # if self.number_of_goals_happened == 500: # change the environment after learning the previouse environment very well
            #     self.number_of_goals_happened = 0
            self.reset_environment()
            return s_, r, done, {}
        
        return s_, r, done, {}  # Return a dictionary for any additional info you want to pass
    
    def min_max_scale_and_weight(self, value, min_val, max_val, weight):
        scaled_value = (value - min_val) / (max_val - min_val)
        return scaled_value * weight
    
    def find_angle_error(self):
        angle_error = abs(self.AngleToTarget - self.CurrentPosAngle)
        if angle_error > 180:
            angle_error = 360 - angle_error
        return angle_error
    
    def is_point_on_line(self, p : Point, v1: Point, v2: Point):
        # Calculate the range for x and y for the point to lie on the line segment
        x_range = (p.x >= min(v1.x, v2.x) and p.x <= max(v1.x, v2.x))
        y_range = (p.y >= min(v1.y, v2.y) and p.y <= max(v1.y, v2.y))
        return (p.y - v1.y) * (v2.x - v1.x) == (v2.y - v1.y) * (p.x - v1.x) and x_range and y_range
    
    def is_point_in_circle(self, pt: Point, circle_center: Point, circle_radius):
        distance = math.sqrt((pt.x - circle_center.x)**2 + (pt.y - circle_center.y)**2)
        
        return distance <= circle_radius

    def reset(self, is_run = False):
        self.current_step = 0
        self.number_of_reset_happened += 1
        if self.number_of_reset_happened == 100 : 
            self.number_of_reset_happened = 0
            # self.number_of_goals_happened = 0
            self.reset_environment()
        elif is_run : 
            self.reset_environment()

        # x = random.randint(0,self.Width)
        # y = random.randint(0,self.Height)
        # self.Start = Point(x, y)
        # self.Target = Point(random.randint(0,self.Width), random.randint(0,self.Height))
        # self.CurrentPos = Point(self.Start.x, self.Start.y)
        # self.StartAngle = random.randint(10, 100)
        # self.CurrentPosAngle = self.StartAngle
        # self.generate_random_obstacles(random.randint(0,self.MaxNumberOfObastacles))
        self.CurrentPos = Point(self.Start.x, self.Start.y)
        self.CurrentPosAngle = self.StartAngle
        self.DistanceToTarget = round(self.calc_distance(self.CurrentPos, self.Target), 2)
        self.AngleToTarget = self.find_angle(self.CurrentPos, self.Target)
        self.minimum_distance_from_nearest_obatacle = self.find_minimum_distance_from_nearest_obatacle()
        s = self.get_states()
        self.Path.clear()
        self.Path.append(Point(self.Start.x, self.Start.y))

        return s
    
    def calc_start_angle(self, angle = None):
        if angle == None:
            return 0 #random.randint(10, 100)
        else:
            return angle
    
    def reset_environment(self):
        self.generate_random_obstacles(random.randint(0,self.MaxNumberOfObastacles))
        self.StartAngle = self.calc_start_angle()
        self.Start = Point(random.randint(0,20), random.randint(0,20))
        self.Target = Point(random.randint(80,self.Width), random.randint(80,self.Height))

    def calc_distance(self, p1, p2):
        return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)
    
    def run(self, l, alpha):
        angle2 = (self.CurrentPosAngle + alpha) % 360
        x2, y2 = self.move_one_step(self.CurrentPos.x, self.CurrentPos.y, l, angle2)
        NewPos = Point(x2, y2)

        is_intersect_with_obstacle = self.is_intersect_with_obatacles(self.CurrentPos, NewPos)
        is_out_of_environment = self.is_out_of_environments(self.CurrentPos, NewPos)

        self.CurrentPos.x = x2
        self.CurrentPos.y = y2
        self.CurrentPosAngle = angle2

        self.AngleToTarget = self.find_angle(self.CurrentPos, self.Target)

        return is_intersect_with_obstacle, is_out_of_environment # is intersection with obstacle 
    
    def move_one_step(self, x, y, l, angle):
        # Assuming angle is in degrees, convert it to radians
        angle_radians = math.radians(angle)

        # Calculate new coordinates after moving one step
        x2 = int(x + l * math.cos(angle_radians))
        y2 = int(y + l * math.sin(angle_radians))

        return x2, y2
    
    def get_environment_bounds(self):
        # Assuming self.max_width and self.max_height are attributes of your environment
        return {'min_x': 0, 'min_y': 0, 'max_x': self.Width, 'max_y': self.Height}

    def is_point_within_bounds(self, point, bounds):
        return bounds['min_x'] <= point.x <= bounds['max_x'] and bounds['min_y'] <= point.y <= bounds['max_y']
    
    def environment_to_lines(self):
        bounds = self.get_environment_bounds()
        lines = []

        # Bottom line
        lines.append((Point(bounds['min_x'], bounds['min_y']), Point(bounds['max_x'], bounds['min_y'])))

        # Right line
        lines.append((Point(bounds['max_x'], bounds['min_y']), Point(bounds['max_x'], bounds['max_y'])))

        # Top line
        lines.append((Point(bounds['max_x'], bounds['max_y']), Point(bounds['min_x'], bounds['max_y'])))

        # Left line
        lines.append((Point(bounds['min_x'], bounds['max_y']), Point(bounds['min_x'], bounds['min_y'])))

        return lines

    
    def is_intersect_with_obatacles(self, p1, p2):
        # Check if the line is within the environment bounds
        # environment_bounds = self.get_environment_bounds()  # You need to implement this method
        # if not self.is_point_within_bounds(p1, environment_bounds) or not self.is_point_within_bounds(p2, environment_bounds):
        #     return True  # Line is out of bounds

        is_intersect_with_obstacle = False
        for ring in self.rings:
            is_intersect_with_obstacle = self.intersection(p1, p2, ring.head.op, ring.head.Next.op)
            if(is_intersect_with_obstacle):
                break
        return is_intersect_with_obstacle
    
    def is_out_of_environments(self, p1, p2):
        # Check if the line is within the environment bounds
        environment_bounds = self.get_environment_bounds()  # You need to implement this method
        if not self.is_point_within_bounds(p1, environment_bounds) or not self.is_point_within_bounds(p2, environment_bounds):
            return True  # Line is out of bounds

        return False
    
    def get_states(self):
        state = np.zeros(self.state_dim) 
        # state[0] = int(self.CurrentPosAngle) 
        angle_error = self.find_angle_error()
        state[0] = round(self.min_max_scale_and_weight(angle_error, 0, 180, 1), 2)
        diometer_of_env = math.sqrt(self.Width * self.Width + self.Height * self.Height) # 141
        state[1] = round(self.min_max_scale_and_weight(self.DistanceToTarget, 0, diometer_of_env, 1), 3)
        state[2] = round(self.min_max_scale_and_weight(self.minimum_distance_from_nearest_obatacle, 0, diometer_of_env, 1), 3)
        # state[4] = int(self.CurrentPos.x)
        # state[5] = int(self.CurrentPos.y)
        # print(F"============states=================")
        # print(F"Current pos angle: {state[0]}, Angle to target: {state[1]}, Distance to target: {state[2]}, Minimum dist to obs: {state[3]}")
        return state
    
    # def find_intersection_point(self, p1: Point, p2: Point, q1: Point, q2: Point):
    #     # Calculate slopes
    #     epsilon = 1e-10
    #     m1_numer = p2.y - p1.y
    #     m1_denom = p2.x - p1.x
    #     m2_numer = q2.y - q1.y
    #     m2_denom = q2.x - q1.x

    #     # Check if lines are close to parallel
    #     if abs(m1_denom) < epsilon:
    #         m1_denom = epsilon if m1_denom < 0 else -epsilon
    #     if abs(m2_denom) < epsilon:
    #         m2_denom = epsilon if m2_denom < 0 else -epsilon

    #     # Calculate slopes
    #     m1 = m1_numer / m1_denom
    #     m2 = m2_numer / m2_denom

    #     # Make sure lines are not parallel
    #     # if abs(m1 - m2) < epsilon:
    #     #     m2_denom += epsilon  # Adjust one of the denominators slightly
    #     #     m2 = m2_numer / m2_denom

    #     m1_m2 = abs(m1 - m2)
    #     if m1_m2 == 0:
    #         m1_m2 = epsilon

    #     # Calculate intersection point
    #     x_intersect = (m1 * p1.x - m2 * q1.x + q1.y - p1.y) / m1_m2
    #     y_intersect = m1 * (x_intersect - p1.x) + p1.y

    #     # intersection_point = Point(x_intersect, y_intersect)
    #     # return intersection_point
    #     # Check if intersection point is within the range of both lines
    #     if (p1.x == p2.x or self.is_between(p1.x, x_intersect, p2.x)) and \
    #     (q1.x == q2.x or self.is_between(q1.x, x_intersect, q2.x)) and \
    #     (p1.y == p2.y or self.is_between(p1.y, y_intersect, p2.y)) and \
    #     (q1.y == q2.y or self.is_between(q1.y, y_intersect, q2.y)):
    #         intersection_point = Point(x_intersect, y_intersect)
    #         return intersection_point
    #     else:
    #         return None
    
    def line_intersection(self, line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return None

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return Point(x, y)
    
    def is_between(self, a: int, b: int, c: int) -> bool:
        return a <= b <= c or c <= b <= a
    
    def intersection(self, line1V1, line1V2, line2V1, line2V2):
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
    
    def find_minimum_distance_from_nearest_obatacle(self):
        is_intersect_with_obstacle = False
        min_dist = self.s_value_for_ray
        x2, y2 = self.move_one_step(self.CurrentPos.x, self.CurrentPos.y, self.s_value_for_ray, self.CurrentPosAngle)
        p1 = self.CurrentPos
        p2 = Point(x2, y2)
        for ring in self.rings:
            is_intersect_with_obstacle = self.intersection(p1, p2, ring.head.op, ring.head.Next.op)
            if(is_intersect_with_obstacle):
                intersection_point = self.line_intersection([[p1.x,p1.y],[p2.x, p2.y]],[[ring.head.op.x,ring.head.op.y],[ring.head.Next.op.x,ring.head.Next.op.y]]) #p1, p2, ring.head.op, ring.head.Next.op)
                if intersection_point != None:
                    min_dist = min(self.calc_distance(self.CurrentPos, intersection_point) , min_dist)

        for line in self.get_environment_lines():    
            intersection_point = self.line_intersection([[p1.x,p1.y],[p2.x, p2.y]],[[line[0].x,line[0].y],[line[1].x,line[1].y]]) 
            if intersection_point != None:
                min_dist = min(self.calc_distance(self.CurrentPos, intersection_point) , min_dist)
        return round(min_dist, 2)
    
    def get_environment_lines(self):
        # Define the four corners of the rectangle
        top_left = Point(0, 0)
        top_right = Point(self.Width, 0)
        bottom_left = Point(0, self.Height)
        bottom_right = Point(self.Width, self.Height)

        # Define the lines using the corner points
        line1 = [top_left, top_right]
        line2 = [top_right, bottom_right]
        line3 = [bottom_right, bottom_left]
        line4 = [bottom_left, top_left]

        # Return the list of lines
        return [line1, line2, line3, line4]
    
    @property
    def _max_episode_steps(self):
        return self.max_episode_steps

    @property
    def observation_space_shape(self):
        return self.observation_space.shape[0]

    @property
    def action_space_shape(self):
        return self.action_space_dim

    @property
    def action_space_high(self):
        return self.max_action_value
    
    @property
    def min_leg_length(self):
        return self._min_leg_length
    
    @property
    def max_leg_length(self):
        return self._max_leg_length
    
    @property
    def max_turning_angle(self):
        return self._max_turning_angle