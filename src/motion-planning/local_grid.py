from os import stat_result
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import math
# import random
from scipy.stats import multivariate_normal
from skimage.measure import block_reduce


class LocalGridSimple:
    def __init__(self, grid_size = 200, grid_value = 100, robot_i = 99, robot_j = 99, step_probability = 0.8):
        self.grid_size = grid_size  # 20x20 grid
        self.grid_value = grid_value # each grid cell is 1/100
        # self.grid_resolution = 1 / self.grid_value  # Each cell in the grid represents 0.1 meter
        self.robot_i = robot_i
        self.robot_j = robot_j
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=float)
        self.step_probability = step_probability
        self.end_x_line_shift = 100 # x dirextion's line is ([robot_i,robot_j], [robot_i, robot_j + end_x_line_shift])
        self.states_size = 6
        self.down_block_size = 5
        self.diameter = 140.0 / self.down_block_size

    def step(self, dx, dy, front, left, right, yaw, target_dist, target_angle_error):
        if dx == 0 and dy == 0:
            return self.get_states(yaw)
        self.shift_grid(dx, dy)
        self.mark_obstacle_in_grid(front, yaw)
        self.mark_obstacle_in_grid(left, yaw + math.pi / 2)
        self.mark_obstacle_in_grid(right, yaw - math.pi / 2)
        self.mark_target_in_grid(target_dist, target_angle_error + yaw)
        states = self.get_states(yaw)
        return states

    def reset(self):
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=float)
        # self.grid[self.robot_i, self.robot_j] = 2

    # def custom_reduce(self, block, axis=None):
    #     if -1 in block:
    #         return -1
    #     else:
    #         return np.max(block)
    
    def get_states(self, yaw):
        # print("yaw", math.degrees(yaw)) #mrr
        states = []
        down_block_size = self.down_block_size
        # min_p1 = self.grid.min()
        # max_p1 = self.grid.max()
        down_grid = block_reduce(self.grid, block_size=(down_block_size, down_block_size), func=np.mean) # 200 * 200 => 40 * 40
        # min_p2 = down_grid.min()
        # max_p2 = down_grid.max()
        # print("min1= ", min_p1)
        # print("min2= ", min_p2)
        # print("max1= ", max_p1)
        # print("max2= ", max_p2)

        for i in range(len(down_grid)):
            for j in range(len(down_grid)):
                robot_i = int(self.robot_i / down_block_size)
                robot_j = int(self.robot_j / down_block_size)
                end_x_line_shift = self.end_x_line_shift / down_block_size
                if down_grid[i, j] != 0 and i != robot_i and j != robot_j:
                    angle3p = self.angle_between_points([i,j],[robot_i, robot_j], [robot_i + end_x_line_shift, robot_j])
                    angle_error = self.angle_error(angle3p, yaw)
                    distance = round(math.sqrt((robot_i - i) ** 2 + (robot_j - j) ** 2), 0)
                    probability = down_grid[i, j]
                    # print(F"angle_error= {angle_error},  distance= {distance}, prob= {probability}, angle3p= {angle3p}, yaw= {yaw} ")
                    # Check if angle is within [yaw - π/2, yaw + π/2] considering wrapping
                    if -math.pi / 2 <= angle_error <= math.pi / 2:# or (probability < 0 and -math.pi / 2 <= angle_error <= math.pi / 2):
                        states.append([angle_error, distance, probability])
                        states.sort(key=lambda state: (state[1]))
                        # if probability >=0:
                        #     # soert by increasing distance and decreasing probability
                        #     states.sort(key=lambda state: (state[1], -state[2]))
                        # else:
                        #     # soert by increasing distance and increasing probability
                        #     states.sort(key=lambda state: (state[1], state[2]))

        unique_states = []
        local_state_with_max_probability = [0, 0, -100]
        if len(states) > 0:
            local_state_with_max_probability = states[0]
            if len(states) == 1:
                unique_states.append(local_state_with_max_probability)

        for i in range(1, len(states)):
            distance = states[i][1]
            probability = abs(states[i][2])
            lsmp_distance = local_state_with_max_probability[1]
            lsmp_probability = abs(local_state_with_max_probability[2])
            if lsmp_distance == distance:
                if lsmp_probability < probability:
                    local_state_with_max_probability = states[i]
            else:
                unique_states.append(local_state_with_max_probability)
                local_state_with_max_probability = states[i]
            if i == len(states) - 1:
                unique_states.append(local_state_with_max_probability)


        while(len(unique_states) < self.states_size):
            unique_states.append([math.pi, self.diameter, 0])

        return unique_states[0:self.states_size]

    def angle_between_points(self, A, B, C):
        # Calculate the vectors AB and BC
        AB = (A[0] - B[0], A[1] - B[1])
        BC = (C[0] - B[0], C[1] - B[1])

        # Calculate the angles of AB and BC relative to the x-axis using atan2
        angle_AB = math.atan2(AB[1], AB[0])
        angle_BC = math.atan2(BC[1], BC[0])

        # Calculate the angle between AB and BC in radians, within [-π, π]
        angle = angle_BC - angle_AB

        # Normalize the angle to the range [-π, π]
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi

        return -angle

    def angle_error(self, angle, yaw):
        # Calculate the difference
        diff = angle - yaw

        # Normalize the difference to the range [-π, π]
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi

        return diff


    def mark_obstacle_in_grid(self, distance, angle):
        distance_m = distance / 1000.0
        new_i = self.robot_i + int(distance_m * math.cos(angle) * self.grid_value)
        new_j = self.robot_j + int(distance_m * math.sin(angle) * self.grid_value)
        if 0 <= new_i < self.grid_size and 0 <= new_j < self.grid_size:
            self.normal_distribution(new_i, new_j, +1)
        return

    def mark_target_in_grid(self, distance_to_target_m, angle_to_target):
        new_i = self.robot_i + int(distance_to_target_m * math.cos(angle_to_target) * self.grid_value)
        new_j = self.robot_j + int(distance_to_target_m * math.sin(angle_to_target) * self.grid_value)
        if 0 <= new_i < self.grid_size and 0 <= new_j < self.grid_size:
            # print(f"i={new_i}, j={new_j}, angle={math.degrees(angle_to_target)}")
            # self.normal_distribution(new_i, new_j, -1)
            self.grid[new_i, new_j] = -(self.down_block_size * self.down_block_size) # -25
        return

    def normal_distribution(self, mean_i, mean_j, sign):
        # Define points p1
        p1 = np.array([mean_i, mean_j])  # p1 coordinates

        # Standard deviations (variances)
        sigma1 = 10  # Variance around p1

        P = np.zeros((self.grid_size, self.grid_size), dtype=float)

        # Define the covariance matrices for p1
        cov1 = [[sigma1**2, 0], [0, sigma1**2]]  # Covariance matrix for p1

        # Create Gaussian distributions for p1
        rv1 = multivariate_normal(p1, cov1)  # Gaussian centered at p1

        # self.grid[self.robot_i, self.robot_j] = 0 #0.001
        # Iterate over the grid and compute the probability at each point
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                if abs(i - mean_i)**2 + abs(j - mean_j)**2 <= sigma1**2:
                    # print(f"i = {i}, j = {j}")

                    # Evaluate the probability at each grid point (i, j)
                    pos = np.array([i, j])
                    P[i, j] = rv1.pdf(pos) * sign # w1 * self.grid[i, j] + w2 * rv1.pdf(pos)  # w1 * rv1.pdf(pos) + w2 * rv2.pdf(pos)
                    # print(f"grid_value = {self.grid[i, j]}")
        # min_P = P.min()
        max_p = P.max()
        # Iterate over the grid and compute the probability at each point
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                if abs(i - mean_i)**2 + abs(j - mean_j)**2 <= sigma1**2:
                    if P[i, j] > 0:
                        self.grid[i, j] = P[i, j] / max_p
                    # elif P[i, j] < 0:
                    #     self.grid[i, j] = -P[i, j] / min_P
                    else:
                        self.grid[i, j] = 0
        return


    def shift_grid(self, dx, dy):
        di = int(dx * self.grid_value)
        dj = int(dy * self.grid_value)
        self.grid[self.robot_i, self.robot_j] = 0
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                cell_val = self.grid[i, j]
                self.grid[i, j] = 0
                new_i = i - di
                new_j = j - dj
                if 0 <= new_i < self.grid_size and 0 <= new_j < self.grid_size and cell_val > 0:
                    # print(round(cell_val * self.step_probability, 1))
                    self.grid[new_i, new_j] = cell_val * self.step_probability
        # self.grid[self.robot_i, self.robot_j] = 2
        return