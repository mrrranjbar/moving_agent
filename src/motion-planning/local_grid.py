import numpy as np
import math
from scipy.stats import multivariate_normal
from skimage.measure import block_reduce
from scipy.ndimage import uniform_filter


class LocalGridSimple:
    def __init__(self, grid_size = 128, grid_value = 64, robot_i = 0, robot_j = 63, step_probability = 0.8):
        self.grid_size = grid_size  # grid_size x grid_size => grid
        self.grid_value = grid_value # each grid cell is 64 = 128/2 => grid is (2*2) meter^2
        self.robot_i = robot_i
        self.robot_j = robot_j
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=float)
        self.step_probability = step_probability
        self.end_x_line_shift = self.grid_size # x dirextion's line is ([robot_i,robot_j], [robot_i + end_x_line_shift, robot_j])
        self.states_size = 6
        self.down_block_size = 16 # 5
        self.diameter = math.sqrt(self.grid_size**2+self.grid_size**2) / self.down_block_size
        self.pre_yaw = 0

    def step(self, dx, dy, front, left, right, yaw, target_dist, target_angle_error):
        if dx == 0 and dy == 0:
            return self.get_states()
        self.shift_grid(dx, dy, self.pre_yaw, yaw) 
        self.mark_obstacle_in_grid(front, yaw, 0) 
        self.mark_obstacle_in_grid(left, yaw, math.pi / 2) 
        self.mark_obstacle_in_grid(right, yaw, -math.pi / 2) 
        self.mark_target_in_grid(target_dist, yaw, target_angle_error) 
        states = self.get_states()
        self.pre_yaw = yaw
        return states

    def reset(self):
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=float)

    def get_states(self):
        down_grid = block_reduce(self.grid, block_size=(self.down_block_size, self.down_block_size), func=np.mean) # 128 * 128 => 8 * 8
        return down_grid.flatten()

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

    def mark_obstacle_in_grid(self, distance, yaw, angle):
        if distance >= 2000.0:
            return
        distance_m = distance / 1000.0
        # (i, j) in A:
        i = int(distance_m * math.cos(yaw + angle) * self.grid_value)
        j = int(distance_m * math.sin(yaw + angle) * self.grid_value)
        # transition to B
        # rotation
        i2 = int(i * math.cos(-yaw) - j * math.sin(-yaw))
        j2 = int(i * math.sin(-yaw) + j * math.cos(-yaw))
        # transfer
        new_i = i2
        new_j = int(j2 + self.grid_size / 2)
        if 0 <= new_i < self.grid_size and 0 <= new_j < self.grid_size:
            self.normal_distribution(new_i, new_j, +1)
        return

    def mark_target_in_grid(self, distance_to_target_m, yaw, angle_to_target):
        # (i, j) in A:
        i = int(distance_to_target_m * math.cos(angle_to_target + yaw) * self.grid_value)
        j = int(distance_to_target_m * math.sin(angle_to_target + yaw) * self.grid_value)
        # transition to B
        # rotation
        i2 = int(i * math.cos(-yaw) - j * math.sin(-yaw))
        j2 = int(i * math.sin(-yaw) + j * math.cos(-yaw))
        # transfer
        new_i = i2
        new_j = int(j2 + self.grid_size / 2)
        if 0 <= new_i < self.grid_size and 0 <= new_j < self.grid_size:
            # self.normal_distribution(new_i, new_j, -1)
            self.grid[new_i, new_j] = -(self.down_block_size * self.down_block_size) # -256
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
        min_P = P.min()
        max_p = P.max()
        # Iterate over the grid and compute the probability at each point
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                if abs(i - mean_i)**2 + abs(j - mean_j)**2 <= sigma1**2:
                    if P[i, j] > 0:
                        self.grid[i, j] = P[i, j] / max_p
                    elif P[i, j] < 0:
                        self.grid[i, j] = -P[i, j] / min_P
                    else:
                        self.grid[i, j] = 0
        return


    def shift_grid(self, dx, dy, pre_yaw, yaw):
        di = int(dx * self.grid_value)
        dj = int(dy * self.grid_value)
        self.grid[self.robot_i, self.robot_j] = 0
        temp_grid = np.zeros((self.grid_size, self.grid_size), dtype=float)
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                cell_val = self.grid[i, j]
                # transition to A
                # transfer
                i2 = i
                j2 = int(j -self.grid_size/2)
                # rotation
                i3 = int(i2 * math.cos(pre_yaw) - j2 * math.sin(pre_yaw))
                j3 = int(i2 * math.sin(pre_yaw) + j2 * math.cos(pre_yaw))
                # shift
                i4 = i3 - di
                j4 = j3 - dj
                # transition to B
                # rotation
                i5 = int(i4 * math.cos(-yaw) - j4 * math.sin(-yaw))
                j5 = int(i4 * math.sin(-yaw) + j4 * math.cos(-yaw))
                # transfer
                new_i = i5
                new_j = int(j5 + self.grid_size / 2)

                if 0 <= new_i < self.grid_size and 0 <= new_j < self.grid_size and cell_val > 0:
                    temp_grid[new_i, new_j] = cell_val * self.step_probability

        self.reset()
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                self.grid[i, j] = temp_grid[i, j]

        self.grid = uniform_filter(self.grid, size=9, mode='reflect')
        return # self.grid is in B