#!/usr/bin/env python
import math 
import numpy as np
from local_grid import LocalGridSimple
import time

class CustomRealEnv:
    def __init__(self, state_dim, max_episode_steps, velocity):
        self.URI = 'radio://0/80/2M/E7E7E7E705'
        self.start_x = -0.36
        self.start_y = -0.56
        self.start_z = 0
        self.start_yaw = 0
        self.Width = 2.0
        self.Height = 2.0
        self.tx = 0.5
        self.ty = 0.5

        self.state_dim = state_dim
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        self.state = np.zeros(self.state_dim) 
        self.velocity = velocity
        self.dt = 0.05  # time step in seconds
        self.x = self.start_x
        self.y = self.start_y
        self.pre_x = self.start_x
        self.pre_y = self.start_y
        self.z = self.start_z
        self.yaw = self.start_yaw
        self.diometer_of_env = math.sqrt(self.Width * self.Width + self.Height * self.Height) 
        self.dist = self.Width - 1
        self.OUT_OF_ENVIRONMENT = -2
        self.GOAL = 3
        self.HAS_INTERSECTION = -2
        self.angle_error = 0 #self.calculate_angle_error()
        self.is_first_state_info_arrived = False
        self.step_counter = 0
        self.local_grid = LocalGridSimple()
        self.local_grid_states = []
        self.is_goal = False
        self.call_grid_steps = []
        ####################################################

    def get_crazyflie_info(self, multi_ranger, current_x, current_y, current_z, current_yaw):
        self.x = current_x
        self.y = current_y
        self.z = current_z
        self.yaw = current_yaw
        self.front = float(multi_ranger.front) * 1000.0 if multi_ranger.front is not None else 2000.0
        self.left = float(multi_ranger.left) * 1000.0 if multi_ranger.left is not None else 2000.0
        self.right = float(multi_ranger.right) * 1000.0 if multi_ranger.right is not None else 2000.0

    
    def step(self, action, motion_commander, multi_ranger, current_x, current_y, current_z, current_yaw):
        self.do_actions(action, motion_commander)
        self.get_crazyflie_info(multi_ranger, current_x, current_y, current_z, current_yaw)
        s_ = self.get_states()
        r = self.get_reward(s_)
        self.current_step += 1
        done = False
        if self.front < 120 or self.right < 120 or self.left < 120:
            print("The drone has intersection!")
            r += self.HAS_INTERSECTION
            done = True
            print("reward: ", r)
            return s_, r, done, {}
        elif self.x > self.Width / 2.0 or self.x < -self.Width / 2.0 or self.y > self.Height / 2.0 or self.y < -self.Height / 2.0:
            print("The drone goes out of the environment!")
            r += self.OUT_OF_ENVIRONMENT
            done = True
            print("reward: ", r)
            return s_, r, done, {}
        elif self.current_step >= self.max_episode_steps:
            print("Max Episode Steps is happened!")
            done = True
            return s_, r, done, {}
        elif  (round(self.min_max_scale(self.angle_error, 0, math.pi/2), 2) < 0.1 and round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2) < 0.2) or (round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2) < 0.1):    
            print("GOAL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            r = self.GOAL
            done = True
            self.is_goal = True
            print("reward: ", r)
            return s_, r, done, {}
        return s_, r, done, {}  # Return a dictionary for any additional info you want to pass
    
    def do_actions(self, action, motion_commander):
        vyaw = 0
        if action == 0:
            vyaw = -0.8
        elif action == 1:
            vyaw = -0.4
        elif action == 2:
            vyaw = 0
        elif action == 3:
            vyaw = 0.4
        elif action == 4:
            vyaw = 0.8
        vx = self.velocity
        motion_commander.start_linear_motion(vx, 0, 0, vyaw * 30)
        time.sleep(self.dt) # 0.1
        return
    
    def calculate_angle_error(self):
        target_angle = math.atan2(self.ty - self.y, self.tx - self.x)
        angle_diff = target_angle - self.yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        angle_error = abs(angle_diff)
        
        return angle_error, angle_diff
    
    def get_reward(self, states):
        w_angle_error = 0.1
        w_dist = 0.8
        w_grid = 0.1
        r_angle_error = -round(self.min_max_scale(self.angle_error, 0, math.pi), 2)
        r_dist = -round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2)
        state_count = 2
        r_grid = 0
        i = state_count
        while i < len(states):
            angle_raw = states[i] 
            i += 1
            distance_raw = states[i]
            i += 1
            p = states[i] 
            i += 1
            if p > 0:
                r_obs_item = -p #* r_obs_raw #* obs_angle
                r_grid = r_obs_item if r_obs_item < r_grid else r_grid # min
            elif p < 0:
                r_target_raw = 1 - (distance_raw * 0.5) 
                r_target_item = r_target_raw #* target_angle
                r_grid = r_target_item if r_target_item > r_grid else r_grid # max
        r = 0
        r += w_angle_error * r_angle_error
        r += w_dist * r_dist
        r += w_grid * r_grid
        r = round(r,3)
        return r 
    
    def get_states(self):
        self.dist = math.sqrt(math.pow(self.x - self.tx , 2) + math.pow(self.y - self.ty, 2))
        self.angle_error, target_angle_error = self.calculate_angle_error()
        dx = round(self.x - self.pre_x, 2) # meter
        dy = round(self.y  - self.pre_y, 2) # meter
        self.local_grid_states = self.local_grid.step(dx, dy, self.front, self.left, self.right, self.yaw, self.dist, target_angle_error)
        self.call_grid_steps.append(f"states = local_grid.step({dx}, {dy}, {self.front}, {self.left}, {self.right}, {self.yaw}, {self.dist}, {target_angle_error})")
        self.pre_x = self.x
        self.pre_y = self.y
        self.state[0] = round(self.min_max_scale(self.angle_error, 0, math.pi), 2)
        self.state[1] = round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2)
        state_count = 2
        local_grid_index = 0
        while(state_count + 3 <= self.state_dim):
            if local_grid_index < len(self.local_grid_states):
                self.state[state_count] = round(self.local_grid_states[local_grid_index][0], 2) 
                state_count += 1
                self.state[state_count] = round(self.min_max_scale(self.local_grid_states[local_grid_index][1], 0, self.local_grid.diameter), 2)
                state_count += 1
                self.state[state_count] = round(self.local_grid_states[local_grid_index][2], 3)
                state_count += 1
                local_grid_index += 1
            else: 
                break
        return self.state
    
    def min_max_scale(self, value, min_val, max_val):
        value = np.clip(value, min_val, max_val)
        scaled_value = (value - min_val) / (max_val - min_val)
        return scaled_value

    def reset(self, multi_ranger, current_x, current_y, current_z, current_yaw):
        self.get_crazyflie_info(multi_ranger, current_x, current_y, current_z, current_yaw)
        self.current_step = 0
        self.local_grid = LocalGridSimple()
        self.local_grid_states = []
        self.local_grid.reset()
        self.step_counter = 0
        self.x = self.start_x
        self.y = self.start_y
        self.z = self.start_z
        self.yaw = self.start_yaw
        self.pre_x = self.start_x
        self.pre_y = self.start_y
        self.state = np.zeros(self.state_dim) 
        self.state = self.get_states()
        self.is_goal = False
        self.call_grid_steps.clear()
        return self.state