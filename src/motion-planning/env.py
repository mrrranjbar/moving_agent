#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import json
import math 

import numpy as np
from local_grid import LocalGridSimple

class CustomEnv:
    def __init__(self, state_dim, action_dim, max_episode_steps, max_action_value, velocity, wheel_base_length):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        # self.state = None
        self.state = np.zeros(self.state_dim) 
        self.max_action_value = max_action_value
        self.velocity = velocity
        self.wheel_base_length = wheel_base_length
        self.dt = 0.05  # time step in seconds
        self.timestep = 32
        self.ANGULAR_VELOCITY_COEF = 5
        # Initialize orientation
        self.orientation = 0.0  # initial orientation angle in radians

        # Define action space (assuming continuous action space)
        self.action_space = np.zeros(action_dim) # np.arange(-max_action_value, max_action_value, 0.1)
        self.action_space_dim = self.action_space.shape[0]

        # Define observation space (assuming continuous state space)
        self.observation_space = np.zeros(state_dim)

        ################## ROS & SIMULATION #########################
        self.start_x = -1.2
        self.start_y = -1.2
        self.start_z = 0
        self.start_yaw = 0
        self.x = self.start_x
        self.y = self.start_y
        self.pre_x = self.start_x
        self.pre_y = self.start_y
        self.z = self.start_z
        self.yaw = self.start_yaw
        self.Width = 4.0
        self.Height = 4.0
        self.diometer_of_env = math.sqrt(self.Width * self.Width + self.Height * self.Height) 
        # self.dist_limit = 3.0
        self.dist = self.Width - 1
        self.pre_dist = self.Width - 1
        rospy.init_node('CustomEnv', anonymous=True)
        rospy.Subscriber("string_message", String, self.callback)
        rospy.wait_for_service('reload_world_service')
        self.reload_world_service = rospy.ServiceProxy("reload_world_service", Empty)
        self.string_publisher = rospy.Publisher('desired', String, queue_size=10)
        self.alarm_area_limit = 0.3
        self.max_distance_from_obstacle = 2000
        self.OUT_OF_ENVIRONMENT = -2
        self.GOAL = 3
        # self.ANGLE_ERROR_ZERO = 0.5
        self.HAS_INTERSECTION = -2
        # self.MAX_EPISODE_STEP = -1
        self.angle_error = 0 #self.calculate_angle_error()
        self.pre_angle_error = 0
        self.is_first_state_info_arrived = False

        self.step_counter = 0

        #grid
        self.local_grid = LocalGridSimple()
        self.local_grid_states = []
        self.is_goal = False

        # for plot
        self.call_grid_steps = []

        # temp
        self.action_count = 0

    def callback(self, data):
        obj = json.loads(data.data)
        self.x = obj['x']
        self.y = obj['y']
        self.z = obj['z']
        self.front = obj['front']
        self.back = obj['back']
        self.left = obj['left']
        self.right = obj['right']
        self.tx = obj['tx']
        self.ty = obj['ty']
        self.timestep = obj['timestep']
        self.yaw = obj['yaw']
        self.obs_01_x = obj['obs_01_x']
        self.obs_01_y = obj['obs_01_y']
        self.obs_02_x = obj['obs_02_x']
        self.obs_02_y = obj['obs_02_y']
        self.obs_03_x = obj['obs_03_x']
        self.obs_03_y = obj['obs_03_y']
        self.obs_04_x = obj['obs_04_x']
        self.obs_04_y = obj['obs_04_y']
        # self.dt = self.timestep / 1000.0  # convert from milliseconds to seconds
        self.is_first_state_info_arrived = True
    
    def publish_desired(self, forward_desired, sideways_desired, yaw_desired):
        message = String()
        data = {
            "forward_desired" : forward_desired,
            "sideways_desired" : sideways_desired,
            "yaw_desired" : yaw_desired,
            "states" : " ".join(map(str, self.state)) 
        }
        json_string = json.dumps(data)
        message.data = json_string
        self.string_publisher.publish(message)
    
    def step(self, action):
        #### action ######
        self.do_actions(action=action)
                
        #### state ######
        s_ = self.get_states()

        #### reward ######
        r = self.get_reward(s_)

        # increasing current step
        self.current_step += 1

        ### done ######
        done = False
        if self.front < 120 or self.right < 120 or self.left < 120:
            rospy.loginfo("The drone has intersection!")
            r += self.HAS_INTERSECTION
            done = True
            print("reward: ", r)
            return s_, r, done, {}
        
        elif self.x > self.Width / 2.0 or self.x < -self.Width / 2.0 or self.y > self.Height / 2.0 or self.y < -self.Height / 2.0:
            rospy.loginfo("The drone goes out of the environment!")
            r += self.OUT_OF_ENVIRONMENT
            done = True
            print("reward: ", r)
            return s_, r, done, {}
        
        elif self.current_step >= self.max_episode_steps:
            rospy.loginfo("Max Episode Steps is happened!")
            # r = self.MAX_EPISODE_STEP
            done = True
            # print("reward: ", r)
            return s_, r, done, {}
        
        # elif self.tx - 0.5 < self.x and self.x < self.tx + 0.5 and self.ty - 0.5 < self.y and self.y < self.ty + 0.5:
        elif  (round(self.min_max_scale(self.angle_error, 0, math.pi/2), 2) < 0.1 and round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2) < 0.2) or (round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2) < 0.1):    
            rospy.loginfo("GOAL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            r = self.GOAL
            done = True
            self.is_goal = True
            print("reward: ", r)
            return s_, r, done, {}
        
        # print("reward: ", r)

        return s_, r, done, {}  # Return a dictionary for any additional info you want to pass
    
    def do_actions(self, action):
        ####################### temp #######################
        real_actions = [-1, 4, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0, 0, 3, 0, 1, 0, 3, 3, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 1, 1, 1, 1, 1, 1, 1, 4, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0, 4, 4, 4, 4, 4, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
        if self.action_count < len(real_actions):
            action = real_actions[self.action_count]
            self.action_count += 1
        else:
            self.publish_desired(str(0), str(0), str(0))
            rospy.sleep(self.dt) # seconds
            self.is_goal = True
            return

        #################################################
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
        vy = 0

        ############################# temp (convert to real drone) ############################
        vyaw *= 30 # degree (-24,-12,0,12,24)
        # vyaw *= 50 # degree (-24,-12,0,12,24)
        # convert to radian
        vyaw = math.radians(vyaw)

        #################################################
        self.publish_desired(str(vx), str(vy), str(vyaw))
        rospy.sleep(self.dt) # seconds
        #################################################

        return
    
    def calculate_angle_error(self):
        # Step 1: Calculate the angle to the target
        target_angle = math.atan2(self.ty - self.y, self.tx - self.x)
        
        # Step 2: Compute the angle difference
        angle_diff = target_angle - self.yaw
        
        # Step 3: Normalize the angle difference to be within the range [0, π]
        # Ensure angle_diff is between -π and π
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        # Take the absolute value to ensure the error is between 0 and π
        angle_error = abs(angle_diff)


        # print("angle error: ", angle_error)
        
        return angle_error, angle_diff
    
    def exponential_func(self, x, c, k, w):
        res = c * np.exp(-k * x)
        # res = c * math.pow(1.0 / math.e , k * x) # f(x) = c . (1 / e) ^ (k.x) 
        return res # self.min_max_scale(res, 0, c) * w # output is [0 , w]
    
    def tanh_func(self, x, w1, w2):
        res = self.min_max_scale(x, 0, 2000) * w1 # output is [0, w1]
        return math.tanh(res) * w2 # res >= 0 so output is [0, w2] 
    
    def get_reward(self, states):
        # w_dist_from_target = 1
        # min_dist = 0.3
        # w_angle_error_agent_target = 1.5
        # w_range_left = 0.1
        # w_range_right = 0.1
        # w_range_front= 0.1
        # w_range_back = 0.1
        # k = np.log(100) / 4 # dist=[0, 4]
        # c = 2 / np.exp(-k * min_dist) # f = [0, 2]
        # if self.dist < self.Width / 4:
        #     reward_dist_from_target = self.exponential_func(x = self.dist, c = c, k = k, w = w_dist_from_target)
        # else:
        #     reward_dist_from_target = 0
        # angle_error_agent_target = self.exponential_func(x = self.angle_error, c = 15, k = 1, w = w_angle_error_agent_target)
        # range_left = self.tanh_func(self.left, w1 = 3, w2 = w_range_left) 
        # range_right = self.tanh_func(self.right, w1 = 3, w2 = w_range_right)
        # range_front = self.tanh_func(self.front, w1 = 3, w2 = w_range_front)
        # range_back = self.tanh_func(self.back, w1 = 3, w2 = w_range_back)
        # r = (reward_dist_from_target + range_left + range_right + range_front + range_back) / (w_dist_from_target + w_range_left + w_range_right + w_range_front + w_range_back)
        # r = reward_dist_from_target
        # r -= 0.001 * self.current_step
        # if self.dist < 1 and  self.dist > 0.8:
        #     r += 0.4
        # elif self.dist <= 0.8 and self.dist > 0.5:
        #     r += 0.6
        # elif self.dist <= 0.5 and self.dist > 0.3:
        #     r += 0.8


        # print("dist_from_target= ", dist_from_target)
        # print("angle_error_agent_target= ", angle_error_agent_target)
        # print("range_left= ", range_left)
        # print("range_right= ", range_right)
        # print("range_front= ", range_front)
        # print("range_back= ", range_back)


        # diff_angle_error = self.pre_angle_error - self.angle_error
        # diff_dist = round(self.pre_dist, 2) - round(self.dist, 2)
        # diff_angle_error = 0 if diff_angle_error > 0 else diff_angle_error

        # if round(self.min_max_scale(self.front, 0, self.max_distance_from_obstacle), 1) <= 0.3 and round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2) >= 0.3:
        #     w_front = 0.6
        #     w_dist = 0.1
        # elif round(self.min_max_scale(self.left, 0, self.max_distance_from_obstacle), 1) <= 0.3 and round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2) >= 0.3:
        #     w_left = 0.6
        #     w_dist = 0.1
        # elif round(self.min_max_scale(self.right, 0, self.max_distance_from_obstacle), 1) <= 0.3 and round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2) >= 0.3:
        #     w_right = 0.6
        #     w_dist = 0.1

        # w1 + w2 + ... + wn = 1
        w_angle_error = 0.1
        w_dist = 0.8
        w_grid = 0.1
        # w_out = 0.2
        # w_front = 0.1
        # w_left = 0.1
        # w_right = 0.1

        # r_front = round(self.min_max_scale(self.front, 0, self.max_distance_from_obstacle), 1) - 1
        # r_left = round(self.min_max_scale(self.left, 0, self.max_distance_from_obstacle), 1) - 1
        # r_right = round(self.min_max_scale(self.right, 0, self.max_distance_from_obstacle), 1) - 1
        r_angle_error = -round(self.min_max_scale(self.angle_error, 0, math.pi), 2)
        r_dist = -round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2)
        # r_out = states[2] - 1
        # r_obs = round(self.min_max_scale(self.get_obs_dist(), 0, 2), 2) - 1
        # print("obs dist= ", self.get_obs_dist())

        state_count = 2
        r_grid = 0
        # print("state= ", states)
        i = state_count
        while i < len(states):
            # angle [-pi/2,pi/2]
            angle_raw = states[i] 
            i += 1
            # distance
            distance_raw = states[i]
            i += 1
            # probability
            p = states[i] 
            i += 1
            if p > 0:
                # obs_angle = 1 - (abs(angle_raw) / (math.pi / 2)) # [-pi/2, pi/2] => angle close to 0, after convert, is close to 1 in [0, 1]
                # r_obs_raw = distance_raw - 1
                r_obs_item = -p #* r_obs_raw #* obs_angle
                r_grid = r_obs_item if r_obs_item < r_grid else r_grid # min
                # print("========= obstacle =============")
                # print("r obs", r_obs_raw)
                # print("p", p)
            elif p < 0:
                # target_angle = 1 - (abs(angle_raw) / (math.pi / 2)) # [-pi/2, pi/2] => angle close to 0, after convert, is close to 1 in [0, 1]
                r_target_raw = 1 - (distance_raw * 0.5) 
                r_target_item = r_target_raw #* target_angle
                r_grid = r_target_item if r_target_item > r_grid else r_grid # max
                # print("========= target =============")
                # print("distance = ", distance_raw)
                # print("target angle", target_angle)
                # print("p", p)


        # r_grid *= 1000
        # if states[i] == 1: # dist to target < 1 meter
        #     w_angle_error = 0.5 
        #     w_dist = 0.5
        #     w_intersect = 0
        #     r += 0.1
        # if r_obs > -0.1:
        #     # r_angle_error = -round(self.min_max_scale(self.angle_error, 0, math.pi), 2)
        #     w_angle_error = 0.5
        #     w_dist = 0.5
        #     w_intersect = 0.0
        r = 0
        r += w_angle_error * r_angle_error
        r += w_dist * r_dist
        # r_1 = r_front if r_front < r_right else r_right 
        # r_2 = r_1 if r_1 < r_left else r_left
        # r += w_intersect * r_2
        r += w_grid * r_grid
        # r += w_out * r_out

        


        # r += w_intersect * r_obs
        # r += w_front * (r_front)
        # r += w_left * (r_left)
        # r += w_right * (r_right)
       
       
       
       
        # r = 0.1 if -0.05 <= self.angle_error <= 0.05 else diff_angle_error
        # r = round(r, 2)
        # r = round(self.min_max_scale(r, -(w1 + w2 + w3), 0), 2)
        # r = round(r - 1, 2)  # This will convert the range from [0, 1] to [-1, 0]
        
        r = round(r,3)
        # print("====== r ================")
        # print("r_angle_error= ", r_angle_error)
        # print("r_dist= ", r_dist)
        # print("r_obstacle= ", r_d)
        # print("r dist  = ", w_dist * r_dist)
        # print("r angle = ", w_angle_error * r_angle_error)
        # print("r grid  = ", w_grid * r_grid)
        # print("r       = ", r)
        # print("dist= ", self.dist)
        return r 
    
    def get_states(self):
        # self.pre_dist = self.dist
        self.dist = math.sqrt(math.pow(self.x - self.tx , 2) + math.pow(self.y - self.ty, 2))
        self.pre_angle_error = self.angle_error
        self.angle_error, target_angle_error = self.calculate_angle_error()
        # print("target angle", round(math.degrees(target_angle_error), 1))

        # self.step_counter += 1
        # if self.step_counter >= 1:
        # self.step_counter = 0
        dx = round(self.x - self.pre_x, 2) # meter
        dy = round(self.y  - self.pre_y, 2) # meter
        self.local_grid_states = self.local_grid.step(dx, dy, self.front, self.left, self.right, self.yaw, self.dist, target_angle_error)
        # print(f"states = local_grid.step({dx}, {dy}, {self.front}, {self.left}, {self.right}, {self.yaw}, {self.dist}, {target_angle})")
        # print("len = ", len(self.local_grid_states))
        self.call_grid_steps.append(f"states = local_grid.step({dx}, {dy}, {self.front}, {self.left}, {self.right}, {self.yaw}, {self.dist}, {target_angle_error})")
        self.pre_x = self.x
        self.pre_y = self.y
        # temp
        # di = int(dx * self.local_grid.grid_value)
        # dj = int(dy * self.local_grid.grid_value)
        # print(F"dx= {dx}, dy= {dy}")
        # print(F"di= {di}, dj= {dj}")
        # # Print the grid row by row, formatting each element to maintain spacing
        # for row in self.local_grid.grid:
        #     formatted_row = ' '.join(f'{elem:.4g}' if elem != 0 else ' ' for elem in row)
        #     print(formatted_row)
        # print("------------------------------------------------------------")

        # state = np.zeros(self.state_dim) # 3k + 2 = static, 2k = dynamic
        # state[1] = round(self.min_max_scale(self.back, 0, self.max_distance_from_obstacle), 1)
        self.state[0] = round(self.min_max_scale(self.angle_error, 0, math.pi), 2)
        self.state[1] = round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2)
        # half_width = self.Width / 2.0
        # half_height = self.Height / 2.0
        # x_y_diff_out = abs(half_width - abs(self.x)) if abs(half_width - abs(self.x)) <= abs(half_height - abs(self.y)) else abs(half_height - abs(self.y)) 
        # x_y_diff_out_max = half_width if half_width <= half_height else half_height
        # self.state[2] = round(self.min_max_scale(x_y_diff_out, 0, x_y_diff_out_max), 1)

        # for dynamic obstacle
        # self.step_counter = self.step_counter + 1
        # state_index = 2
        # if self.step_counter > 4:
        #     self.step_counter = 0
        
        # if self.step_counter == 1:
        #     state_index = 2
        # elif self.step_counter == 2:
        #     state_index = 6
        # elif self.step_counter == 3:
        #     state_index = 10
        # elif self.step_counter == 4:
        #     state_index = 14
        
        # temp_data = [[math.pi/2, 140],[math.pi/2, 140]]            
        # if len(self.local_grid_states) == 1 :
        #     temp_data[0][0] = self.local_grid_states[0][0]
        #     temp_data[0][1] = self.local_grid_states[0][1]
        # if len(self.local_grid_states) == 2 :
        #     temp_data = self.local_grid_states
        
        # self.state[state_index] = round(temp_data[0][0], 2) # angle 1
        # self.state[state_index + 1] = round(self.min_max_scale(temp_data[0][1], 0, 140), 2) # distance 1
        # self.state[state_index + 2] = round(temp_data[1][0], 2) # angle 2
        # self.state[state_index + 3] = round(self.min_max_scale(temp_data[1][1], 0, 140), 2) # distance 2

        
        state_count = 2
        local_grid_index = 0
        while(state_count + 3 <= self.state_dim):
            if local_grid_index < len(self.local_grid_states):
                # angle [0 , 2 * pi]
                self.state[state_count] = round(self.local_grid_states[local_grid_index][0], 2) 
                state_count += 1
                # distance [0 , 14]
                self.state[state_count] = round(self.min_max_scale(self.local_grid_states[local_grid_index][1], 0, self.local_grid.diameter), 2)
                state_count += 1
                # probability [0 , 1.0]
                self.state[state_count] = round(self.local_grid_states[local_grid_index][2], 3)
                state_count += 1
                local_grid_index += 1
            else: 
                break
        
        # self.state[state_count] = 1 if self.dist <= 1.0 else 0

        # # fill rest of states by default values
        # while(state_count + 3 <= self.state_dim):
        #     state[state_count] = math.pi/2
        #     state_count += 1
        #     state[state_count] = 1
        #     state_count += 1
        #     state[state_count] = 0
        #     state_count += 1


        # state[2] = self.latent_vector[0]
        # state[3] = self.latent_vector[1]
        # state[4] = self.latent_vector[2]
        # state[5] = self.latent_vector[3]
        # state[6] = self.latent_vector[4]
        # state[7] = self.latent_vector[5]
        # state[8] = self.latent_vector[6]
        # state[9] = self.latent_vector[7]
        # state[10] = self.latent_vector[8]
        # state[11] = self.latent_vector[9]
        # state = np.concatenate((state, latent_vector), axis=0) # 10 states
        # state[2] = round(self.min_max_scale(self.front, 0, self.max_distance_from_obstacle), 1)
        # state[3] = round(self.min_max_scale(self.right, 0, self.max_distance_from_obstacle), 1)
        # state[4] = round(self.min_max_scale(self.left, 0, self.max_distance_from_obstacle), 1)
        
        
        # state[2] = round(self.min_max_scale(self.get_obs_dist(), 0, 2), 2)
        # state[0] = 1 if self.front < 250 and self.front > 100 else 0
        # state[1] = 1 if self.back < 250 and self.back > 100 else 0
        # state[2] = 1 if self.right < 250 and self.right > 100 else 0
        # state[3] = 1 if self.left < 250 and self.left > 100 else 0
        # state[4] = round(self.angle_error, 2)

        # print("front:", self.front)
        # print("Current state:", self.state[4])
        
        # note: previous trained model doese not work because of adding these new states.
        # state[6] = round(self.min_max_scale(self.x, -self.Width / 2.0, self.Width / 2.0), 1)
        # state[7] = round(self.min_max_scale(self.y, -self.Height / 2.0, self.Height / 2.0), 1)
        # state[8] = round(self.min_max_scale(self.tx, -self.Width / 2.0, self.Width / 2.0), 1)
        # state[9] = round(self.min_max_scale(self.ty, -self.Height / 2.0, self.Height / 2.0), 1)

        return self.state
    
    def min_max_scale(self, value, min_val, max_val):
        value = np.clip(value, min_val, max_val)
        scaled_value = (value - min_val) / (max_val - min_val)
        return scaled_value

    def reset(self):
        self.reload_world_service()
        # waiting_time = 5
        # print(f"waiting for drone hovering, {waiting_time} seconds")
        # rospy.sleep(waiting_time) # seconds
        # rospy.loginfo("Hovering is DONE!")

        while not self.is_first_state_info_arrived:
            # print("MRR")
            rospy.spin()

        # temp
        self.action_count = 0

        self.is_first_state_info_arrived = False
        self.current_step = 0
        self.orientation = 0

        self.local_grid = LocalGridSimple()
        self.local_grid_states = []

        self.local_grid.reset()
        self.step_counter = 0
        # dx = self.x - self.pre_x # meter
        # dy = self.y  - self.pre_y # meter
        # self.local_grid_states = self.local_grid.step(dx, dy, self.front, self.left, self.right, self.yaw, self.dist, )
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
    
    def is_safe_area(self, s):
        # False: drone is in alarm or dangerous area (left or right side), True: drone is in safe area
        if s[0] <= self.alarm_area_limit:
            return False
        
        return True
    
    def save_model(self, filepath):
        self.local_grid.save_model(filepath)
    
    def load_model(self, filepath):
        self.local_grid.load_model(filepath)
    
    # def seed(self, seed):
    #     np.random.seed(seed)

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