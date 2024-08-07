#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import json
import math 
import random

import numpy as np

class CustomEnv:
    def __init__(self, state_dim, action_dim, max_episode_steps, max_action_value, velocity, wheel_base_length):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        self.state = None
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
        self.x = 0
        self.y = 0 
        self.z = 0
        self.Width = 8.0
        self.Height = 8.0
        self.diometer_of_env = self.Width # math.sqrt(self.Width * self.Width + self.Height * self.Height) 
        # self.dist_limit = 3.0
        self.dist = self.Width - 1
        self.pre_dist = self.Width - 1
        rospy.init_node('CustomEnv', anonymous=True)
        rospy.Subscriber("string_message", String, self.callback)
        rospy.wait_for_service('reload_world_service')
        self.reload_world_service = rospy.ServiceProxy("reload_world_service", Empty)
        self.string_publisher = rospy.Publisher('desired', String, queue_size=10)
        self.alarm_area_limit = 0.5
        self.max_distance_from_obstacle = 2000
        self.OUT_OF_ENVIRONMENT = -2
        self.GOAL = 3
        self.ANGLE_ERROR_ZERO = 0.5
        self.HAS_INTERSECTION = -2
        self.MAX_EPISODE_STEP = -2
        self.angle_error = 0 #self.calculate_angle_error()
        self.pre_angle_error = 0
        self.is_first_state_info_arrived = False

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
        # self.dt = self.timestep / 1000.0  # convert from milliseconds to seconds
        self.is_first_state_info_arrived = True
    
    def publish_desired(self, forward_desired, sideways_desired, yaw_desired):
        message = String()
        data = {
            "forward_desired" : forward_desired,
            "sideways_desired" : sideways_desired,
            "yaw_desired" : yaw_desired
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
        r = self.get_reward()

        # increasing current step
        self.current_step += 1

        ### done ######
        done = False
        if self.front < 100 or self.back < 100 or self.right < 100 or self.left < 100:
            rospy.loginfo("The drone has intersection!")
            r = self.HAS_INTERSECTION
            done = True
            # print("reward: ", r)
            return s_, r, done, {}
        
        # elif self.x > self.Width or self.x < -self.Width or self.y > self.Height or self.y < -self.Height:
        #     rospy.loginfo("The drone goes out of the environment!")
        #     r = self.OUT_OF_ENVIRONMENT
        #     done = True
        #     return s_, r, done, {}
        
        elif self.current_step >= self.max_episode_steps:
            rospy.loginfo("Max Episode Steps is happened!")
            # r = self.MAX_EPISODE_STEP
            done = True
            # print("reward: ", r)
            return s_, r, done, {}
        
        elif self.tx - 0.3 < self.x and self.x < self.tx + 0.3 and self.ty - 0.3 < self.y and self.y < self.ty + 0.3:
            rospy.loginfo("GOAL!!!!!!!!!!!!!!!!!!!!!!!!!!")
            # r = self.GOAL
            done = True
            # print("reward: ", r)
            return s_, r, done, {}
        
        # print("reward: ", r)

        return s_, r, done, {}  # Return a dictionary for any additional info you want to pass
    
    def do_actions(self, action):
        # vx = action[0] 
        # vx = ((vx + 1) / 2) * (self.max_action_value + self.max_action_value) - self.max_action_value 

        # vy = action[1] 
        # vy = ((vy + 1) / 2) * (self.max_action_value + self.max_action_value) - self.max_action_value 

        # vyaw = action[2]
        # vyaw = ((vyaw + 1) / 2) * (self.max_action_value + self.max_action_value) - self.max_action_value 

        # steering_angle = action[0]
        # steering_angle = ((steering_angle + 1) / 2) * (self.max_action_value + self.max_action_value) - self.max_action_value # (-math.pi / 4.0, math.pi / 4.0)
        # vyaw = (self.velocity / self.wheel_base_length) * math.tan(steering_angle)
        # Update orientation angle
        vyaw = action[0] #(-1,1)
        # vyaw *= self.ANGULAR_VELOCITY_COEF
        vyaw = ((vyaw + 1) / 2) * (self.max_action_value + self.max_action_value) - self.max_action_value # (-max_action_value, max_action_value)
        # print("angular velocity: ", vyaw)
        # self.orientation += vyaw * self.dt

        # Constrain orientation angle to ensure vx remains positive and not zero
        # Keep the orientation angle strictly within (-pi/2, pi/2)
        # if self.orientation >= math.pi / 2:
        #     self.orientation = math.pi / 2 - 0.01  # slightly less than pi/2
        # elif self.orientation <= -math.pi / 2:
        #     self.orientation = -math.pi / 2 + 0.01  # slightly more than -pi/2

        vx = self.velocity
        # print("linear velocity: ", vx)
        vy = 0
        # vx = self.velocity * math.cos(self.orientation)
        # vy = self.velocity * math.sin(self.orientation)
        # vy = 0
        # transfer to the robot frame


        # print("timestep: ", self.timestep)
        # print("steering angle: ", steering_angle)
        # print("tan(steering_angle): ", math.tan(steering_angle))
        # print("orientation: ", self.orientation)
        # print("cos(self.orientation): ", math.cos(self.orientation))
        # print("sin(self.orientation): ", math.sin(self.orientation))
        # print("Linear velocity: ", self.velocity)
        # print("vx: ", vx)
        # print("vy: ", vy)
        # print("vyaw (angular velocity): ", vyaw)


        self.publish_desired(str(vx), str(vy), str(vyaw))
        rospy.sleep(self.dt) # seconds

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
        
        return angle_error
    
    def exponential_func(self, x, c, k, w):
        res = c * np.exp(-k * x)
        # res = c * math.pow(1.0 / math.e , k * x) # f(x) = c . (1 / e) ^ (k.x) 
        return res # self.min_max_scale(res, 0, c) * w # output is [0 , w]
    
    def tanh_func(self, x, w1, w2):
        res = self.min_max_scale(x, 0, 2000) * w1 # output is [0, w1]
        return math.tanh(res) * w2 # res >= 0 so output is [0, w2] 
    
    def get_reward(self):
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
        # if self.angle_error < 0.05: # < 3 degrees
        #     r += self.ANGLE_ERROR_ZERO

        # print("dist_from_target= ", dist_from_target)
        # print("angle_error_agent_target= ", angle_error_agent_target)
        # print("range_left= ", range_left)
        # print("range_right= ", range_right)
        # print("range_front= ", range_front)
        # print("range_back= ", range_back)


        # diff_angle_error = self.pre_angle_error - self.angle_error
        # diff_dist = round(self.pre_dist, 2) - round(self.dist, 2)
        # diff_angle_error = 0 if diff_angle_error > 0 else diff_angle_error

        r = -round(self.min_max_scale(self.angle_error, 0, math.pi), 2)
        # r = 0.1 if -0.05 <= self.angle_error <= 0.05 else diff_angle_error
        # r = round(r, 2)
        print("reward= ", r)
        # print("dist= ", self.dist)
        return r # [0, 1]
    
    def get_states(self):
        # self.pre_dist = self.dist
        # self.dist = math.sqrt(math.pow(self.x - self.tx , 2) + math.pow(self.y - self.ty, 2))
        self.pre_angle_error = self.angle_error
        self.angle_error = self.calculate_angle_error()
        state = np.zeros(self.state_dim) 
        state[0] = round(self.min_max_scale(self.front, 0, self.max_distance_from_obstacle), 1)
        state[1] = round(self.min_max_scale(self.back, 0, self.max_distance_from_obstacle), 1)
        state[2] = round(self.min_max_scale(self.right, 0, self.max_distance_from_obstacle), 1)
        state[3] = round(self.min_max_scale(self.left, 0, self.max_distance_from_obstacle), 1)
        state[4] = round(self.min_max_scale(self.angle_error, 0, math.pi), 2)
        # state[5] = round(self.min_max_scale(self.dist, 0, self.diometer_of_env), 2)

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

        return state
    
    def min_max_scale(self, value, min_val, max_val):
        scaled_value = (value - min_val) / (max_val - min_val)
        return scaled_value

    def reset(self):
        self.reload_world_service()
        waiting_time = 8
        rospy.loginfo(f"waiting for drone hovering, {waiting_time} seconds")
        rospy.sleep(waiting_time) # seconds
        rospy.loginfo("Hovering is DONE!")

        while not self.is_first_state_info_arrived:
            # print("MRR")
            rospy.spin()

        self.current_step = 0
        self.orientation = 0
        self.state = np.zeros(self.state_dim) 
        self.state = self.get_states()
        return self.state
    
    def is_safe_area(self, s):
        # False: drone is in alarm or dangerous area (left or right side), True: drone is in safe area
        if s[0] > self.Width - self.alarm_area_limit:
            return False, "right"
        elif s[0] < -self.Width + self.alarm_area_limit:
            return False, "left"
        else:
            return True, ""
    
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