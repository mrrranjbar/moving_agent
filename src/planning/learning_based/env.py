#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import json
import math 

import numpy as np

class CustomEnv:
    def __init__(self, state_dim, action_dim, max_episode_steps, max_action_value):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        self.state = None
        self.max_action_value = max_action_value

        # Define action space (assuming continuous action space)
        self.action_space = np.zeros(action_dim) # np.arange(-max_action_value, max_action_value, 0.1)
        self.action_space_dim = self.action_space.shape[0]

        # Define observation space (assuming continuous state space)
        self.observation_space = np.zeros(state_dim)

        ################## ROS & SIMULATION #########################
        self.x = 0
        self.y = 0 
        self.z = 0
        self.Width = 4.0
        self.Height = 4.0
        self.diometer_of_env = math.sqrt(self.Width * self.Width + self.Height * self.Height) 
        # self.dist_limit = 3.0
        self.dist = self.Width - 1
        rospy.init_node('CustomEnv', anonymous=True)
        rospy.Subscriber("string_message", String, self.callback)
        rospy.wait_for_service('reload_world_service')
        self.reload_world_service = rospy.ServiceProxy("reload_world_service", Empty)
        self.string_publisher = rospy.Publisher('desired', String, queue_size=10)
        self.alarm_area_limit = 0.5
        self.max_distance_from_obstacle = 2000
        self.OUT_OF_ENVIRONMENT = -2
        self.GOAL = 2
        self.HAS_INTERSECTION = -2
        self.MAX_EPISODE_STEP = -2
        


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
        if self.x > self.Width or self.x < -self.Width or self.y > self.Height or self.y < -self.Height:
            rospy.loginfo("The drone has intersection!")
            r = self.HAS_INTERSECTION
            done = True
            return s_, r, done, {}
        
        elif self.x > self.Width or self.x < -self.Width or self.y > self.Height or self.y < -self.Height:
            rospy.loginfo("The drone goes out of the environment!")
            r = self.OUT_OF_ENVIRONMENT
            done = True
            return s_, r, done, {}
        
        elif self.current_step >= self.max_episode_steps:
            rospy.loginfo("Max Episode Steps is happened!")
            r = self.MAX_EPISODE_STEP
            done = True
            return s_, r, done, {}
        
        elif self.tx - 0.2 < self.x and self.x < self.tx + 0.2 and self.ty - 0.2 < self.y and self.y < self.ty + 0.2:
            rospy.loginfo("GOAL!!!!!!!!!!!!!!!!!!!!!!!!!!")
            r = self.GOAL
            done = True
            return s_, r, done, {}
        
        return s_, r, done, {}  # Return a dictionary for any additional info you want to pass
    
    def do_actions(self, action):
        vx = action[0] 
        vx = ((vx + 1) / 2) * (self.max_action_value + self.max_action_value) - self.max_action_value 

        vy = action[1] 
        vy = ((vy + 1) / 2) * (self.max_action_value + self.max_action_value) - self.max_action_value 

        vyaw = action[2]
        vyaw = ((vyaw + 1) / 2) * (self.max_action_value + self.max_action_value) - self.max_action_value 


        self.publish_desired(str(vx), str(vy), str(vyaw))
        rospy.sleep(0.2) # seconds

        return
    
    def get_reward(self):
        temp_coef = 1 if self.front <= 0.2 else self.front
        inverse_distance_for_front = (1.0 / temp_coef) * self.max_distance_from_obstacle
        inverse_distance_for_front = 0 if inverse_distance_for_front == 1 else inverse_distance_for_front

        temp_coef = 1 if self.back <= 0.2 else self.back
        inverse_distance_for_back = (1.0 / temp_coef) * self.max_distance_from_obstacle
        inverse_distance_for_back = 0 if inverse_distance_for_back == 1 else inverse_distance_for_back

        temp_coef = 1 if self.right <= 0.2 else self.right
        inverse_distance_for_right = (1.0 / temp_coef) * self.max_distance_from_obstacle
        inverse_distance_for_right = 0 if inverse_distance_for_right == 1 else inverse_distance_for_right

        temp_coef = 1 if self.left <= 0.2 else self.left
        inverse_distance_for_left = (1.0 / temp_coef) * self.max_distance_from_obstacle
        inverse_distance_for_left = 0 if inverse_distance_for_left == 1 else inverse_distance_for_left

        w_current_step = 1.0
        w_dist = 1.0
        w_inverse_distance_for_front = 1.0
        w_inverse_distance_for_back = 1.0
        w_inverse_distance_for_right = 1.0
        w_inverse_distance_for_left = 1.0
        sum =  (
               self.min_max_scale_and_weight(self.current_step, 0, self.max_episode_steps, w_current_step) + 
               self.min_max_scale_and_weight(self.dist, 0, self.diometer_of_env, w_dist) + 
               self.min_max_scale_and_weight(inverse_distance_for_front, 0, self.max_distance_from_obstacle, w_inverse_distance_for_front) + 
               self.min_max_scale_and_weight(inverse_distance_for_back, 0, self.max_distance_from_obstacle, w_inverse_distance_for_back) + 
               self.min_max_scale_and_weight(inverse_distance_for_right, 0, self.max_distance_from_obstacle, w_inverse_distance_for_right) + 
               self.min_max_scale_and_weight(inverse_distance_for_left, 0, self.max_distance_from_obstacle, w_inverse_distance_for_left)
               )
        r = - (sum / (w_inverse_distance_for_front + w_inverse_distance_for_back + w_inverse_distance_for_right + w_inverse_distance_for_left))

        return r
    
    def get_states(self):
        self.dist = math.sqrt(math.pow(self.x - self.tx , 2) + math.pow(self.y - self.ty, 2))
        state = np.zeros(self.state_dim) 
        state[0] = round(self.min_max_scale_and_weight(self.dist, 0, self.diometer_of_env, 1), 2)
        state[1] = round(self.min_max_scale_and_weight(self.front, 0, self.max_distance_from_obstacle, 1), 1)
        state[2] = round(self.min_max_scale_and_weight(self.back, 0, self.max_distance_from_obstacle, 1), 1)
        state[3] = round(self.min_max_scale_and_weight(self.right, 0, self.max_distance_from_obstacle, 1), 1)
        state[4] = round(self.min_max_scale_and_weight(self.left, 0, self.max_distance_from_obstacle, 1), 1)

        return state
    
    def min_max_scale_and_weight(self, value, min_val, max_val, weight):
        scaled_value = (value - min_val) / (max_val - min_val)
        return scaled_value * weight

    def reset(self):
        self.reload_world_service()
        rospy.loginfo("waiting for drone hovering, 10 seconds")
        rospy.sleep(10) # seconds
        rospy.loginfo("Hovering is DONE!")
        self.current_step = 0
        self.state = np.zeros(self.state_dim) 
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
    



# # Example usage:
# if __name__ == "__main__":
#     state_dim = 4  # Replace with the dimension of your state space
#     action_dim = 1  # Replace with the dimension of your action space
#     max_episode_steps = 100  # Replace with your desired maximum episode steps

#     env = CustomEnv(state_dim, action_dim, max_episode_steps)
#     s = env.reset()
#     action = 0.5  # Replace with your chosen action
#     s_, r, done, _ = env.step(action)

#     print("State:", s)
#     print("Next State:", s_)
#     print("Reward:", r)
#     print("Done:", done)
#     print("Observation Space Shape:", env.observation_space_shape)
#     print("Action Space Shape:", env.action_space_shape)
#     print("Action Space High:", env.action_space_high)
#     print("Max Episode Steps:", env._max_episode_steps)










#######################################################################################
# x = 0
# y = 0 
# z = 0
# def callback(data):
#     global x,y,z
#     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     obj = json.loads(data.data)
#     # rospy.loginfo(f"x={obj['x']}, y={obj['y']}, z={obj['z']}")
#     x = obj['x']
#     y = obj['y']
#     z = obj['z']

# def publish_forward_desired(forward_desired):
#     string_publisher = rospy.Publisher('forward_desired', String, queue_size=10)
#     message = String()
#     message.data = forward_desired
#     string_publisher.publish(message)


# if __name__ == '__main__':
#     x_limit = 4.0
#     dist_limit = 3.0
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("string_message", String, callback)
#     rospy.wait_for_service('reload_world_service')
#     reload_world_service = rospy.ServiceProxy("reload_world_service", Empty)

#     episod_count = 0
#     while(True):
#         episod_count = episod_count + 1
#         rospy.loginfo(f"Episod {episod_count} is running!")
#         rospy.loginfo(f"Waition for 6 seconds, drone is hovering!")
#         rospy.sleep(6) # seconds => for drone hovering

#         dist = dist_limit - x
#         while(True):
#             if x <= dist_limit :
#                 dist = dist_limit - x # reward = -distance
#             else:
#                 dist = x - dist_limit
#             rospy.loginfo(f"reward: {-dist}")
#             n = random.random()
#             if n > 0.5:
#                 publish_forward_desired("0.5")
#                 rospy.loginfo(f"plus is choosed!")
#             else:
#                 publish_forward_desired("-0.5")
#                 rospy.loginfo(f"minus is choosed!")

#             rospy.sleep(1) # seconds

#             rospy.loginfo(f"x={x}, y={y}, z={z}")
#             if x > x_limit or x < -x_limit:
#                 publish_forward_desired("0")
#                 rospy.loginfo(f"stop is choosed!")
#                 reload_world_service()
#                 break

#     rospy.spin()
    