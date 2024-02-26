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
        self.x_limit = 4.0
        self.dist_limit = 3.0
        self.dist = self.dist_limit - self.x
        rospy.init_node('CustomEnv', anonymous=True)
        rospy.Subscriber("string_message", String, self.callback)
        rospy.wait_for_service('reload_world_service')
        self.reload_world_service = rospy.ServiceProxy("reload_world_service", Empty)
        self.string_publisher = rospy.Publisher('forward_desired', String, queue_size=10)
        self.alarm_area_limit = 0.5
        


    def callback(self, data):
        obj = json.loads(data.data)
        self.x = obj['x']
        self.y = obj['y']
        self.z = obj['z']
    
    def publish_forward_desired(self, forward_desired):
        message = String()
        message.data = forward_desired
        self.string_publisher.publish(message)

    def step(self, action):
        done = False
        if self.current_step >= self.max_episode_steps:
            rospy.loginfo("Max Episode Steps is happened!")
            done = True
            # raise Exception("Episode has already ended. Call reset() to start a new episode.")

        # Implement your custom step logic here.
        self.publish_forward_desired(str(action[0]))
        rospy.sleep(0.2) # seconds
        
        # Compute the next state (s_), reward (r), and done flag (done) based on the given action.
        ####state######
        s_ = np.zeros(self.state_dim)  
        s_[0] = self.x

        #### reward ######
        # note: reward = -exp(distance)
        if self.x <= self.dist_limit :
            self.dist = self.dist_limit - self.x 
        else:
            self.dist = self.x - self.dist_limit
        r = -math.exp(self.dist)

        ### done ######
        # alarm area
        if self.x > self.x_limit - self.alarm_area_limit or self.x < -self.x_limit + self.alarm_area_limit:
            rospy.loginfo("The drone goes in the ALARM area!")
            r = -(math.exp(self.dist) * 1.2)
            
        if self.x > self.x_limit or self.x < -self.x_limit:
            # self.publish_forward_desired("0")
            # rospy.loginfo(f"stop is choosed!")
            # self.reload_world_service()
            rospy.loginfo("The drone goes out of the environment!")
            r = -(math.exp(self.dist) * 1.3)
            done = True
        
        if  self.dist_limit - 0.2 < self.x and self.x < self.dist_limit + 0.2:
            rospy.loginfo("GOAL!!!!!!!!!!!!!!!!!!!!!!!!!!")
            r = 0
            done = True

        self.current_step += 1

        return s_, r, done, {}  # Return a dictionary for any additional info you want to pass

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
        if s[0] > self.x_limit - self.alarm_area_limit:
            return False, "right"
        elif s[0] < -self.x_limit + self.alarm_area_limit:
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
    