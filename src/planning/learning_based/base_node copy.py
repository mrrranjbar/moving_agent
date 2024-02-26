#!/usr/bin/env python

# from PPO_Continues.PPO_continuous_main import Base_PPO
# import argparse

# if __name__ == '__main__':
#     parser = argparse.ArgumentParser("Hyperparameters Setting for PPO-continuous")
#     parser.add_argument("--max_train_steps", type=int, default=int(3e6), help=" Maximum number of training steps")
#     parser.add_argument("--evaluate_freq", type=float, default=5e3, help="Evaluate the policy every 'evaluate_freq' steps")
#     parser.add_argument("--save_freq", type=int, default=20, help="Save frequency")
#     parser.add_argument("--policy_dist", type=str, default="Gaussian", help="Beta or Gaussian")
#     parser.add_argument("--batch_size", type=int, default=2048, help="Batch size")
#     parser.add_argument("--mini_batch_size", type=int, default=64, help="Minibatch size")
#     parser.add_argument("--hidden_width", type=int, default=64, help="The number of neurons in hidden layers of the neural network")
#     parser.add_argument("--lr_a", type=float, default=3e-4, help="Learning rate of actor")
#     parser.add_argument("--lr_c", type=float, default=3e-4, help="Learning rate of critic")
#     parser.add_argument("--gamma", type=float, default=0.99, help="Discount factor")
#     parser.add_argument("--lamda", type=float, default=0.95, help="GAE parameter")
#     parser.add_argument("--epsilon", type=float, default=0.2, help="PPO clip parameter")
#     parser.add_argument("--K_epochs", type=int, default=10, help="PPO parameter")
#     parser.add_argument("--use_adv_norm", type=bool, default=True, help="Trick 1:advantage normalization")
#     parser.add_argument("--use_state_norm", type=bool, default=True, help="Trick 2:state normalization")
#     parser.add_argument("--use_reward_norm", type=bool, default=False, help="Trick 3:reward normalization")
#     parser.add_argument("--use_reward_scaling", type=bool, default=True, help="Trick 4:reward scaling")
#     parser.add_argument("--entropy_coef", type=float, default=0.01, help="Trick 5: policy entropy")
#     parser.add_argument("--use_lr_decay", type=bool, default=True, help="Trick 6:learning rate Decay")
#     parser.add_argument("--use_grad_clip", type=bool, default=True, help="Trick 7: Gradient clip")
#     parser.add_argument("--use_orthogonal_init", type=bool, default=True, help="Trick 8: orthogonal initialization")
#     parser.add_argument("--set_adam_eps", type=float, default=True, help="Trick 9: set Adam epsilon=1e-5")
#     parser.add_argument("--use_tanh", type=float, default=True, help="Trick 10: tanh activation function")

#     args = parser.parse_args()

#     env_name = ['BipedalWalker-v3', 'HalfCheetah-v2', 'Hopper-v2', 'Walker2d-v2']
#     env_index = 1
#     base_PPO = Base_PPO()
#     base_PPO.main(args, env_name=env_name[env_index], number=1, seed=10)



import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import json
import random  

x = 0
y = 0 
z = 0
def callback(data):
    global x,y,z
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    obj = json.loads(data.data)
    # rospy.loginfo(f"x={obj['x']}, y={obj['y']}, z={obj['z']}")
    x = obj['x']
    y = obj['y']
    z = obj['z']


if __name__ == '__main__':
    x_limit = 4.0
    dist_limit = 3.0
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("string_message", String, callback)
    rospy.wait_for_service('plus_forward_desired_service')
    client_plus_forward_desired_service = rospy.ServiceProxy("plus_forward_desired_service", Empty)
    rospy.wait_for_service('minus_forward_desired_service')
    client_minus_forward_desired_service = rospy.ServiceProxy("minus_forward_desired_service", Empty)
    rospy.wait_for_service('stop_forward_desired_service')
    client_stop_forward_desired_service = rospy.ServiceProxy("stop_forward_desired_service", Empty)
    rospy.wait_for_service('reload_world_service')
    reload_world_service = rospy.ServiceProxy("reload_world_service", Empty)

    episod_count = 0
    while(True):
        episod_count = episod_count + 1
        rospy.loginfo(f"Episod {episod_count} is running!")
        rospy.loginfo(f"Waition for 6 seconds, drone is hovering!")
        rospy.sleep(6) # seconds => for drone hovering

        dist = dist_limit - x
        while(True):
            if x <= dist_limit :
                dist = dist_limit - x # reward = -distance
            else:
                dist = x - dist_limit
            rospy.loginfo(f"reward: {-dist}")
            n = random.random()
            if n > 0.5:
                client_plus_forward_desired_service()
                rospy.loginfo(f"plus is choosed!")
            else:
                client_minus_forward_desired_service()
                rospy.loginfo(f"minus is choosed!")

            rospy.sleep(1) # seconds

            rospy.loginfo(f"x={x}, y={y}, z={z}")
            if x > x_limit or x < -x_limit:
                client_stop_forward_desired_service()
                rospy.loginfo(f"stop is choosed!")
                reload_world_service()
                break

    
    # client_plus_forward_desired_service()
    # client_minus_forward_desired_service()
    # client_stop_forward_desired_service()
    rospy.spin()
    
    