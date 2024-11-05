import torch
import numpy as np
from PPO_DISCRETE.normalization import Normalization
from PPO_DISCRETE.ppo_discrete import PPO_discrete
from env_real import CustomRealEnv
import os
import json

############
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig

import math


class Base_PPO_Discrete_Real():

    def log_data_callback(self, timestamp, data, logconf):
            self.current_x = data['stateEstimate.x']
            self.current_y = data['stateEstimate.y']
            self.current_z = data['stateEstimate.z']
            self.current_yaw = math.radians(data['stateEstimate.yaw'])

    def run_model(self, args, env, agent, state_norm, actor_path, critic_path):
        result_path = args.absolute_dir + '/result/res.txt'
        agent.load_models(actor_path, critic_path)
        print("models are LOADED!!!")
        if args.use_state_norm:
            state_norm_path = os.path.join(os.path.dirname(critic_path), "norm_params.pkl")
            state_norm.load_params(state_norm_path)
            print("state norm is loaded!!!")
        list_of_data = []

        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers(enable_debug_driver=False)
        cf = Crazyflie(rw_cache='./cache')

        # Add the log configuration for x, y, z, yaw
        log_config = LogConfig(name='Position', period_in_ms=50)
        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        log_config.add_variable('stateEstimate.z', 'float')
        log_config.add_variable('stateEstimate.yaw', 'float')

        with SyncCrazyflie(self.URI, cf=cf) as scf:
            cf.log.add_config(log_config)
            log_config.data_received_cb.add_callback(self.log_data_callback)
            log_config.start()
            with MotionCommander(scf, 0.2) as motion_commander:
                with Multiranger(scf) as multi_ranger:
                    # motion_commander.start_linear_motion(0, 0, 0, 0)
                    print(f"x: {self.current_x}")
                    print(f"y: {self.current_y}")
                    print(f"z: {self.current_z}")
                    print(f"yaw: {self.current_yaw}")
                    print("DRONE is Taking off...")
                    print("Sleep for 1 seconds...")
                    time.sleep(1)
                    print(f"x: {self.current_x}")
                    print(f"y: {self.current_y}")
                    print(f"z: {self.current_z}")
                    print(f"yaw: {self.current_yaw}")
                    s = env.reset(multi_ranger, self.current_x, self.current_y, self.current_z, self.current_yaw)
                    print(f"front: {env.front}")
                    print(f"left: {env.left}")
                    print(f"right: {env.right}")
                    s_un_norm = s
                    if args.use_state_norm:
                        s = state_norm(s, update=False)

                    done = False
                    episode_reward = 0
                    list_of_data.clear()
                    data = {
                        # "obs_01" : [env.obs_01_x, env.obs_01_y],
                        # "obs_02" : [env.obs_02_x, env.obs_02_y],
                        # "obs_03" : [env.obs_03_x, env.obs_03_y],
                        # "obs_04" : [env.obs_04_x, env.obs_04_y],
                        "agent": [env.x, env.y],
                        "current_step_un_norm" : s_un_norm.tolist(),
                        "current_step_norm" : s.tolist(),
                        "next_step_un_norm" : s_un_norm.tolist(),
                        "next_step_norm" : s.tolist(),
                        "action": -1,
                        "reward": -100
                    }
                    list_of_data.append(data)
                    ii = 0
                    while not done:
                        # ii += 1
                        # motion_commander.start_linear_motion(0.05, 0, 0, 80)
                        # time.sleep(0.05)
                        a = agent.evaluate(s)  # We use the deterministic policy during the evaluating
                        s_, r, done, _ = env.step(a, motion_commander, multi_ranger, self.current_x, self.current_y, self.current_z, self.current_yaw)
                        s_next_un_norm = s_
                        if args.use_state_norm:
                            s_ = state_norm(s_, update=False)
                        data = {
                            # "obs_01" : [env.obs_01_x, env.obs_01_y],
                            # "obs_02" : [env.obs_02_x, env.obs_02_y],
                            # "obs_03" : [env.obs_03_x, env.obs_03_y],
                            # "obs_04" : [env.obs_04_x, env.obs_04_y],
                            "agent": [env.x, env.y],
                            "current_step_un_norm" : s_un_norm.tolist(),
                            "current_step_norm" : s.tolist(),
                            "next_step_un_norm" : s_next_un_norm.tolist(),
                            "next_step_norm" : s_.tolist(),
                            "action": a,
                            "reward": r
                        }
                        list_of_data.append(data)
                        # print("Current action:", a)
                        # print("Current state:", s_)
                        # print("Reward: ", r)
                        episode_reward += r
                        s = s_
                        s_un_norm = s_next_un_norm
                        print(f"x: {self.current_x}")
                        print(f"y: {self.current_y}")
                        print(f"z: {self.current_z}")
                        print(f"yaw: {self.current_yaw}")
                        print(f"front: {env.front}")
                        print(f"left: {env.left}")
                        print(f"right: {env.right}")
        
                    print(f"is done = {done}")
                    print(f"is goal = {env.is_goal}")
        print("episode_reward:{} \t".format(episode_reward))

        # Ensure result directory exists
        if not os.path.exists(os.path.dirname(result_path)):
            os.makedirs(os.path.dirname(result_path))

        # Save agent and obstacle coordinates
        with open(result_path, 'w') as file:
            for data in list_of_data:
                file.write(f"{json.dumps(data, default=self.convert_to_serializable)}\n")
            file.write("CallForGridSteps\n")
            step = 0
            for data in env.call_grid_steps:
                file.write(f"{data}\n")
                file.write(f"show({step})\n")
                step += 1
        return
    
    def convert_to_serializable(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        else:
            return obj


    def main(self, args, env_name, seed):
        self.URI = 'radio://0/80/2M/E7E7E7E705'
        actor_path = args.absolute_dir + '/model/actor_model.pth'  # Specify the paths where you want to save the models
        critic_path = args.absolute_dir + '/model/critic_model.pth'
        env = CustomRealEnv(state_dim=args.state_dim, max_episode_steps=args.max_episode_steps, velocity=args.velocity)
        np.random.seed(seed)
        torch.manual_seed(seed)
        print("env={}".format(env_name))
        print("state_dim={}".format(args.state_dim))
        print("action_dim={}".format(args.action_dim))
        print("max_episode_steps={}".format(args.max_episode_steps))
        print("max_train_steps={}".format(args.max_train_steps))
        agent = PPO_discrete(args)
        state_norm = Normalization(shape=args.state_dim)  # Trick 2:state normalization
        self.run_model(args=args, env=env, agent=agent, state_norm=state_norm, actor_path=actor_path, critic_path=critic_path)