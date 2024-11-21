import torch
import numpy as np
# from torch.utils.tensorboard import SummaryWriter
# import gym
# import argparse
from PPO_DISCRETE.normalization import Normalization, RewardScaling
from PPO_DISCRETE.replaybuffer import ReplayBuffer
from PPO_DISCRETE.ppo_discrete import PPO_discrete

from env import CustomEnv

import wandb
import os
import json
from datetime import datetime


class Base_PPO_Discrete():
    def evaluate_policy(self, args, env, agent, state_norm):
        times = 3
        evaluate_reward = 0
        for episode_i in range(times):
            s = env.reset()
            if args.use_state_norm:  # During the evaluating,update=False
                s = state_norm(s, update=False)
            done = False
            episode_reward = 0
            while not done:
                a = agent.evaluate(s)  # We use the deterministic policy during the evaluating
                s_, r, done, _ = env.step(a)
                if args.use_state_norm:
                    s_ = state_norm(s_, update=False)
                episode_reward += r
                s = s_
            print(f"reward of episode{episode_i + 1}: {episode_reward}")
            evaluate_reward += episode_reward

        return evaluate_reward / times
    
    def convert_to_serializable(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        # elif isinstance(obj, np.ndarray):
        #     return obj.tolist()
        else:
            return obj
    
    def run_model(self, args, env, agent, state_norm, actor_path, critic_path):
        # Get the current date and time
        current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # Build the result file path
        result_path = os.path.join(args.absolute_dir, 'result', f'res-{current_time}.txt')
        agent.load_models(actor_path, critic_path)
        # env.load_model(grid_model_path)
        print("models are LOADED!!!")
        if args.use_state_norm:
            state_norm_path = os.path.join(os.path.dirname(critic_path), "norm_params.pkl")
            state_norm.load_params(state_norm_path)
            print("state norm is loaded!!!")
        list_of_data = []
        while(True):
            s = env.reset()
            s_un_norm = s
            if args.use_state_norm:
                s = state_norm(s, update=False)

            done = False
            episode_reward = 0
            # agent_xy.clear()
            # agent_xy.append([env.x, env.y])
            list_of_data.clear()
            data = {
                "obs_01" : [env.obs_01_x, env.obs_01_y],
                "obs_02" : [env.obs_02_x, env.obs_02_y],
                "obs_03" : [env.obs_03_x, env.obs_03_y],
                "obs_04" : [env.obs_04_x, env.obs_04_y],
                "obs_05" : [env.obs_05_x, env.obs_05_y],
                "obs_06" : [env.obs_06_x, env.obs_06_y],
                "obs_07" : [env.obs_07_x, env.obs_07_y],
                "obs_08" : [env.obs_08_x, env.obs_08_y],
                "obs_09" : [env.obs_09_x, env.obs_09_y],
                "obs_10" : [env.obs_10_x, env.obs_10_y],
                "agent": [env.x, env.y],
                "current_step_un_norm" : s_un_norm.tolist(),
                "current_step_norm" : s.tolist(),
                "next_step_un_norm" : s_un_norm.tolist(),
                "next_step_norm" : s.tolist(),
                "action": -1,
                "reward": -100,
                "front": env.front,
                "left": env.left,
                "right": env.right,
                "yaw": env.yaw,
                "roll": env.roll,
                "pitch": env.pitch,
                "v_x": env.v_x,
                "v_y": env.v_y,
                "yaw_rate": env.yaw_rate,
            }
            list_of_data.append(data)
            while not done:
                a = agent.evaluate(s)  # We use the deterministic policy during the evaluating
                s_, r, done, _ = env.step(a)
                s_next_un_norm = s_
                # agent_xy.append([env.x, env.y])
                if args.use_state_norm:
                    s_ = state_norm(s_, update=False)
                data = {
                    "obs_01" : [env.obs_01_x, env.obs_01_y],
                    "obs_02" : [env.obs_02_x, env.obs_02_y],
                    "obs_03" : [env.obs_03_x, env.obs_03_y],
                    "obs_04" : [env.obs_04_x, env.obs_04_y],
                    "obs_05" : [env.obs_05_x, env.obs_05_y],
                    "obs_06" : [env.obs_06_x, env.obs_06_y],
                    "obs_07" : [env.obs_07_x, env.obs_07_y],
                    "obs_08" : [env.obs_08_x, env.obs_08_y],
                    "obs_09" : [env.obs_09_x, env.obs_09_y],
                    "obs_10" : [env.obs_10_x, env.obs_10_y],
                    "agent": [env.x, env.y],
                    "current_step_un_norm" : s_un_norm.tolist(),
                    "current_step_norm" : s.tolist(),
                    "next_step_un_norm" : s_next_un_norm.tolist(),
                    "next_step_norm" : s_.tolist(),
                    "action": a,
                    "reward": r,
                    "front": env.front,
                    "left": env.left,
                    "right": env.right,
                    "yaw": env.yaw,
                    "roll": env.roll,
                    "pitch": env.pitch,
                    "v_x": env.v_x,
                    "v_y": env.v_y,
                    "yaw_rate": env.yaw_rate,
                }
                list_of_data.append(data)
                # print("Current action:", a)
                # print("Current state:", s_)
                # print("Reward: ", r)
                episode_reward += r
                s = s_
                s_un_norm = s_next_un_norm
            # s = env.reset() # just for refreshing the simulator
            print("episode_reward:{} \t".format(episode_reward))
            if env.is_goal:
                # Ensure result directory exists
                if not os.path.exists(os.path.dirname(result_path)):
                    os.makedirs(os.path.dirname(result_path))

                # Save agent and obstacle coordinates
                with open(result_path, 'w') as file:
                    for data in list_of_data:
                        # file.write(f"{agent_pos[0]},{agent_pos[1]},{obstacle_pos[0]},{obstacle_pos[1]}\n")
                        file.write(f"{json.dumps(data, default=self.convert_to_serializable)}\n")
                    file.write("CallForGridSteps\n")
                    step = 0
                    for data in env.call_grid_steps:
                        file.write(f"{data}\n")
                        file.write(f"show({step})\n")
                        step += 1

                # print("Agent positions saved:", agent_xy)
                # print("Obstacle positions saved:", obstacle_xy)
                break
        
        ss = env.reset()
        return


    def main(self, args, env_name, number, seed):
        actor_path = args.absolute_dir + '/model/actor_model.pth'  # Specify the paths where you want to save the models
        critic_path = args.absolute_dir + '/model/critic_model.pth'
        # grid_model_path = args.absolute_dir + '/model/grid_model.pth'
        env = CustomEnv(state_dim=args.state_dim, action_dim=args.action_dim, max_episode_steps=args.max_episode_steps, max_action_value=0, velocity=args.velocity, wheel_base_length=args.wheel_base_length)
        env_evaluate = CustomEnv(state_dim=args.state_dim, action_dim=args.action_dim, max_episode_steps=args.max_episode_steps, max_action_value=0, velocity=args.velocity, wheel_base_length=args.wheel_base_length)

        # Set random seed
        # env.seed(seed)
        # env.action_space.seed(seed)
        # env_evaluate.seed(seed)
        # env_evaluate.action_space.seed(seed)
        np.random.seed(seed)
        torch.manual_seed(seed)

        # args.state_dim = env.observation_space.shape[0]
        # args.action_dim = env.action_space.n
        # args.max_episode_steps = env._max_episode_steps  # Maximum number of steps per episode
        print("env={}".format(env_name))
        print("state_dim={}".format(args.state_dim))
        print("action_dim={}".format(args.action_dim))
        print("max_episode_steps={}".format(args.max_episode_steps))
        print("max_train_steps={}".format(args.max_train_steps))

        is_wandb = args.is_wandb


        evaluate_num = 0  # Record the number of evaluations
        evaluate_rewards = []  # Record the rewards during the evaluating
        total_steps = 0  # Record the total steps during the training

        replay_buffer = ReplayBuffer(args)
        agent = PPO_discrete(args)

        if args.is_transform_training:
            agent.load_models(actor_path, critic_path, False)
            print("Trained model is LOADED!!!")

        # Build a tensorboard
        # writer = SummaryWriter(log_dir='runs/PPO_discrete/env_{}_number_{}_seed_{}'.format(env_name, number, seed))

        state_norm = Normalization(shape=args.state_dim)  # Trick 2:state normalization
        if args.use_reward_norm:  # Trick 3:reward normalization
            reward_norm = Normalization(shape=1)
        elif args.use_reward_scaling:  # Trick 4:reward scaling
            reward_scaling = RewardScaling(shape=1, gamma=args.gamma)

        if args.is_run_mode :
            self.run_model(args=args, env=env, agent=agent, state_norm=state_norm, actor_path=actor_path, critic_path=critic_path)
            return
        
        if is_wandb:
            # start a new wandb run to track this script
            wandb.init(
                # set the wandb project where this run will be logged
                project="Crazyflie-DRL",

                # track hyperparameters and run metadata
                config={
                "state_dim": args.state_dim,
                "action_dim": args.action_dim,
                "max_episode_steps": args.max_episode_steps,
                "max_train_steps": args.max_train_steps,
                }
            )
        total_episodes = 0
        total_goals = 0
        actor_loss_data_counter = 0
        critic_loss_data_counter = 0
        while total_steps < args.max_train_steps:
            print("NEW EPISODE!!!!!!!!")
            print("total_steps={}".format(total_steps))
            print("total episodes={}".format(total_episodes))
            print("total goals={}".format(total_goals))
            s = env.reset()
            if args.use_state_norm:
                s = state_norm(s)
            if args.use_reward_scaling:
                reward_scaling.reset()
            episode_steps = 0
            total_episodes += 1
            sum_reward = 0
            sum_raw_reward = 0
            done = False
            while not done:
                episode_steps += 1
                a, a_logprob = agent.choose_action(s)  # Action and the corresponding log probability
                s_, r, done, _ = env.step(a)

                # simulation bug
                if r < -2:
                    break
                if env.is_goal:
                    total_goals += 1
                
                sum_raw_reward += r

                if is_wandb:
                    wandb.log({"raw rewards": r, "steps counter" : episode_steps})

                if args.use_state_norm:
                    s_ = state_norm(s_)
                if args.use_reward_norm:
                    r = reward_norm(r)
                elif args.use_reward_scaling:
                    r = reward_scaling(r)
                
                sum_reward += r

                if is_wandb:
                    wandb.log({"scaled rewards": r, "steps counter" : episode_steps})

                # print("Current action:", a)
                # print("Current state:", s_)
                # print("scaled reward: ", r)


                # When dead or win or reaching the max_episode_steps, done will be Ture, we need to distinguish them;
                # dw means dead or win,there is no next state s';
                # but when reaching the max_episode_steps,there is a next state s' actually.
                if done and episode_steps != args.max_episode_steps:
                    dw = True
                else:
                    dw = False

                replay_buffer.store(s, a, a_logprob, r, s_, dw, done)
                s = s_
                total_steps += 1

                # When the number of transitions in buffer reaches batch_size,then update
                if replay_buffer.count == args.batch_size:
                    actor_loss_data, critic_loss_data = agent.update(replay_buffer, total_steps)
                    if is_wandb:
                        for los in actor_loss_data:
                            wandb.log({"actor loss": los, "actor loss counter" : actor_loss_data_counter})
                            actor_loss_data_counter += 1
                        for los in critic_loss_data:
                            wandb.log({"critic loss": los, "critic loss counter" : critic_loss_data_counter})
                            critic_loss_data_counter += 1
                    print("network is UPDATED!!!")
                    replay_buffer.count = 0

                # Evaluate the policy every 'evaluate_freq' steps
                if total_steps % args.evaluate_freq == 0:
                    evaluate_num += 1
                    print(f"================ Evaluation =================")
                    evaluate_reward = self.evaluate_policy(args, env_evaluate, agent, state_norm)
                    evaluate_rewards.append(evaluate_reward)
                    print("evaluate_num:{} \t evaluate_reward:{} \t".format(evaluate_num, evaluate_reward))
                    goal_percentage = (total_goals/total_episodes) * 100
                    print(f"GOAL percentage: {goal_percentage}")
                    # writer.add_scalar('step_rewards_{}'.format(env_name), evaluate_rewards[-1], global_step=total_steps)
                    # Save the rewards MRR
                    file_path = args.absolute_dir + '/data_train/PPO_discrete_env_{}_number_{}_seed_{}.npy'.format(env_name, number, seed)
                    # Extract the directory path from the file path
                    directory = os.path.dirname(file_path)
                    if not os.path.exists(directory):
                        os.makedirs(directory)
                    np.save(file_path, np.array(evaluate_rewards)) #MRR
                    # if evaluate_num % args.save_freq == 0:
                    #     if not os.path.exists(directory):
                    #         os.makedirs(directory)  
                    #     np.save(file_path, np.array(evaluate_rewards))
                    if not os.path.exists(os.path.dirname(actor_path)):
                        os.makedirs(os.path.dirname(actor_path))
                    if not os.path.exists(os.path.dirname(critic_path)):
                        os.makedirs(os.path.dirname(critic_path))
                    # if not os.path.exists(os.path.dirname(grid_model_path)):
                    #     os.makedirs(os.path.dirname(grid_model_path))
                    agent.save_models(actor_path, critic_path)
                    # env.save_model(grid_model_path)
                    print("models are SAVED!!!")
                    if args.use_state_norm:
                        state_norm_path = os.path.join(os.path.dirname(critic_path), "norm_params.pkl")
                        state_norm.save_params(state_norm_path)
                        print("state norm is SAVED!!!")
                
            if is_wandb:
                wandb.log({"avg episode scaled reward": sum_reward/episode_steps, "episode counter" : total_episodes})
                wandb.log({"avg episode raw reward": sum_raw_reward/episode_steps, "episode counter" : total_episodes})
            
            goal_percentage = (total_goals/total_episodes) * 100
            print(f"GOAL percentage: {goal_percentage}")
            
        if not os.path.exists(os.path.dirname(actor_path)):
            os.makedirs(os.path.dirname(actor_path))
        if not os.path.exists(os.path.dirname(critic_path)):
            os.makedirs(os.path.dirname(critic_path))
        # if not os.path.exists(os.path.dirname(grid_model_path)):
        #     os.makedirs(os.path.dirname(grid_model_path))
        agent.save_models(actor_path, critic_path)
        # env.save_model(grid_model_path)
        print("LAST Model is SAVED!!!")
        if args.use_state_norm:
            state_norm_path = os.path.join(os.path.dirname(critic_path), "norm_params.pkl")
            state_norm.save_params(state_norm_path)
            print("state norm is SAVED!!!")
        ss = env.reset()