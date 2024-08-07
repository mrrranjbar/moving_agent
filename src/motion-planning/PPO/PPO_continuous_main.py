import torch
import numpy as np
from torch.utils.tensorboard import SummaryWriter
import gym
import argparse
from PPO.normalization import Normalization, RewardScaling
from PPO.replaybuffer import ReplayBuffer
from PPO.ppo_continuous import PPO_continuous

# import wandb

from env import CustomEnv

class Base_PPO():
    def evaluate_policy(self, args, env, agent, state_norm):
        times = 3
        evaluate_reward = 0
        for _ in range(times):
            s = env.reset()
            if args.use_state_norm:
                s = state_norm(s, update=False)  # During the evaluating,update=False
            done = False
            episode_reward = 0
            while not done:
                a = agent.evaluate(s)  # We use the deterministic policy during the evaluating
                if args.policy_dist == "Beta":
                    action = 2 * (a - 0.5) * args.max_action  # [0,1]->[-max,max]
                else:
                    action = a
                s_, r, done, _ = env.step(action)
                if args.use_state_norm:
                    s_ = state_norm(s_, update=False)
                episode_reward += r
                s = s_
            evaluate_reward += episode_reward

        return evaluate_reward / times
    
    def run_model(self, args, env, agent, actor_path, critic_path):
        agent.load_models(actor_path, critic_path)
        print("model is LOADED!!!")
        s = env.reset()
        done = False
        episode_reward = 0
        while not done:
            a = agent.evaluate(s)  # We use the deterministic policy during the evaluating
            if args.policy_dist == "Beta":
                action = 2 * (a - 0.5) * args.max_action  # [0,1]->[-max,max]
            else:
                action = a
            s_, r, done, _ = env.step(action)
            print("Current state:", s_)
            episode_reward += r
            s = s_
        s = env.reset() # just for refreshing the simulator
        print("episode_reward:{} \t".format(episode_reward))
        return


    def main(self, args, env_name, number, seed):
        # wandb.init(project='rl_ppo_crazyflie', entity='mrr-ranjbar')
        actor_path = args.absolute_dir + '/model/actor_model.pth'  # Specify the paths where you want to save the models
        critic_path = args.absolute_dir + '/model/critic_model.pth'
        
        env = CustomEnv(state_dim=args.state_dim, action_dim=args.action_dim, max_episode_steps=args.max_episode_steps, max_action_value=args.max_action_value, velocity=args.velocity, wheel_base_length=args.wheel_base_length)
        # env = gym.make(env_name)
        # env_evaluate = gym.make(env_name)  # When evaluating the policy, we need to rebuild an environment
        # Set random seed
        # env.seed(seed)
        # env.action_space.seed(seed)
        # env_evaluate.seed(seed)
        # env_evaluate.action_space.seed(seed)
        np.random.seed(seed)
        torch.manual_seed(seed)

        args.state_dim = env.observation_space.shape[0]
        args.action_dim = env.action_space.shape[0]
        args.max_action = float(env.action_space_high)
        args.max_episode_steps = env._max_episode_steps  # Maximum number of steps per episode
        print("env={}".format(env_name))
        print("state_dim={}".format(args.state_dim))
        print("action_dim={}".format(args.action_dim))
        print("max_action={}".format(args.max_action))
        print("max_episode_steps={}".format(args.max_episode_steps))
        print("max_train_steps={}".format(args.max_train_steps))

        evaluate_num = 0  # Record the number of evaluations
        evaluate_rewards = []  # Record the rewards during the evaluating
        total_steps = 0  # Record the total steps during the training

        replay_buffer = ReplayBuffer(args)
        agent = PPO_continuous(args)
        if args.is_transform_training:
            agent.load_models(actor_path, critic_path)
            print("Trained model is LOADED!!!")
        

        # Build a tensorboard
        writer = SummaryWriter(log_dir= args.absolute_dir + '/runs/PPO_continuous/env_{}_{}_number_{}_seed_{}'.format(env_name, args.policy_dist, number, seed))

        state_norm = Normalization(shape=args.state_dim)  # Trick 2:state normalization
        if args.use_reward_norm:  # Trick 3:reward normalization
            reward_norm = Normalization(shape=1)
        elif args.use_reward_scaling:  # Trick 4:reward scaling
            reward_scaling = RewardScaling(shape=1, gamma=args.gamma)

        if args.is_run_mode :
            self.run_model(args=args, env=env, agent=agent, actor_path=actor_path, critic_path=critic_path)
            return
        
        is_first_update = True
        is_agent_in_alarm_area = False
        direc = ""

        while total_steps < args.max_train_steps:
            print("NEW EPISODE!!!!!!!!")
            print("total_steps={}".format(total_steps))
            s = env.reset()
            if args.use_state_norm:
                s = state_norm(s)
            if args.use_reward_scaling:
                reward_scaling.reset()
            episode_steps = 0
            done = False
            while not done:
                episode_steps += 1
                if args.check_safty and is_agent_in_alarm_area:
                    action, a_logprob = agent.choose_action_from_custom_controller(s, direc)
                else:
                    if args.using_guidance and is_first_update:
                        a, a_logprob = agent.choose_action_for_first_update(s)
                        # print("choose_action_for_first_update!")
                    else:
                        a, a_logprob = agent.choose_action(s)  # Action and the corresponding log probability
                        # print("choose_action_normal!")
                    if args.policy_dist == "Beta":
                        action = 2 * (a - 0.5) * args.max_action  # [0,1]->[-max,max]
                    else:
                        action = a
                s_, r, done, _ = env.step(action)
                print("Current state:", s_)
                if args.check_safty:
                    is_agent_in_safe_area, direc = env.is_safe_area(s_)
                    is_agent_in_alarm_area = not is_agent_in_safe_area
                
                # #wandb
                # wandb.log({"step": total_steps, "reward": r})

                if args.use_state_norm:
                    s_ = state_norm(s_)
                if args.use_reward_norm:
                    r = reward_norm(r)
                elif args.use_reward_scaling:
                    r = reward_scaling(r)

                # When dead or win or reaching the max_episode_steps, done will be Ture, we need to distinguish them;
                # dw means dead or win,there is no next state s';
                # but when reaching the max_episode_steps,there is a next state s' actually.
                if done and episode_steps != args.max_episode_steps:
                    dw = True 
                else:
                    dw = False

                # Take the 'action'，but store the original 'a'（especially for Beta）
                replay_buffer.store(s, a, a_logprob, r, s_, dw, done)
                s = s_
                total_steps += 1

                #wandb
                # wandb.log({"reward": r})

                # When the number of transitions in buffer reaches batch_size,then update
                if replay_buffer.count == args.batch_size:
                    if args.using_guidance and is_first_update:
                        is_first_update = False
                    agent.update(replay_buffer, total_steps)
                    print("network is UPDATED!!!")
                    replay_buffer.count = 0

                # Evaluate the policy every 'evaluate_freq' steps
                if total_steps % args.evaluate_freq == 0:
                    # evaluate_num += 1
                    # evaluate_reward = self.evaluate_policy(args, env, agent, state_norm)
                    # evaluate_rewards.append(evaluate_reward)
                    # print("evaluate_num:{} \t evaluate_reward:{} \t".format(evaluate_num, evaluate_reward))
                    # writer.add_scalar('step_rewards_{}'.format(env_name), evaluate_rewards[-1], global_step=total_steps)
                    # Save the rewards MRR
                    import os
                    file_path = args.absolute_dir + '/data_train/PPO_continuous_{}_env_{}_number_{}_seed_{}.npy'.format(args.policy_dist, env_name, number, seed)
                    # Extract the directory path from the file path
                    directory = os.path.dirname(file_path)
                    # if evaluate_num % args.save_freq == 0:
                    if not os.path.exists(directory):
                        os.makedirs(directory)  
                    np.save(file_path, np.array(evaluate_rewards))

                    if not os.path.exists(os.path.dirname(actor_path)):
                        os.makedirs(os.path.dirname(actor_path))
                    if not os.path.exists(os.path.dirname(critic_path)):
                        os.makedirs(os.path.dirname(critic_path))
                    agent.save_models(actor_path, critic_path)
                    print("model is SAVED!!!")
