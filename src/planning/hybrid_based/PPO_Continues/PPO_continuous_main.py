import torch
import numpy as np
from torch.utils.tensorboard import SummaryWriter
import gym
import argparse
from PPO_Continues.normalization import Normalization, RewardScaling
from PPO_Continues.replaybuffer import ReplayBuffer
from PPO_Continues.ppo_continuous import PPO_continuous
from PPO_Continues.classic_approach.Process import Process
import math
from torch.distributions import Normal
import os
import random
import time

# import wandb
from PPO_Continues.env import CustomShortPathEnv
from PPO_Continues.classic_approach.GUI import Preparation

class Base_PPO():
    def evaluate_policy(self, args, env, agent, state_norm):
        times = 3
        evaluate_reward = 0
        for _ in range(times):
            s = env.reset(True)
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

        # env.show()
        return evaluate_reward / times
    
    def run_model(self, args, env, agent, actor_path, critic_path, state_norm):
        agent.load_models(actor_path, critic_path)
        print("model is LOADED!!!")
        s = env.reset(True)
        if args.use_state_norm:
            s = state_norm(s, update=False)  # During the evaluating,update=False
        done = False
        episode_reward = 0
        r = -100
        while not done:
            a = agent.evaluate(s)  # We use the deterministic policy during the evaluating
            if args.policy_dist == "Beta":
                action = 2 * (a - 0.5) * args.max_action  # [0,1]->[-max,max]
            else:
                action = a
            s_, r, done, _ = env.step(action, is_run = True)
            if args.use_state_norm:
                s_ = state_norm(s_, update=False)
            episode_reward += r
            s = s_
        if r == env.GOAL:
            env.show()
            print("episode_reward:{} \t".format(episode_reward))
            return True
        
        return False
    
    def solve_with_classic_approach(self, env):
        path_exist = False
        proc = Process()
        proc.initialize(env.rings, env.Start, env.min_leg_length, env.max_turning_angle - 10, env.Width, env.Height)

        proc.MainLoop()
        path = proc.MainFunction(env.Target)

        # print("result path is: ")
        # for pt in path:
        #     print("X:", pt.x, "Y:", pt.y)

        actions = []
        if len(path) > 1 :
            path_exist = True
            first_angle = env.find_angle(path[0], path[1]) # [0, 359]
            if first_angle > 180:
                first_angle = - (360 - first_angle)
            first_angle = self.clip_number(first_angle, -env.max_turning_angle, env.max_turning_angle)
            first_angle = self.scale_to_minus_one_one(first_angle, -env.max_turning_angle, env.max_turning_angle)
            env.StartAngle = env.calc_start_angle() # 0
            l = env.find_length(path[0], path[1])
            l = self.clip_number(l, env.min_leg_length, env.max_leg_length)
            length = self.scale_to_minus_one_one(l, env.min_leg_length, env.max_leg_length)
            action = np.array([length, first_angle])
            actions.append(action)
            # convert path to actions
            for i in range(1, len(path) - 1):
                a1 = env.find_angle(path[i - 1], path[i])
                a2 = env.find_angle(path[i], path[i + 1])
                turning_status = 1 if proc.ccw(path[i - 1], path[i], path[i + 1]) == 1 else -1 # 1: ccw, -1:cw
                abs_a = abs(a1 - a2)
                if abs_a > 180:
                    abs_a = 360 - abs_a
                angle = turning_status * abs_a
                angle = self.clip_number(angle, -env.max_turning_angle, env.max_turning_angle)
                angle = self.scale_to_minus_one_one(angle, -env.max_turning_angle, env.max_turning_angle)
                l = env.find_length(path[i], path[i + 1])
                l = self.clip_number(l, env.min_leg_length, env.max_leg_length)
                length = self.scale_to_minus_one_one(l, env.min_leg_length, env.max_leg_length)
                action = np.array([length, angle])
                actions.append(action)
        del proc
        return path_exist, actions
    
    def scale_to_minus_one_one(self, original_value, min_val, max_val):
        scaled_value = 2 * (original_value - min_val) / (max_val - min_val) - 1
        return scaled_value
    
    def clip_number(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)
    
    def run_classic_approach(self, env):
        preparation = Preparation()
        env.Path, env.rings, env.Start, env.Target = preparation.main(env.min_leg_length, env.max_turning_angle - 10, env.Width, env.Height)
        env.show()
        return


    def main(self, args, env_name, number, seed):
        # wandb.init(project='rl_ppo_crazyflie', entity='mrr-ranjbar')
        actor_path = args.absolute_dir + '/model/actor_model.pth'  # Specify the paths where you want to save the models
        critic_path = args.absolute_dir + '/model/critic_model.pth'
        
        env = CustomShortPathEnv(state_dim=args.state_dim, action_dim=args.action_dim, max_episode_steps=args.max_episode_steps, max_turning_angle=args.max_turning_angle, min_leg_length=args.min_leg_length)
        if args.run_classic_approach:
            start_time = time.time()
            self.run_classic_approach(env)
            end_time = time.time()
            duration = (end_time - start_time) * 1000  # convert to milliseconds
            print("(Classic) => Runtime is: ", duration, "milliseconds")
            return
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
        args.max_turning_angle = int(env.max_turning_angle)
        args.min_leg_length = int(env.min_leg_length)
        args.max_episode_steps = env._max_episode_steps  # Maximum number of steps per episode
        print("env={}".format(env_name))
        print("state_dim={}".format(args.state_dim))
        print("action_dim={}".format(args.action_dim))
        # print("max_action={}".format(args.max_action))
        print("max_episode_steps={}".format(args.max_episode_steps))
        print("max_train_steps={}".format(args.max_train_steps))

        evaluate_num = 0  # Record the number of evaluations
        evaluate_rewards = []  # Record the rewards during the evaluating
        total_steps = 0  # Record the total steps during the training

        replay_buffer = ReplayBuffer(args)
        # replay_buffer_temp = ReplayBuffer(args)
        agent = PPO_continuous(args)

        # Build a tensorboard
        writer = SummaryWriter(log_dir= args.absolute_dir + '/runs/PPO_continuous/env_{}_{}_number_{}_seed_{}'.format(env_name, args.policy_dist, number, seed))

        state_norm = Normalization(shape=args.state_dim)  # Trick 2:state normalization
        if args.use_reward_norm:  # Trick 3:reward normalization
            reward_norm = Normalization(shape=1)
        elif args.use_reward_scaling:  # Trick 4:reward scaling
            reward_scaling = RewardScaling(shape=1, gamma=args.gamma)

        if args.is_run_mode :
            for i in range(args.max_number_of_run):
                start_time = time.time()
                run_res = self.run_model(args=args, env=env, agent=agent, actor_path=actor_path, critic_path=critic_path, state_norm=state_norm)
                end_time = time.time()
                if  run_res:
                    duration = (end_time - start_time) * 1000  # convert to milliseconds
                    print("(Hybrid) => Runtime is: ", duration, "milliseconds")
                    return
            return
        
        is_first_update = True
        is_agent_in_alarm_area = False
        direc = ""
        # is_classic_approach_used = False

########################################## 1 ###############################################
        while total_steps < args.max_train_steps:
            print("NEW EPISODE======================")
            print("total_steps={}".format(total_steps))
            if total_steps % 1000 == 0:
                env.show()
            s = env.reset()
            if args.use_state_norm:
                s = state_norm(s)
            if args.use_reward_scaling:
                reward_scaling.reset()
            episode_steps = 0
            done = False
            # replay_buffer_temp.count = 0
            
            is_solved_with_classic_approach = False
            is_classic_approach_used = False
            random_number = random.random()

            limit_value = 1
            if total_steps <= (args.max_train_steps * 0.05):
                limit_value = 0.9
            elif total_steps <= (args.max_train_steps * 0.2) :
                limit_value = 0.7
            elif total_steps <= (args.max_train_steps * 0.3) :
                limit_value = 0.6
            elif total_steps <= (args.max_train_steps * 0.4) :
                limit_value = 0.5
            elif total_steps <= (args.max_train_steps * 0.5) :
                limit_value = 0.2
            elif total_steps <= (args.max_train_steps * 0.7) :
                limit_value = 0.05
            else :
                limit_value = 0
            # if total_steps <= (args.max_train_steps / 2) :
            if 0 < random_number < limit_value :
                is_classic_approach_used = True # not is_classic_approach_used
            # elif total_steps > (args.max_train_steps / 10) and total_steps < (args.max_train_steps / 2) :
            #     is_classic_approach_used = not is_classic_approach_used
            # else:
                # is_classic_approach_used = False

############################################### 2 ###############################################
            if is_classic_approach_used :
                is_solved_with_classic_approach, classic_actions = self.solve_with_classic_approach(env)
                #    s = env.reset()
                reward_temp = 0
                if args.use_state_norm:
                    s = state_norm(s)
                for a in classic_actions: # start episode
                    episode_steps += 1
                    s_, r, done, _ = env.step(a, solved_by_classic_method=True)
                    reward_temp = r
                    if r == env.OUT_OF_ENVIRONMENT:
                        env.reset(True)
                        break

                    # mrr
                    dist_now = agent.actor.get_dist(torch.unsqueeze(torch.tensor(s, dtype=torch.float), 0))
                    a_logprob = dist_now.log_prob(torch.tensor(a, dtype=torch.float))
                    a_logprob = a_logprob.detach().numpy().flatten()

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

                    # When the number of transitions in buffer reaches batch_size,then update
                    if replay_buffer.count == args.batch_size:
                        agent.update(replay_buffer, total_steps)
                        print("network is UPDATED!!!")
                        replay_buffer.count = 0

                    # Evaluate the policy every 'evaluate_freq' steps
                    if total_steps % args.evaluate_freq == 0:
                        if not os.path.exists(os.path.dirname(actor_path)):
                            os.makedirs(os.path.dirname(actor_path))
                        if not os.path.exists(os.path.dirname(critic_path)):
                            os.makedirs(os.path.dirname(critic_path))
                        agent.save_models(actor_path, critic_path)
                        print("model is SAVED!!!")  
                    
                if reward_temp != env.GOAL:
                    env.reset(True)
                   
################################################ 3 #########################################################
            while not done and not is_solved_with_classic_approach:
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

                # replay_buffer_temp.store(s, a, a_logprob, r, s_, dw, done)
                # Take the 'action'，but store the original 'a'（especially for Beta）
                # if r == env.GOAL :
                #     s_t, a_t, a_logprob_t, r_t, s__t, dw_t, done_t = replay_buffer_temp.get_numpy()
                #     for i in range(replay_buffer_temp.count):
                #         replay_buffer.store(s_t[i], a_t[i], a_logprob_t[i], r_t[i], s__t[i], dw_t[i], done_t[i]) 
                #         # When the number of transitions in buffer reaches batch_size,then update
                #         if replay_buffer.count == args.batch_size:
                #             if args.using_guidance and is_first_update:
                #                 is_first_update = False
                #             agent.update(replay_buffer, total_steps)
                #             print("network is UPDATED!!!")
                #             replay_buffer.count = 0
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
                    evaluate_num += 1
                    evaluate_reward = self.evaluate_policy(args, env, agent, state_norm)
                    evaluate_rewards.append(evaluate_reward)
                    print("evaluate_num:{} \t evaluate_reward:{} \t".format(evaluate_num, evaluate_reward))
                    writer.add_scalar('step_rewards_{}'.format(env_name), evaluate_rewards[-1], global_step=total_steps)
                    # Save the rewards MRR
                    
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

                # file_path = '/home/mohammad/catkin_ws/src/moving_agent/src/planning/hybrid_based/PPO_Continues/log_rewards.txt'
                # with open(file_path, 'a') as file:
                #     # Write content to the file
                #     # log_content = F'=========== New Step ===========================\n'
                #     log_content = F'reward = {r}\n'
                #     file.write(log_content)
