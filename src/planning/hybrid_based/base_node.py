#!/usr/bin/env python
from PPO_Continues.PPO_continuous_main import Base_PPO
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser("Hyperparameters Setting for PPO-continuous")
    parser.add_argument("--max_train_steps", type=int, default=int(2e6), help=" Maximum number of training steps") # 3e6 => MRR: 5000
    parser.add_argument("--evaluate_freq", type=float, default=2e4, help="Evaluate the policy every 'evaluate_freq' steps") # 5e3 => MRR: 5000
    parser.add_argument("--save_freq", type=int, default=20, help="Save frequency") # not using
    parser.add_argument("--policy_dist", type=str, default="Gaussian", help="Beta or Gaussian")
    parser.add_argument("--batch_size", type=int, default=2048, help="Batch size") #2048 => MRR:512
    parser.add_argument("--mini_batch_size", type=int, default=64, help="Minibatch size")
    parser.add_argument("--hidden_width", type=int, default=64, help="The number of neurons in hidden layers of the neural network")
    parser.add_argument("--lr_a", type=float, default=3e-4, help="Learning rate of actor")
    parser.add_argument("--lr_c", type=float, default=3e-4, help="Learning rate of critic")
    parser.add_argument("--gamma", type=float, default=0.99, help="Discount factor")
    parser.add_argument("--lamda", type=float, default=0.95, help="GAE parameter")
    parser.add_argument("--epsilon", type=float, default=0.2, help="PPO clip parameter")
    parser.add_argument("--K_epochs", type=int, default=30, help="PPO parameter") #10
    parser.add_argument("--use_adv_norm", type=bool, default=True, help="Trick 1:advantage normalization")
    parser.add_argument("--use_state_norm", type=bool, default=False, help="Trick 2:state normalization")
    parser.add_argument("--use_reward_norm", type=bool, default=False, help="Trick 3:reward normalization")
    parser.add_argument("--use_reward_scaling", type=bool, default=False, help="Trick 4:reward scaling")
    parser.add_argument("--entropy_coef", type=float, default=0.01, help="Trick 5: policy entropy")
    parser.add_argument("--use_lr_decay", type=bool, default=True, help="Trick 6:learning rate Decay")
    parser.add_argument("--use_grad_clip", type=bool, default=True, help="Trick 7: Gradient clip")
    parser.add_argument("--use_orthogonal_init", type=bool, default=True, help="Trick 8: orthogonal initialization")
    parser.add_argument("--set_adam_eps", type=float, default=True, help="Trick 9: set Adam epsilon=1e-5")
    parser.add_argument("--use_tanh", type=float, default=True, help="Trick 10: tanh activation function")
    parser.add_argument("--is_run_mode", type=bool, default=True, help="is run mode = Flase means that the mode is Training")
    parser.add_argument("--max_number_of_run", type=int, default=1000, help="max number of run (works in run mode only)")
    parser.add_argument("--state_dim", type=int, default=3, help="state dimention")
    parser.add_argument("--action_dim", type=int, default=2, help="action dimention")
    parser.add_argument("--max_episode_steps", type=int, default=28, help="maximum episode steps") #512
    parser.add_argument("--max_turning_angle", type=int, default=30, help="max turning angle (degree)")
    parser.add_argument("--min_leg_length", type=int, default=5, help="min leg length")
    parser.add_argument("--using_guidance", type=bool, default=False, help="using guidance?")
    parser.add_argument("--check_safty", type=bool, default=False, help="check safty?")
    parser.add_argument("--absolute_dir", type=str, default="/home/mohammad/catkin_ws/src/moving_agent/src/planning/hybrid_based/PPO_Continues", help="absolute direction")
    parser.add_argument("--run_classic_approach", type=bool, default=True, help="Do you want to run the classic approach?")

    args = parser.parse_args()

    base_PPO = Base_PPO()
    base_PPO.main(args, env_name="PathPlanning", number=1, seed=10)