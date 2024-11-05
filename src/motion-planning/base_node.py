#!/usr/bin/env python
from PPO.PPO_continuous_main import Base_PPO
from PPO_DISCRETE.PPO_discrete_main import Base_PPO_Discrete
from PPO_DISCRETE.PPO_discrete_real_main import Base_PPO_Discrete_Real
import argparse
import json
import os
from enum import Enum

def load_hyperparameters(json_filename, method_name):
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get the directory of the current script
    json_file = os.path.join(script_dir, method_name, json_filename)
    
    with open(json_file, 'r') as f:
        return json.load(f)

class Method(Enum):
    PPO_CONTINUOUS = 1
    PPO_DISCRETE = 2
    DDPG = 3
    PPO_DISCRETE_REAL = 4

if __name__ == '__main__':
    method = Method.PPO_DISCRETE
    if method == Method.PPO_CONTINUOUS:
        parser = argparse.ArgumentParser("Hyperparameters Setting for PPO-continuous")
        
        # Load hyperparameters from JSON file
        hyperparams = load_hyperparameters('hyperparameters.json', "PPO") 

        # Add arguments from the loaded hyperparameters
        for key, value in hyperparams.items():
            arg_type = type(value)
            parser.add_argument(f"--{key}", type=arg_type, default=value, help=f"{key.replace('_', ' ').capitalize()}")

        args = parser.parse_args()

        base_PPO = Base_PPO()
        base_PPO.main(args, env_name="WebotsCrazyfly3D", number=1, seed=10)
    elif method == Method.PPO_DISCRETE:
        parser = argparse.ArgumentParser("Hyperparameters Setting for PPO-discrete")
        
        # Load hyperparameters from JSON file
        hyperparams = load_hyperparameters('hyperparameters.json', "PPO_DISCRETE") 

        # Add arguments from the loaded hyperparameters
        for key, value in hyperparams.items():
            arg_type = type(value)
            parser.add_argument(f"--{key}", type=arg_type, default=value, help=f"{key.replace('_', ' ').capitalize()}")

        args = parser.parse_args()

        base_PPO_Discrete = Base_PPO_Discrete()
        base_PPO_Discrete.main(args, env_name="WebotsCrazyfly3DPPODiscrete", number=1, seed=10)
    elif method == Method.PPO_DISCRETE_REAL:
        parser = argparse.ArgumentParser("Hyperparameters Setting for PPO-discrete")
        
        # Load hyperparameters from JSON file
        hyperparams = load_hyperparameters('hyperparameters.json', "PPO_DISCRETE") 

        # Add arguments from the loaded hyperparameters
        for key, value in hyperparams.items():
            arg_type = type(value)
            parser.add_argument(f"--{key}", type=arg_type, default=value, help=f"{key.replace('_', ' ').capitalize()}")

        args = parser.parse_args()

        base_PPO_Discrete = Base_PPO_Discrete_Real()
        base_PPO_Discrete.main(args, env_name="WebotsCrazyfly3DPPODiscreteReal", seed=10)

