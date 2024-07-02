#!/usr/bin/env python
from PPO.PPO_continuous_main import Base_PPO
import argparse
import json
import os

def load_hyperparameters(json_filename, method_name):
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get the directory of the current script
    json_file = os.path.join(script_dir, method_name, json_filename)
    
    with open(json_file, 'r') as f:
        return json.load(f)

if __name__ == '__main__':
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
