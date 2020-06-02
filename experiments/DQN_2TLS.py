import argparse
import os
import sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")
import pandas as pd
import ray
from ray.rllib.agents.dqn.dqn import DQNTrainer
from ray.rllib.agents.dqn.dqn import DQNTFPolicy
from ray.tune.registry import register_env
from ray.tune.logger import pretty_print
from gym import spaces
import numpy as np
from sumo_rl.environment.env import SumoEnvironment
import traci

def policy_mapping(id):
    # if id == 'gneJ00':
    #     return 'gneJ00'
    # else:
    return 'offset_agent'

if __name__ == '__main__':
    ray.init()

    register_env("2TLS", lambda _: SumoEnvironment(net_file='/home/sonic/Desktop/sumo-rl-research-offset/sumo-rl-research/experiments/nets/Research/case04/intersection.net.xml',
                                                    route_file='/home/sonic/Desktop/sumo-rl-research-offset/sumo-rl-research/experiments/nets/Research/case04/intersection.rou.xml',
                                                    out_csv_path='outputs/case04/',
                                                    out_csv_name='DQN_3',
                                                    use_gui=True,
                                                    num_seconds=15300510,
                                                    time_to_load_vehicles=510,
                                                    max_depart_delay=0)
                    )

    trainer = DQNTrainer(env="2TLS", config={
        "multiagent": {
            "policy_graphs": {
                'offset_agent': (DQNTFPolicy, spaces.Box(low=np.zeros(15), high=np.array(['inf']*15)), spaces.MultiDiscrete([102,102]), {})
            },
            "policy_mapping_fn": policy_mapping  # Traffic lights are always controlled by this policy
        },
        "lr": 0.0001,
    })
    
    while True:
        result = trainer.train()
# /home/sonic/Desktop/sumo-rl-research-offset/sumo-rl-research/experiments/