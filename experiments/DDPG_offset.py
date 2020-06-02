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
from ray.rllib.agents.ddpg.ddpg import DDPGTrainer
from ray.rllib.agents.ddpg.ddpg import DDPGTFPolicy
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
                                                    out_csv_name='DDPG',
                                                    use_gui=False,
                                                    num_seconds=12240612,
                                                    time_to_load_vehicles=612,
                                                    max_depart_delay=0)
                    )

    trainer = DDPGTrainer(env="2TLS", config={
        "multiagent": {
            "policy_graphs": {
                'offset_agent': (DDPGTFPolicy, spaces.Box(low=np.zeros(2), high=np.array(['inf']*2)), spaces.Box(low=np.array([0,0]), high=np.array([+1,+1])), {})
            },
            "policy_mapping_fn": policy_mapping  # Traffic lights are always controlled by this policy
        },
        "lr": 0.0001,
    })
    
    while True:
        result = trainer.train()
# /home/sonic/Desktop/sumo-rl-research-offset/sumo-rl-research/experiments/