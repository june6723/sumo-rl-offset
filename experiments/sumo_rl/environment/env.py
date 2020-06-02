import os
import sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")
import traci
import sumolib
from gym import Env
import traci.constants as tc
from gym import spaces
from ray.rllib.env.multi_agent_env import MultiAgentEnv
import numpy as np
import pandas as pd

from .traffic_signal import TrafficSignal


class SumoEnvironment(MultiAgentEnv):
    """
    SUMO Environment for Traffic Signal Control

    :param net_file: (str) SUMO .net.xml file
    :param route_file: (str) SUMO .rou.xml file
    :param phases: (traci.trafficlight.Phase list) Traffic Signal phases definition
    :param out_csv_name: (str) name of the .csv output with simulation results. If None no output is generated
    :param use_gui: (bool) Wheter to run SUMO simulation with GUI visualisation
    :param num_seconds: (int) Number of simulated seconds on SUMO
    :param max_depart_delay: (int) Vehicles are discarded if they could not be inserted after max_depart_delay seconds
    :param time_to_load_vehicles: (int) Number of simulation seconds ran before learning begins
    :param delta_time: (int) Simulation seconds between actions
    :param min_green: (int) Minimum green time in a phase
    :param max_green: (int) Max green time in a phase
    :single_agent: (bool) If true, it behaves like a regular gym.Env. Else, it behaves like a MultiagentEnv (https://github.com/ray-project/ray/blob/master/python/ray/rllib/env/multi_agent_env.py)
    """
    
    def __init__(self, net_file, route_file, out_csv_path=None,out_csv_name=None, use_gui=False, num_seconds=20000, max_depart_delay=100000,
                 time_to_load_vehicles=0, single_agent=False):

        self._net = net_file
        self._route = route_file
        self.use_gui = use_gui
        if self.use_gui:
            self._sumo_binary = sumolib.checkBinary('sumo-gui')
        else:
            self._sumo_binary = sumolib.checkBinary('sumo')        
        traci.start([sumolib.checkBinary('sumo'), '-n', self._net])  # start only to retrieve information

        self.single_agent = single_agent
        self.ts_ids = ['gneJ00','gneJ9']
        self.ts_ids_all = ['gneJ26','gneJ00','gneJ9']
        self.traffic_signals = dict()
        self.sim_max_time = num_seconds
        self.time_to_load_vehicles = time_to_load_vehicles  # number of simulation seconds ran in reset() before learning starts
        self.max_depart_delay = max_depart_delay  # Max wait time to insert a vehicle

        # addition
        self.passed_num_of_veh_in_ts = dict()
        self.pass_veh_between_intersection_start = dict()
        self.pass_veh_between_intersection_end = []
        self.wait_veh = dict()
        self.last_step_waiting_time = 0

        self.remain_time = 0
        self.last_reward = 0
        self.last_queue_sum = 0
    

        input = [{'gneE1_0':'gneE11_0'}, {'-gneE9_0':'gneE3_0'}, {'gneE1_0':'gneE10_0'}, {'-gneE9_0':'gneE2_0'}] # manually
        self.in_lane = []
        self.out_lane = []
        for index in input : 
            for key, value in index.items() :
                if key not in self.in_lane :    
                    self.in_lane.append(key)
                if value not in self.out_lane :
                    self.out_lane.append(value)  

        self.reward_range = (-float('inf'), float('inf'))
        self.metadata = {}
        self.spec = ''

        self.run = 0
        self.metrics = []
        self.out_csv_name = out_csv_name
        self.out_csv_path = out_csv_path

        for ts in self.ts_ids_all:
            self.traffic_signals[ts] = TrafficSignal(self, ts)

        traci.close()

    # Reset before start iterations    
    def reset(self):
        if self.run != 0:
            self.save_csv(self.out_csv_path, self.out_csv_name, self.run)
            traci.close()
        self.run += 1
        self.metrics = []
        # origin
        sumo_cmd = [self._sumo_binary,
                     '-n', self._net,
                     '-r', self._route,
                     '--max-depart-delay', str(self.max_depart_delay), 
                     '--waiting-time-memory', '10000', 
                     '--random']
        
        if self.use_gui:
            sumo_cmd.append('--start')
        traci.start(sumo_cmd)
        
        # Load vehicles
        for i in range(self.time_to_load_vehicles):
            self._sumo_step()
        
        if self.single_agent:
            return self._compute_observations()[self.ts_ids[0]]
        else:
            return self._compute_observations()

    def _sumo_step(self):
        traci.simulationStep()
    
    def _get_queue_density(self) :
        # if self.sim_step % 5 == 0 :
        for ts in self.ts_ids_all :
            self.traffic_signals[ts].get_accumulated_queue_in_valid_lanes()
                # self.traffic_signals[ts].get_accumulated_density_in_lanes()

    def _compute_observations(self):
        """
        Return the current observation for each traffic signal
        """
        observations = {}
        state = []
        for ts in self.ts_ids:
            state.append(self.traffic_signals[ts].offset)
        observations['offset_agent'] = state
        return observations

    def step(self, action):
        # act
        adjust_cycle = {}
        index = 0
        for ts in self.ts_ids :
            adjust_cycle[ts] = self.traffic_signals[ts].change_offset(action['offset_agent'][index])
            index += 1
            
        if adjust_cycle['gneJ00'] > adjust_cycle['gneJ9'] :
            for _ in range(adjust_cycle['gneJ9']) :
                self._sumo_step()
            self.traffic_signals['gneJ9'].origin_cycle()
            for _ in range(adjust_cycle['gneJ00']-adjust_cycle['gneJ9']) :
                self._sumo_step()
            self.traffic_signals['gneJ00'].origin_cycle()

        elif adjust_cycle['gneJ00'] == adjust_cycle['gneJ9'] :
            for _ in range(adjust_cycle['gneJ9']) :
                self._sumo_step()
            self.traffic_signals['gneJ00'].origin_cycle()
            self.traffic_signals['gneJ9'].origin_cycle()
        else :
            for _ in range(adjust_cycle['gneJ00']) :
                self._sumo_step()
            self.traffic_signals['gneJ00'].origin_cycle()
            for _ in range(adjust_cycle['gneJ9'] - adjust_cycle['gneJ00']) :
                self._sumo_step()
            self.traffic_signals['gneJ9'].origin_cycle()

        # offs = [self.traffic_signals['gneJ00'].offset, self.traffic_signals['gneJ9'].offset]
        # print(offs)
        # print('gneJ26_phase:{0}    gneJ26_dur:{1}'.format(traci.trafficlight.getPhase('gneJ26'),traci.trafficlight.getPhaseDuration('gneJ26')-traci.trafficlight.getNextSwitch('gneJ26')+traci.simulation.getTime()))
        # print('gneJ00_phase:{0}    gneJ00_dur:{1}'.format(traci.trafficlight.getPhase('gneJ00'),traci.trafficlight.getPhaseDuration('gneJ00')-traci.trafficlight.getNextSwitch('gneJ00')+traci.simulation.getTime()))
        # print('gneJ9_phase:{0}    gneJ9_dur:{1}'.format(traci.trafficlight.getPhase('gneJ9'),traci.trafficlight.getPhaseDuration('gneJ9')-traci.trafficlight.getNextSwitch('gneJ9')+traci.simulation.getTime()))

        # simulation
        dur = [adjust_cycle['gneJ00'], adjust_cycle['gneJ9']]
        self.remain_time = 612 - max(dur)
        for i in range(self.remain_time) :
            self._sumo_step()
            if i > self.remain_time-306 and i<=self.remain_time :
                self._get_queue_density()

        # observe new state and reward
        observation = self._compute_observations() 
        reward = self._compute_rewards() 
        info = self._compute_step_info()
        done = {'__all__': self.sim_step > self.sim_max_time}
        self.metrics.append(info)
        
        for ts in self.ts_ids_all:
            for i in range(len(self.traffic_signals[ts].queue)):
                self.traffic_signals[ts].queue[i] = 0
                self.traffic_signals[ts].queue_sum_count[i] = 0

        if self.single_agent:
            return observation[self.ts_ids[0]], reward[self.ts_ids[0]], done['__all__'], {}
        else:
            return observation, reward, done, {}

    @property
    def sim_step(self):
        """
        Return current simulation second on SUMO
        """
        return traci.simulation.getTime()

    # 2
    def _compute_rewards(self):
        rewards = {}
        queue_sum = 0

        for ts in self.ts_ids_all:
            for i in range(len(self.traffic_signals[ts].queue)) :
                avg = self.traffic_signals[ts].queue[i] / self.traffic_signals[ts].queue_sum_count[i]
                queue_sum += avg
        
        rewards['offset_agent'] = -1 * queue_sum
        self.last_queue_sum = queue_sum
        return rewards

    # 3
    def _compute_step_info(self):
        info = {
            'step_time': int(self.sim_step),
            'gneJ00_offset': self.traffic_signals['gneJ00'].offset,
            'gneJ9_offset': self.traffic_signals['gneJ9'].offset,
            'queue_sum' : round(self.last_queue_sum,1)
        }
        return info

    def save_csv(self, out_csv_path, out_csv_name, run):
        if out_csv_name is not None:
            df = pd.DataFrame(self.metrics)
            df.to_csv(out_csv_path + out_csv_name + '_run{}'.format(run) + '.csv', index=False)