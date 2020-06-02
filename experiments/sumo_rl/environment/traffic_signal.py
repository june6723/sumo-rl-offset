import os
import sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")
import traci
import numpy as np

class TrafficSignal:
    """
    This class represents a Traffic Signal of an intersection
    It is responsible for retrieving information and changing the traffic phase using Traci API
    """
    def _get_phase_info(self) :
        phases = []
        Logic = traci.trafficlight.getAllProgramLogics(self.id) # returns logics in tuple
        for phase in Logic[0].getPhases() :  # so first element of tuple
            phases.append(phase)
            if 'G' in phase.state or 'g' in phase.state :
                self.num_green_phases += 1
                self.cycle += phase.duration
            else :
                self.cycle += phase.duration
        return phases

    def _get_lane_info(self) :
        full_lane = []
        full_lane += self.local_lane

        add_lane=self._add_lane_info()
        full_lane += add_lane
        while(add_lane) :
            add_lane=self._add_lane_info()
            full_lane += add_lane
        return full_lane

    def _add_lane_info(self) :
        add_lane = []
        all_lanes_in_net = traci.lane.getIDList()
        for lane in all_lanes_in_net :
            if not ':' in lane and lane not in self.local_lane :
                link = traci.lane.getLinks(lane)
                for index in link :
                    if index[0] in self.local_lane and self._judge_whether_internal_lane_is_in_ts(index[4]) and lane not in add_lane :
                        add_lane.append(lane)
        self.local_lane+=add_lane                
        return add_lane    
    
    def _judge_whether_internal_lane_is_in_ts(self, internal_lane) :
        judge = True
        # for ts_id in self.env.ts_ids :
        for ts_id in traci.trafficlight.getIDList() :
            if ts_id in internal_lane :
                judge = False
        return judge


    def __init__(self, env, ts_id):
        self.id = ts_id
        self.env = env
        self.num_green_phases = 0
        self.cycle = 0
        self.full_cycle = self.cycle + self.num_green_phases * 3
        # Bring phase information out of net file  
        self.phases = self._get_phase_info()
        self.origin_phases = []
        for p in self.phases:
            self.origin_phases.append(traci.trafficlight.Phase(p.duration, p.state,p.duration,p.duration))
        self.local_lane = list(dict.fromkeys(traci.trafficlight.getControlledLanes(self.id)))
        self.lanes = self._get_lane_info() # 3 <------------------------------------------

        if self.id == 'gneJ26' :
            self.valid_lanes = ['gneE6_0','gneE24_0','gneE24_1'] #'-gneE29_0','-gneE26_0','-gneE26_1',
        elif self.id == 'gneJ00' :
            self.valid_lanes = ['-gneE6_0','-gneE3_0','-gneE3_1','-gneE1_1','-gneE1_0','-gneE4_0']
        else :
            self.valid_lanes = ['gneE4_0','gneE9_0','gneE9_1'] # ,'-gneE11_0','-gneE11_1','-gneE14_0'

        self.lane_by_phase = dict()
        if self.id == 'gneJ26' :
            self.lane_by_phase[0] = ['gneE6_0','gneE24_0']
            self.lane_by_phase[2] = ['gneE24_1'] 
        elif self.id == 'gneJ00' :
            self.lane_by_phase[0] = ['-gneE6_0','-gneE3_0','-gneE1_0','-gneE4_0']
            self.lane_by_phase[2] = ['-gneE3_1','-gneE1_1'] 
        else :
            self.lane_by_phase[0] = ['gneE4_0','gneE9_0']
            self.lane_by_phase[2] = ['gneE9_1'] 

        self.offset = 0
        self.queue = [0]*len(self.valid_lanes)
        self.queue_sum_count = [0]*len(self.valid_lanes)

        self.density = [0]*len(self.lanes)
        self.last_phase = 0

        
        
        logic = traci.trafficlight.Logic("new-program", 0, 0, phases=self.phases)
        traci.trafficlight.setCompleteRedYellowGreenDefinition(self.id, logic)

        self.cycle = int(self.cycle)
        

    @property
    def phase(self):
        return traci.trafficlight.getPhase(self.id)

    # 1
    def get_accumulated_queue_in_lanes(self):
        index = 0
        for lane in self.lanes :
            if traci.lane.getLastStepHaltingNumber(lane) > self.queue[index] :
                self.queue[index] += traci.lane.getLastStepHaltingNumber(lane)
            index += 1

    def get_accumulated_queue_in_valid_lanes(self):
        # index = 0
        # for lane in self.valid_lanes :
        #     self.queue[index] += traci.lane.getLastStepHaltingNumber(lane)
        #     index += 1
        if self.phase in [7,1] and traci.trafficlight.getNextSwitch(self.id) == traci.simulation.getTime() :
            if self.phase==7 :
                for lane in self.lane_by_phase[0] :
                    self.queue[self.valid_lanes.index(lane)] += traci.lane.getLastStepHaltingNumber(lane)
                    self.queue_sum_count[self.valid_lanes.index(lane)] += 1
            else : 
                for lane in self.lane_by_phase[2] :
                    self.queue[self.valid_lanes.index(lane)] += traci.lane.getLastStepHaltingNumber(lane)
                    self.queue_sum_count[self.valid_lanes.index(lane)] += 1
    def get_accumulated_density_in_lanes(self):
        index = 0
        for lane in self.lanes :
            self.density[index] += traci.lane.getLastStepVehicleNumber(lane)
            index += 1
            
    def change_offset(self, action) :
        adjust = int(self.cycle*action)
        self.offset = self.offset + adjust if (self.offset+adjust)<self.cycle else (self.offset+adjust-self.cycle)
        adjust_cycle = self.cycle + adjust

        self.last_phase = self.phase
        remain_time_of_current_phase = traci.trafficlight.getNextSwitch(self.id) - traci.simulation.getTime()
        elapsed_time_of_current_phase = self.origin_phases[self.last_phase].duration - remain_time_of_current_phase

        remain_time_in_adj_cycle = adjust_cycle 
        adjust_phases = []
        index = 0
        for p in self.origin_phases : 
            dur = int(adjust_cycle * (p.duration/self.cycle))
            if index != 7 :
                adjust_phases.append(traci.trafficlight.Phase(dur, p.state,dur,dur))
                remain_time_in_adj_cycle -= adjust_phases[index].duration
            else :
                adjust_phases.append(traci.trafficlight.Phase(remain_time_in_adj_cycle, p.state,remain_time_in_adj_cycle,remain_time_in_adj_cycle))
            index += 1

        logic = traci.trafficlight.Logic("adjust", 0, 0, phases=adjust_phases)
        traci.trafficlight.setCompleteRedYellowGreenDefinition(self.id, logic)
        traci.trafficlight.setPhase(self.id, self.last_phase)
        # if remain_time_of_current_phase != 0:
        traci.trafficlight.setPhaseDuration(self.id, adjust_phases[self.last_phase].duration - elapsed_time_of_current_phase)
        adjust_cycle -= int(elapsed_time_of_current_phase)
        
        return adjust_cycle

    def origin_cycle(self) :
        logic = traci.trafficlight.Logic("new-program", 0, 0, phases=self.origin_phases)
        traci.trafficlight.setCompleteRedYellowGreenDefinition(self.id, logic)
        traci.trafficlight.setPhase(self.id, self.last_phase)