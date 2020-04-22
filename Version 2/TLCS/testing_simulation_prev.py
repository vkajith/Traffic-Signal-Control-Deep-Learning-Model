import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import traci
import numpy as np
import random
import timeit
import os

# phase codes based on environment.net.xml
PHASE_NS_GREEN = 0  # action 0 code 00
PHASE_NS_YELLOW = 1
PHASE_NSL_GREEN = 2  # action 1 code 01
PHASE_NSL_YELLOW = 3
PHASE_EW_GREEN = 4  # action 2 code 10
PHASE_EW_YELLOW = 5
PHASE_EWL_GREEN = 6  # action 3 code 11
PHASE_EWL_YELLOW = 7


class Simulation:
   
    def __init__(self, Model, TrafficGen, sumo_cmd, max_steps, green_duration, yellow_duration, num_states, num_actions):
        self._Model = Model
        self._TrafficGen = TrafficGen
        self._step = 0
        self._sumo_cmd = sumo_cmd
        self._max_steps = max_steps
        self._green_duration = green_duration
        self._yellow_duration = yellow_duration
        self._num_states = num_states
        self._num_actions = num_actions
        self._reward_episode = []
        self._queue_length_episode = []
        self._counter=np.zeros(num_actions)
        self._sum_waiting_times = []
        self._sum_waiting_times_c = []
        


    def run(self, episode):
        """
        Runs the testing simulation
        """
        start_time = timeit.default_timer()

        # first, generate the route file for this simulation and set up sumo
        self._TrafficGen.generate_routefile(seed=episode)
        traci.start(self._sumo_cmd)
        print("Simulating...")

        # inits
        self._step = 0
        self._waiting_times = {}
        old_total_wait = 0
        old_action = -1 # dummy init

        while self._step < self._max_steps:

            # get current state of the intersection
            current_state = self._get_state()

            # calculate reward of previous action: (change in cumulative waiting time between actions)
            # waiting time = seconds waited by a car since the spawn in the environment, cumulated for every car in incoming lanes
            current_total_wait = self._collect_waiting_times()
            reward = -self._get_queue_length()

            # choose the light phase to activate, based on the current state of the intersection
            action = self._choose_action(current_state)

            # if the chosen phase is different from the last phase, activate the yellow phase
            if self._step != 0 and old_action != action:
                self._set_yellow_phase(old_action)
                self._simulate(self._yellow_duration)

            # execute the phase selected before
            self._set_green_phase(action)
            self._simulate(self._green_duration)

            # saving variables for later & accumulate reward
            old_action = action
            old_total_wait = current_total_wait

            self._reward_episode.append(reward)

        #print("Total reward:", np.sum(self._reward_episode))
        traci.close()
        simulation_time = round(timeit.default_timer() - start_time, 1)

        return simulation_time
    
    def run_c(self, episode):
        
        self._TrafficGen.generate_routefile(seed=episode)
        traci.start(self._sumo_cmd)
        print("Simulating...")
        
        #inits
        self._step = 0
        self._waiting_times = {}
        action = 0
        while self._step < self._max_steps:
            
            self._set_green_phase(action)
            self._simulate_c(self._green_duration)
            self._set_green_phase(action)
            self._simulate_c(self._green_duration)
            self._set_green_phase(action)
            self._simulate_c(self._green_duration)
            
            self._set_yellow_phase(action)
            self._simulate_c(self._yellow_duration)
            
            if(action == self._num_actions - 1):
                action = 0
            else:
                action = action + 1
            
            
            


    def _simulate(self, steps_todo):
        """
        Proceed with the simulation in sumo
        """
        if (self._step + steps_todo) >= self._max_steps:  # do not do more steps than the maximum allowed number of steps
            steps_todo = self._max_steps - self._step

        while steps_todo > 0:
            traci.simulationStep()  # simulate 1 step in sumo
            self._step += 1 # update the step counter
            steps_todo -= 1
            wait_time = self._collect_waiting_times()
            self._sum_waiting_times.append(wait_time)
            
    def _simulate_c(self, steps_todo):
        """
        Proceed with the simulation in sumo
        """
        if (self._step + steps_todo) >= self._max_steps:  # do not do more steps than the maximum allowed number of steps
            steps_todo = self._max_steps - self._step

        while steps_todo > 0:
            traci.simulationStep()  # simulate 1 step in sumo
            self._step += 1 # update the step counter
            steps_todo -= 1
            wait_time = self._collect_waiting_times()
            self._sum_waiting_times_c.append(wait_time)


    def _collect_waiting_times(self):
        """
        Retrieve the waiting time of every car in the incoming roads
        """
        car_list = traci.vehicle.getIDList()
        for car_id in car_list:
            wait_time = traci.vehicle.getAccumulatedWaitingTime(car_id)
            self._waiting_times[car_id] = wait_time
        total_waiting_time = sum(self._waiting_times.values())
        return total_waiting_time


    def _choose_action(self, state):
        """
        Pick the best action known based on the current state of the env. Added Counter Variable.
        """
       
        actions = np.argsort(self._Model.predict_one(state))
        actions_flatten = [j for sub in actions for j in sub]
        actions_flatten = np.flip(actions_flatten)
        for i in range(0,self._num_actions):
            if(self._counter[actions_flatten[i]] > 10):
                self._counter[actions_flatten[i]] = 0
                for j in range(num_actions) and j!=i:
                    self._counter[actions_flatten[j]]+=1
                return actions_flatten[i]
        self._counter[actions_flatten[0]] = 0    
        return actions_flatten[0]


    def _set_yellow_phase(self, old_action):
        """
        Activate the correct yellow light combination in sumo
        """
        yellow_phase_code = old_action * 2 + 1 # obtain the yellow phase code, based on the old action (ref on environment.net.xml)
        traci.trafficlight.setPhase("TL", yellow_phase_code)


    def _set_green_phase(self, action_number):
        """
        Activate the correct green light combination in sumo
        """
       
        if action_number == 0:
            traci.trafficlight.setPhase("TL", PHASE_NS_GREEN)
        elif action_number == 1:
            traci.trafficlight.setPhase("TL", PHASE_NSL_GREEN)
        elif action_number == 2:
            traci.trafficlight.setPhase("TL", PHASE_EW_GREEN)
        elif action_number == 3:
            traci.trafficlight.setPhase("TL", PHASE_EWL_GREEN)


    def _get_queue_length(self):
        """
        Retrieve the number of cars with speed = 0 in every incoming lane
        """
        halt_N = traci.edge.getLastStepHaltingNumber("N2TL")
        halt_S = traci.edge.getLastStepHaltingNumber("S2TL")
        halt_E = traci.edge.getLastStepHaltingNumber("E2TL")
        halt_W = traci.edge.getLastStepHaltingNumber("W2TL")
        queue_length = halt_N + halt_S + halt_E + halt_W
        return queue_length


    def _get_state(self):
        """
        Retrieve the state of the intersection from sumo, in the form of cell occupancy
        """
        state = np.zeros(self._num_states)
        state[0] = traci.lane.getLastStepVehicleNumber('W2TL_0') + \
                        traci.lane.getLastStepVehicleNumber('W2TL_1') + \
                        traci.lane.getLastStepVehicleNumber('W2TL_2')
        state[1] = traci.lane.getLastStepVehicleNumber('W2TL_3')
        state[2] = traci.lane.getLastStepVehicleNumber('N2TL_0') + \
                        traci.lane.getLastStepVehicleNumber('N2TL_1') + \
                        traci.lane.getLastStepVehicleNumber('N2TL_2')
        state[3] = traci.lane.getLastStepVehicleNumber('N2TL_3')
        state[4] = traci.lane.getLastStepVehicleNumber('E2TL_0') + \
                        traci.lane.getLastStepVehicleNumber('E2TL_1') + \
                        traci.lane.getLastStepVehicleNumber('E2TL_2')
        state[5] = traci.lane.getLastStepVehicleNumber('E2TL_3')
        state[6] = traci.lane.getLastStepVehicleNumber('S2TL_0') + \
                        traci.lane.getLastStepVehicleNumber('S2TL_1') + \
                        traci.lane.getLastStepVehicleNumber('S2TL_2') 
        state[7] = traci.lane.getLastStepVehicleNumber('S2TL_3') 

        return state


    @property
    def queue_length_episode(self):
        return self._queue_length_episode


    @property
    def reward_episode(self):
        return self._reward_episode
    
    @property
    def sum_waiting_times(self):
        return self._sum_waiting_times

    @property
    def sum_waiting_times_c(self):
        return self._sum_waiting_times_c


