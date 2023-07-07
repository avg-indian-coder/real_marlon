from sumolib import checkBinary
from grid_build_file import gen_rou_file
import subprocess
import configparser
import os
import traci
import time
import numpy as np





DEFAULT_PORT = 8000
VEH_LEN_M = 7.5 # effective vehicle length
PHASE_NUM = 5
STATE_NAMES = ['wave']   # include 'wait' and 'wave'
QUEUE_MAX = 10


class PhaseSet:
    def __init__(self, phases):
        self.num_phase = len(phases)
        self.num_lane = len(phases[0])
        self.phases = phases
        self._init_phase_set()

    @staticmethod
    def _get_phase_lanes(phase, signal='r'):
        phase_lanes = []
        for i, l in enumerate(phase):
            if l == signal:
                phase_lanes.append(i)
        return phase_lanes

    def _init_phase_set(self):
        self.red_lanes = []
        for phase in self.phases:
            self.red_lanes.append(self._get_phase_lanes(phase))


class PhaseMap:
    def __init__(self):
        self.phases = {}

    def get_phase(self, phase_id, action):
        # phase_type is either green or yellow

        # Gonna return a string, based on action index. ex : "GGgrrrGGgrrr"
        return self.phases[phase_id].phases[int(action)]

    def get_phase_num(self, phase_id):

        # Get number of phases for a given phase_id
        return self.phases[phase_id].num_phase

    def get_lane_num(self, phase_id):
        # the lane number is link number

        # Get number of lanes in the intersection
        return self.phases[phase_id].num_lane

    def get_red_lanes(self, phase_id, action):
        # the lane number is link number

        # Get red lanes for a given phase_id and action
        return self.phases[phase_id].red_lanes[int(action)]


class LargeGridPhase(PhaseMap):
    def __init__(self):
        phases = ['GGgrrrGGgrrr', 'rrrGrGrrrGrG', 'rrrGGrrrrGGr',
                  'rrrGGGrrrrrr', 'rrrrrrrrrGGG']
        self.phases = {PHASE_NUM: PhaseSet(phases)}




class Node:
    def __init__(self, name, neighbor=[], control=False):
        self.control = control # disabled
        self.ilds_in = [] # for state
        self.lanes_capacity = []
        self.fingerprint = [] # local policy
        self.name = name
        self.neighbor = neighbor
        self.num_state = 0 # wave and wait should have the same dim
        self.wave_state = [] # local state
        self.wait_state = [] # local state
        self.phase_id = -1
        self.n_a = 0
        self.prev_action = -1


class TrafficSimulator :

    def __init__(self, config, output_path, is_record, record_stats, port=0):
        self.obj = config.get('objective') # What to optimize
        self.name = config.get('scenario')
        self.agent = config.get('agent')
        self.output_path = output_path
        self.is_record = is_record
        self.cur_episode = 0
        self.seed = config.getint('seed')


        self.control_interval_sec = config.getint('control_interval_sec') # Time between Actions
        self.yellow_interval_sec = config.getint('yellow_interval_sec') # Yellow Time
        self.episode_length_sec = config.getint('episode_length_sec') # Episode Length
        self.data_path = config.get('data_path') # Where to store the xml files

        self.T = np.ceil(self.episode_length_sec / self.control_interval_sec) # Number of Actions
        self.port = DEFAULT_PORT + port # Port for SUMO
        self.sim_thread = port # Thread for SUMO
        
        self._init_map() # Implemented in super class
        self._init_sim(self.seed)   # Can generate output files here.
        self._init_nodes()

    def terminate(self):
        self.sim.close()

    def _init_map(self):
        # needs to be overwriteen
        self.neighbor_map = None
        self.phase_map = None
        self.state_names = None
        raise NotImplementedError()
    
    def _get_node_phase_id(self, node_name):
        # needs to be overwriteen
        raise NotImplementedError()
    
    def _reset_state(self):
        for node_name in self.node_names:
            node = self.nodes[node_name]
            # prev action for yellow phase before each switch
            node.prev_action = 0
    
    def _init_action_space(self):
        # for local and neighbor coop level
        self.n_agent = self.n_node
        # to simplify the sim, we assume all agents have the max action dim,
        # with tailing zeros during run time
        self.n_a_ls = []   # Its gonna be a bunch of 5's (25)
        for node_name in self.node_names:
            node = self.nodes[node_name]
            phase_id = self._get_node_phase_id(node_name)
            phase_num = self.phase_map.get_phase_num(phase_id)
            node.phase_id = phase_id
            node.n_a = phase_num
            self.n_a_ls.append(phase_num)

    def _init_state_space(self):
        self._reset_state()
        self.n_s_ls = []
        for node_name in self.node_names:
            node = self.nodes[node_name]
            node.num_state = len(node.ilds_in)
        for node_name in self.node_names:
            node = self.nodes[node_name]
            num_wave = node.num_state
            num_wait = 0 if 'wait' not in self.state_names else node.num_state
            if not self.agent.startswith('ma2c'):
                for nnode_name in node.neighbor:
                    num_wave += self.nodes[nnode_name].num_state
            self.n_s_ls.append(num_wait + num_wave)

    def _init_nodes(self):
        nodes = {}
        tl_nodes = self.sim.trafficlight.getIDList()      # node names and tl_nodes are the same
        for node_name in self.node_names:
            if node_name not in tl_nodes:
                exit(1)
            neighbor = self.neighbor_map[node_name]
            nodes[node_name] = Node(node_name,
                                    neighbor=neighbor,
                                    control=True)
            # controlled lanes: l:j,i_k
            lanes_in = self.sim.trafficlight.getControlledLanes(node_name)
            lanes_in = list(set(lanes_in))
            ilds_in = []
            lanes_cap = []
            for lane_name in lanes_in:
                ilds_in.append(lane_name)                
                cur_cap = self.sim.lane.getLength(lane_name)

                lanes_cap.append(cur_cap/float(VEH_LEN_M))

            nodes[node_name].ilds_in = ilds_in   
            nodes[node_name].lanes_capacity = lanes_cap

        
        self.nodes = nodes
        s = 'Env: init %d node information:\n' % len(self.node_names)
        for node_name in self.node_names:
            s += node_name + ':\n'
            node = self.nodes[node_name]
            s += '\tneigbor: %r\n' % node.neighbor
            s += '\tilds_in: %r\n' % node.ilds_in

        self._init_action_space()
        self._init_state_space()

    def update_fingerprint(self, policy):
        for node_name, pi in zip(self.node_names, policy):
            self.nodes[node_name].fingerprint = pi

    def _get_state(self):
        # hard code the state ordering as wave, wait, fp
        state = []
        # measure the most recent state
        self._measure_state_step()

        # get the appropriate state vectors
        for node_name in self.node_names:
            node = self.nodes[node_name]
            # wave is required in state
            if self.agent == 'greedy':
                state.append(node.wave_state)
            # else:
            #     cur_state = [node.wave_state]

            #     # include wave states of neighbors
            #     if self.agent.startswith('ia2c'):
            #         for nnode_name in node.neighbor:
            #             cur_state.append(self.nodes[nnode_name].wave_state)

            #     # include fingerprints of neighbors
            #     if self.agent == 'ia2c_fp':
            #         for nnode_name in node.neighbor:
            #             cur_state.append(self.nodes[nnode_name].fingerprint)

            #     # include wait state
            #     if 'wait' in self.state_names:
            #         cur_state.append(node.wait_state)
                # state.append(np.concatenate(cur_state))
        return state
    
    def _measure_state_step(self):
        for node_name in self.node_names:
            node = self.nodes[node_name]
            for state_name in self.state_names:
                if state_name == 'wave':
                    cur_state = []
                    for k, ild in enumerate(node.ilds_in):
                
                        cur_wave = self.sim.lanearea.getLastStepVehicleNumber(ild) # Gets number of vehicles in the lane 
                        cur_state.append(cur_wave)
                    cur_state = np.array(cur_state)
                # elif state_name == 'wait':
                #     cur_state = []
                #     for ild in node.ilds_in:
                #         max_pos = 0
                #         car_wait = 0
                       
                #         cur_cars = self.sim.lanearea.getLastStepVehicleIDs(ild) # Gets vehivle id for a lane
                #         for vid in cur_cars:
                #             car_pos = self.sim.vehicle.getLanePosition(vid) # Gets position of vehicle from bumper to front (not the end at the inter) of lane
                #             if car_pos > max_pos:
                #                 max_pos = car_pos
                #                 car_wait = self.sim.vehicle.getWaitingTime(vid) 
                #         cur_state.append(car_wait)
                #     cur_state = np.array(cur_state)
                
                # Get emergency class vehicles

                # for ild in node.ilds_in:
                #     cur_cars = self.sim.lanearea.getLastStepVehicleIDs(ild) 
                #     flag = 0
                #     for vid in cur_cars:
                #         e = self.sim.vehicle.getVehicleClass(vid) 
                #         if e == 'emergency':
                #             cur_state = np.append(cur_state, 1)
                #             flag = 1
                #             break
                    
                    # if flag == 0 :
                    #     cur_state = np.append(cur_state, 0)
                    


                # if self.record_stats:
                #     self.state_stat[state_name] += list(cur_state)
                # normalization
                norm_cur_state = cur_state

                if state_name == 'wave':
                    node.wave_state = norm_cur_state
                else:
                    node.wait_state = norm_cur_state


    def _init_sim_config(self):
        # needs to be overwriteen
        raise NotImplementedError()
    
    def reset(self, gui=True, test_ind=0):
        # have to terminate previous sim before calling reset
        self._reset_state()
        seed = test_ind
        self._init_sim(seed, gui=gui)
        self.cur_sec = 0
        self.cur_episode += 1
        # initialize fingerprint
        self.update_fingerprint(self._init_policy())
        # next environment random condition should be different
        self.seed += 1
        return self._get_state()

    
    def _init_policy(self):
        # Will return an array of 25 sub arrays containing 5 elements each, the elements are 0.2 
        return [np.ones(self.n_a_ls[i]) / self.n_a_ls[i] for i in range(self.n_agent)]


    def _init_sim(self, seed, gui=True):
        
        sumocfg_file = self._init_sim_config(seed)
        if gui:
            app = 'sumo-gui'
        else:
            app = 'sumo'
        command = [checkBinary(app), '-c', sumocfg_file]
        command += ['--seed', str(seed)]
        command += ['--remote-port', str(self.port)]
        command += ['--no-step-log', 'True']
        command += ['--time-to-teleport', '600'] # long teleport for safety
        command += ['--no-warnings', 'True']
        command += ['--duration-log.disable', 'True']
        # collect trip info if necessary
        if self.is_record:
            command += ['--tripinfo-output',
                        self.output_path + ('%s_%s_trip.xml' % (self.name, self.agent))]
        subprocess.Popen(command)
        # wait 1s to establish the traci server
        time.sleep(1)
        self.sim = traci.connect(port=self.port)

    def step(self, action):
        self._set_phase(action, 'yellow', self.yellow_interval_sec)
        self._simulate(self.yellow_interval_sec)
        rest_interval_sec = self.control_interval_sec - self.yellow_interval_sec
        self._set_phase(action, 'green', rest_interval_sec)
        self._simulate(rest_interval_sec)
        state = self._get_state()
        reward = self._measure_reward_step()
        done = False
        if self.cur_sec >= self.episode_length_sec:
            done = True
        global_reward = np.sum(reward)
        

        # use original rewards in test
        # if not self.train_mode:
        #     return state, reward, done, global_reward
        # if (self.agent == 'greedy') or (self.coop_gamma < 0):
        #     reward = global_reward
        return state, reward, done, global_reward

    def _measure_reward_step(self):
        rewards = []
        for node_name in self.node_names:
            queues = []
            waits = []
            for ild in self.nodes[node_name].ilds_in:
                if self.obj in ['queue', 'hybrid']:
                    if self.name == 'atsc_real_net':
                        cur_queue = self.sim.lane.getLastStepHaltingNumber(ild[0])
                        cur_queue = min(cur_queue, QUEUE_MAX)
                    else:
                        cur_queue = self.sim.lanearea.getLastStepHaltingNumber(ild)
                    queues.append(cur_queue)
                if self.obj in ['wait', 'hybrid']:
                    max_pos = 0
                    car_wait = 0
                    if self.name == 'atsc_real_net':
                        cur_cars = self.sim.lane.getLastStepVehicleIDs(ild[0])
                    else:
                        cur_cars = self.sim.lanearea.getLastStepVehicleIDs(ild)
                    for vid in cur_cars:
                        car_pos = self.sim.vehicle.getLanePosition(vid)
                        if car_pos > max_pos:
                            max_pos = car_pos
                            car_wait = self.sim.vehicle.getWaitingTime(vid)
                    waits.append(car_wait)
            queue = np.sum(np.array(queues)) if len(queues) else 0
            wait = np.sum(np.array(waits)) if len(waits) else 0
            if self.obj == 'queue':
                reward = - queue
            elif self.obj == 'wait':
                reward = - wait
            else:
                reward = - queue - self.coef_wait * wait
            rewards.append(reward)
        return np.array(rewards)
    
    def _get_node_phase(self, action, node_name, phase_type):
        node = self.nodes[node_name]
        cur_phase = self.phase_map.get_phase(node.phase_id, action)
        if phase_type == 'green':
            return cur_phase
        prev_action = node.prev_action
        node.prev_action = action
        if (prev_action < 0) or (action == prev_action):
            return cur_phase
        prev_phase = self.phase_map.get_phase(node.phase_id, prev_action)
        switch_reds = []
        switch_greens = []
        for i, (p0, p1) in enumerate(zip(prev_phase, cur_phase)):
            if (p0 in 'Gg') and (p1 == 'r'):
                switch_reds.append(i)
            elif (p0 in 'r') and (p1 in 'Gg'):
                switch_greens.append(i)
        if not len(switch_reds):
            return cur_phase
        yellow_phase = list(cur_phase)
        for i in switch_reds:
            yellow_phase[i] = 'y'
        for i in switch_greens:
            yellow_phase[i] = 'r'
        return ''.join(yellow_phase)
    
    def _set_phase(self, action, phase_type, phase_duration):
        for node_name, a in zip(self.node_names, list(action)):
            phase = self._get_node_phase(a, node_name, phase_type)
            self.sim.trafficlight.setRedYellowGreenState(node_name, phase)
            self.sim.trafficlight.setPhaseDuration(node_name, phase_duration)

    def _simulate(self, num_step):
        # reward = np.zeros(len(self.control_node_names))
        for _ in range(num_step):
            self.sim.simulationStep()
            self.cur_sec += 1
            






class GridEnvironment(TrafficSimulator) : 

    def __init__(self, config, port=0, output_path='', is_record=False, record_stat=False):
        self.peak_flow1 = config.getint('peak_flow1')
        self.peak_flow2 = config.getint('peak_flow2')
        self.init_density = config.getfloat('init_density')
        super().__init__(config, output_path, is_record, record_stat, port=port)

    def _init_sim_config(self, seed):
        return gen_rou_file(self.data_path,
                            self.peak_flow1,
                            self.peak_flow2,
                            self.init_density,
                            seed=seed,
                            thread=self.sim_thread)
    
    def _get_node_phase_id(self, node_name):
        return PHASE_NUM
    
    def _init_neighbor_map(self):
        neighbor_map = {}
        # corner nodes
        neighbor_map['nt1'] = ['nt6', 'nt2']
        neighbor_map['nt5'] = ['nt10', 'nt4']
        neighbor_map['nt21'] = ['nt22', 'nt16']
        neighbor_map['nt25'] = ['nt20', 'nt24']
        # edge nodes
        neighbor_map['nt2'] = ['nt7', 'nt3', 'nt1']
        neighbor_map['nt3'] = ['nt8', 'nt4', 'nt2']
        neighbor_map['nt4'] = ['nt9', 'nt5', 'nt3']
        neighbor_map['nt22'] = ['nt23', 'nt17', 'nt21']
        neighbor_map['nt23'] = ['nt24', 'nt18', 'nt22']
        neighbor_map['nt24'] = ['nt25', 'nt19', 'nt23']
        neighbor_map['nt10'] = ['nt15', 'nt5', 'nt9']
        neighbor_map['nt15'] = ['nt20', 'nt10', 'nt14']
        neighbor_map['nt20'] = ['nt25', 'nt15', 'nt19']
        neighbor_map['nt6'] = ['nt11', 'nt7', 'nt1']
        neighbor_map['nt11'] = ['nt16', 'nt12', 'nt6']
        neighbor_map['nt16'] = ['nt21', 'nt17', 'nt11']
        # internal nodes
        for i in [7, 8, 9, 12, 13, 14, 17, 18, 19]:
            n_node = 'nt' + str(i + 5)
            s_node = 'nt' + str(i - 5)
            w_node = 'nt' + str(i - 1)
            e_node = 'nt' + str(i + 1)
            cur_node = 'nt' + str(i)
            neighbor_map[cur_node] = [n_node, e_node, s_node, w_node]
        self.neighbor_map = neighbor_map
        self.neighbor_mask = np.zeros((self.n_node, self.n_node))
        for i in range(self.n_node):
            for nnode in neighbor_map['nt%d' % (i+1)]:
                ni = self.node_names.index(nnode)
                self.neighbor_mask[i, ni] = 1

    def _init_distance_map(self):
        block0 = np.array([[0,1,2,3,4],[1,0,1,2,3],[2,1,0,1,2],[3,2,1,0,1],[4,3,2,1,0]])
        block1 = block0 + 1
        block2 = block0 + 2
        block3 = block0 + 3
        block4 = block0 + 4
        row0 = np.hstack([block0, block1, block2, block3, block4])
        row1 = np.hstack([block1, block0, block1, block2, block3])
        row2 = np.hstack([block2, block1, block0, block1, block2])
        row3 = np.hstack([block3, block2, block1, block0, block1])
        row4 = np.hstack([block4, block3, block2, block1, block0])
        self.distance_mask = np.vstack([row0, row1, row2, row3, row4]) 

    def _init_map(self):
        self.node_names = ['nt%d' % i for i in range(1, 26)]
        self.n_node = 25
        self._init_neighbor_map()     # Sets up neighbourhood map and mask
        # for spatial discount
        self._init_distance_map()     # Sets up distance of each node to each other node
        self.max_distance = 8
        self.phase_map = LargeGridPhase()
        self.state_names = STATE_NAMES

class GridController:
    def __init__(self, node_names):
        self.name = 'greedy'
        self.node_names = node_names

    def forward(self, obs):
        actions = []
        for ob, node_name in zip(obs, self.node_names):
            actions.append(self.greedy(ob, node_name))
        return actions

    def greedy(self, ob, node_name):
        # hard code the mapping from state to number of cars

        print(node_name)

        flows = [ob[0] + ob[3], ob[2] + ob[5], ob[1] + ob[4],
                 ob[1] + ob[2], ob[4] + ob[5]]
        
        print(ob)
        print(flows)
        print(np.argmax(np.array(flows)))

        return np.argmax(np.array(flows))


config = configparser.ConfigParser()
config.read('./config/config_greedy.ini')
base_dir = './outputs/greedy/'
if not os.path.exists(base_dir):
    os.mkdir(base_dir)
env = GridEnvironment(config['ENV_CONFIG'], 2, base_dir, is_record=True, record_stat=True)
    
controller = GridController(env.node_names)
rewards = []

print(env.nodes["nt14"].ilds_in)

ob = env.reset(test_ind=2)
while True:
    print(env.cur_sec)
    next_ob, _, done, reward = env.step(controller.forward(ob))
    rewards.append(reward)
    if done:
        break
    ob = next_ob
    print("................")


env.terminate()