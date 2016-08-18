import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator
import itertools


class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # TODO: Initialize any additional variables here'
        self.actions = ['forward', 'left', 'right', None] # allowed actions
        self.states = ['permit', 'deny']
        self.colors = ['red', 'green'] # colors for states
        self.directions = ['forward', 'left', 'right']
        self.prev_state = None


        self.Q_matrix = {}  
        self.init_Q()
        self.trial_penalties = 0.0 # counter of penalties during trial
        self.trial_num = 0 # trials counter
        self.preformance_report = [] # list of tuples for report after run all trials
        self.moves = 0 # agent's moves during trial
        self.trial_penalties_details = [] # list of sensors and agent penalties during trial
        
    def simplier_state(self, color, next_waypoint, other_agent_waypoints):
        # simlifier world state and get GoStatus
        state = 'stop'
        if color == 'red':
            if next_waypoint == 'right' and other_agent_waypoints[1] != 'forward':
                state = 'go'
            else:
                state = 'stop'
        else:
            state = 'go'
            if next_waypoint == 'left' and other_agent_waypoints[0] == 'forward':
                state == 'stop'
            if next_waypoint == 'left' and other_agent_waypoints[0] == 'right':
                state == 'stop'
        
        return state
        
    
    def init_Q(self):
        # initialize Q_matrix
    
        other_next_waypoints = list(itertools.product(['forward', 'left', 'right', None], repeat=3)) # get all combinations of other agent states
        
        print "Combinations count", len(other_next_waypoints)
        
        self.Q_matrix = {} # reset matrix
        for color in self.colors:
            for next_waypoint in self.directions:
                for go_state in ['go', 'stop']: # other agent next waypoint
                    for action in self.actions:
                        key = "{}_{}_{}_{}".format(color, next_waypoint, go_state, action)
                        self.Q_matrix[key] = (0.0, color, next_waypoint, go_state, action)
        print "Total states in Q matrix", len(self.Q_matrix.items())
    
    def save_trial_performance(self):
        # save particular trial performance results
        self.preformance_report.append([self.trial_num, self.trial_penalties, self.deadline, self.moves, self.trial_penalties_details])

    def reset(self, destination=None):
        self.planner.route_to(destination)
        
        # save prev trial
        if self.prev_state != None:
            self.save_trial_performance()
        
        self.trial_num += 1
        self.trial_penalties = 0.0
        self.moves = 0
        self.epsilon = 0.1 # for e-greedy strategy
        self.trial_penalties_details = []
        self.deadline = 0
        
        # TODO: Prepare for a new trip; reset any variables here, if required

          
    def learn_policy(self, curr_action, reward):
        # learn policy using Q-Learning algorithm
                   
        inputs = self.env.sense(self)
        other_agentwaypoint = "{}:{}:{}".format(inputs['oncoming'], inputs['left'], inputs['right'])
        go_status = self.simplier_state(inputs['light'], self.next_waypoint, other_agentwaypoint)
        self.state = "{}_{}_{}_{}".format(inputs['light'], self.next_waypoint, go_status, curr_action)
                   
        alpha = 1.0
        gamma = 0.15
        
        # get old q value
        q_old =self.Q_matrix[self.prev_state][0]
        #print q_old

        act, maxQ = self.get_best_action(self.state)
        #print "LP: Best action: {}, {:.3f}".format(act, maxQ)
        
        q = q_old + alpha * (reward + gamma * maxQ - q_old)
               
        # update value of Q(s,a)
        features = list(self.Q_matrix[self.prev_state])
        features[0] = q
        self.Q_matrix["{}_{}_{}_{}".format(*(features[1:-1] + [curr_action]))] = tuple(features)
        
                
    def get_best_action(self, state):
        # get action with highest Q value for current state
        # uses epsilon greedy strategy
        
        features = list(self.Q_matrix[state][1:-1]) # ignore firs and last element in the list due it is Q value and state
        best_value = None
        
        epsilon = 1.0/self.trial_num
        #epsilon = 1.0/self.moves
        
        act_values = [] # temporary list for saving actions and q_values
        
        if random.uniform(0, 1) < epsilon:
            # select random action
            action = self.actions[random.randint(0, len(self.actions) - 1)]
            state_query = "{}_{}_{}_{}".format(*(features + [action]))
            best_value = self.Q_matrix[state_query][0]
            best_action = action
        else:        
            # get max Q value
            for action in self.actions:
                state_query = "{}_{}_{}_{}".format(*(features + [action]))
                #print "{}, {:.3f}, {}".format(action, self.Q_matrix[state_query][0], state_query)
                q_value = self.Q_matrix[state_query][0]
                act_values.append((q_value, action))
                if best_value is None:
                    best_action = action
                    best_value = q_value
                    
                    continue
                
                if self.Q_matrix[state_query][0] > best_value:
                    best_action = action
                    best_value = q_value
        
            # return best action and their Q value or select random action if there were more than one action with max Q value
            to_choose = []
            for q_value, action in act_values:
                if q_value == best_value:
                    to_choose.append((q_value, action))
            _, action = to_choose[random.randint(0, len(to_choose) - 1)]
        
        
         
        return best_action, best_value
        

    def update(self, t):
        
        # Gather inputs
        inputs = self.env.sense(self)
        self.deadline = self.env.get_deadline(self)
        
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        
        # TODO: Update state
        other_agent_waypoint = [inputs['oncoming'], inputs['right'], inputs['left']]
        go_status = self.simplier_state(inputs['light'], self.next_waypoint, other_agent_waypoint)
        self.prev_state = "{}_{}_{}_{}".format(inputs['light'], self.next_waypoint, go_status, 'forward')
     
        
        # TODO: Select action according to your policy
        #[None, 'forward', 'left', 'right']
        action, _ = self.get_best_action(self.prev_state)
        
        ''' random action implementation
        
        print "Next waypoint", self.next_waypoint
        action = self.actions[random.randint(0, len(self.actions) - 1)]
        '''
     
        '''
        # implementation of traffic rules depended on agent inputs
        # red signal check
        if inputs['light'] == 'red':
            if self.next_waypoint == 'right' and inputs['left'] != 'forward':
                action = 'right'
                self.state = self.states["RA"]
            else:
                action = None
                self.state = self.states["WA"]
        # green signal check
        elif inputs['light'] == 'green':
            if self.next_waypoint == 'left' and inputs['oncoming'] != 'forward' and \
                inputs['oncoming'] != 'right':
                    action = 'left'
                    self.state = self.states["LA"]
            elif self.next_waypoint == 'right':
                action = 'right'
                self.state = self.states["RA"]
            elif self.next_waypoint == 'forward':
                action = 'forward'
                self.state = self.states["FA"]
            else:
                action = None
                self.state = self.states["WA"]
        '''  
        
        
        # Execute action and get reward
        reward = self.env.act(self, action)
        out_message = "deadline={}, sense={}, act={}, next_wp={}, reward={}".format(self.deadline, inputs, action, self.next_waypoint, reward)
        if reward < 0.0:
            self.trial_penalties += reward
            self.trial_penalties_details.append(out_message)
        
        self.moves += 1 # increase moves of agent
        # TODO: Learn policy based on state, action, reward
        self.learn_policy(action, reward)        
        
        print "LearningAgent.update():"+ out_message # [debug]

    def show_performance_results(self, last_trials=10):
        # print N last trials' performance data
        result_list = self.preformance_report[-last_trials:]
        for line in result_list:
            print "{} trial: penalties {:.3f}, deadline {}, moves: {}".format(*line[:-1])
            for line in line[-1]:
                print "    {}".format(line)

def run():
    """Run the agent for a finite number of trials."""

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=True)  # specify agent to track
    # NOTE: You can set enforce_deadline=False while debugging to allow longer trials

    # Now simulate it
    sim = Simulator(e, update_delay=0.05, display=False)  # create simulator (uses pygame when display=True, if available)
    # NOTE: To speed up simulation, reduce update_delay and/or set display=False
    #e.act(a,'left')
    sim.run(n_trials=100)  # run for a specified number of trials
    a.save_trial_performance() # save performance data for the last trial
    a.show_performance_results() # print performance results
    # NOTE: To quit midway, press Esc or close pygame window, or hit Ctrl+C on the command-line


if __name__ == '__main__':
    run()
