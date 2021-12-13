# CMPT 417 Fall 2021 by.Guiyi Su(Eric) ID:301300440
from os import path
import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import copy


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    for t in range(0,max(len(path1),len(path2))):
        if(get_location(path1,t) == get_location(path2,t)):
            return [get_location(path1,t)],t
        elif(t >= 0 and (get_location(path1,t-1) == get_location(path2,t) and get_location(path2,t-1) == get_location(path1,t))):
            return [get_location(path1,t-1),get_location(path1,t)],t  
    return None,None
    pass


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    result = []
    
    for i, agent1 in enumerate(paths):
        for j , agent2 in enumerate(paths[i+1:],start = i+1):
            loc,t = detect_collision(agent1,agent2)
            if(loc != None and t != None):
                result.append({'a1': i, 'a2': j, 'loc': loc, 'timestep': t})
    return result
    pass

def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    result = []
    result.append({'agent': collision['a1'], 'loc':  collision['loc'], 'timestep': collision['timestep']})
    result.append({'agent': collision['a2'], 'loc':  collision['loc'][::-1], 'timestep': collision['timestep']})
    return result
    pass


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    #[4.2]

    result = []
    rand = random.randint(0,1) # [4.2] pick agent randomly.
    if(rand == 1 and len(collision['loc']) > 1):
        result.append({'agent': collision['a2'], 'loc':  collision['loc'][::-1], 'timestep': collision['timestep']})
        result.append({'agent': collision['a2'], 'loc':  collision['loc'][::-1], 'timestep': collision['timestep'],'positive' : True})
    else:
        agentName = 'a' + str(rand + 1) # [4.2] Select agent names randomly using implicit conversions
        result.append({'agent': collision[agentName], 'loc':  collision['loc'], 'timestep': collision['timestep']})
        result.append({'agent': collision[agentName], 'loc':  collision['loc'], 'timestep': collision['timestep'],'positive' : True})        
    return result
    pass


def paths_violate_constraint(constraint, paths): # refer from file "paths_violate_constraint.py"
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def generateChild(self, root, constraint): #base on the constraint used to generate a child
        result = dict()
        result['paths'] = root['paths'].copy()
        result['constraints'] = root['constraints'].copy()
        if(constraint not in  result['constraints']):
            result['constraints'].append(copy.deepcopy(constraint))
        return result, constraint['agent']
  
    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,          
                'constraints': [],  
                'paths': [],        
                'collisions': []}   
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node() 
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint 
        #           Ensure to create a copy of any objects that your child nodes might inherit

        result = root
        #disjoint = False # In test [3.3], use stadndardCheck
        disjoint = True  # In test [4.3], use disjointCheck (which means not use standardCheck)
        standardCheck = not disjoint  
        disjointCheck = disjoint
        

        while len(self.open_list) > 0:
            curr = self.pop_node()
            curr['collisions'] =  detect_collisions(curr['paths']) # make sure there is no collsions in the result
            if(curr['collisions'] == []):
                result = curr
                break
            collision = curr['collisions'][0]

            constraintUsed = []
            if(standardCheck == True):
                constraintUsed = standard_splitting(collision)
            elif(disjointCheck == True):
                constraintUsed = disjoint_splitting(collision)
            # [3.3][4.3] use different constraint in standard splite or disjoint split.

            for each in constraintUsed:
                child,agent = self.generateChild(curr,copy.deepcopy(each)) # bulid a new child for branch
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, child['constraints'])
                if(path != None):
                    child['paths'][agent] = path
                    violateList = []
                    check = True # [3.3] When 'positive constraint' is not applied, the new path is imported directly into the heap.
                    if('positive' in each): # [4.3] If positive constraints are used,consider whether other agents conflict with the current agent
                        violateList = paths_violate_constraint(each, child['paths'])   
                    for vio in violateList: # [4.3] Single pathfinding is performed on all conflicting agents, and corresponding negative constraints are added
                        temp = {'agent' : vio, 'loc' : each['loc'][::-1], 'timestep' : each['timestep']}
                        if(temp not in child['constraints']):
                            child['constraints'].append(copy.deepcopy(temp))
                        path = a_star(self.my_map, self.starts[vio], self.goals[vio], self.heuristics[vio], vio, child['constraints'])
                        if(path == None): 
                            check = False
                            break
                        child['paths'][vio] = path
                    if(check == True): # [4.3] If any agent has no solution under this constraint, the constraint will have no solution
                        child['collisions'] = detect_collisions(child['paths'])
                        child['cost'] = get_sum_of_cost(child['paths'])
                        self.push_node(child)

        root = result
        self.print_results(root)
        #print(root['paths'])
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))


