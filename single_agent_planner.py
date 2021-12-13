# CMPT 417 Fall 2021 by.Guiyi Su(Eric) ID:301300440
import heapq
from typing import Dict


def move(loc, dir):
    directions = [ (0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)] #[1.1] Add "wait" command for move step
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraintTable = {}
    posConstraintTable = {}
    for each in constraints:
        if(each['agent'] == agent):
            if('positive' in each): # [4.1] Positive constraints are being used
                if(each['timestep'] in posConstraintTable):
                    posConstraintTable[each['timestep']].append(each['loc'])
                else:                
                    posConstraintTable[each['timestep']] = [each['loc'].copy()]
            else: # [1.3] Negative constraints are being imported into the constraint table
                if(each['timestep'] in constraintTable):
                    constraintTable[each['timestep']].append(each['loc'])
                else:
                    constraintTable[each['timestep']] = [each['loc'].copy()]
    return constraintTable,posConstraintTable
    pass


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    if((-1) in constraint_table): # [2.3] Determine whether low-priority agents agents collide the high-priority agents when they arrive.
        for each in constraint_table[-1]:
            if(each[1] == next_loc and next_time > each[0]):
                return True

    # [1.2] Determine whether two agents collide in the middle of MOVEING.
    return (next_time in constraint_table) and ([next_loc] in constraint_table[next_time] or [curr_loc,next_loc] in constraint_table[next_time])
    
def is_pos_constrained(curr_loc, next_loc, next_time, pos_constraint_table):
    #[4.3] Within the specified time, must step into the specified position, otherwise cannot satisfy the positive constraint
    
    if(next_time not in pos_constraint_table): # [4.1] The current step has nothing to do with positive constraints
        return True

    elif next_time in pos_constraint_table:
        if([next_loc] in pos_constraint_table[next_time] or [curr_loc,next_loc] in pos_constraint_table[next_time]):
        # [4.1] The current step matches the positive constraint
            return True

    return False    

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val']  <  n2['g_val'] + n2['h_val'] 


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position 
        goal_loc    - goal position 
        h_values    - heuristics function value 
        agent       - the agent that is being re-planned 
        constraints - constraints defining where robot should or cannot go at each timestep 
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict() 
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 't_val': 0, 'parent': None} # [1.1] t_val: time step
    push_node(open_list, root)
    closed_list[(root['loc'],0)] = root # [1.1] use tuple instead dictionary: easy to doing time constraint.

    constraintTable, posConstraintTable = build_constraint_table(constraints, agent) # create the base constraint table ([4.1] include postive constraint table)
    timeLimit = -1
    for each in constraintTable:
        if(each > timeLimit):
            timeLimit = each  
    # [1.4] To make sure that the solution is compeletely follow the constraints, 
    # it only can retrun the solution when the arrival time ensures that none of the constraints are violated
    # in other words: more than the minimum timeLimit.

    while len(open_list) > 0:
        curr = pop_node(open_list)
        timeStep = curr['t_val']
        if(len(my_map) == 0 or timeStep > (len(my_map) * len(my_map[0])) ** 2): #[2.4] Unsolved graphs will be terminated
            break # if map is Invalid, or the The number of iterations has exceeded a large value, means it may be unsolvable.
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints 
        if (curr['loc'] == goal_loc and timeStep >= timeLimit): # [1.4] make sure it's already match the time limit: The finish decision
            return get_path(curr)

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if(child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0])):
                continue # [2.2] agent escapes the edge of map: This is illegal
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if(is_constrained(curr['loc'], child_loc, timeStep + 1, constraintTable) == True): 
                continue # [1.2/1.3] not allow generate the child because constraint   
            if(is_pos_constrained(curr['loc'], child_loc, timeStep + 1, posConstraintTable) == False):    
                continue # [4.1] not allow generate the child beacuse violate the positive constraint
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    't_val': curr['t_val'] + 1,
                    }          
            if (child['loc'],timeStep + 1) in closed_list:
                existing_node = closed_list[(child['loc'],timeStep + 1)]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'],timeStep + 1)] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'],timeStep + 1)] = child
                push_node(open_list, child)
        #print(open_list)
            
    return None  # Failed to find solutions