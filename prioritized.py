# CMPT 417 Fall 2021 by.Guiyi Su(Eric) ID:301300440
import time as timer

from numpy.lib.twodim_base import _trilu_indices_form_dispatcher
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        #constraints.append({'agent': 0, 'loc': [(1,5)], 'timestep': 0}) 
        #[1.4] add a constraint to test the algorithm (Uncomment when test 1.4)
        temp = [{'agent': 1, 'loc': [(1,2)], 'timestep': 1},
                {'agent': 1, 'loc': [(1,3)], 'timestep': 2},
                {'agent': 1, 'loc': [(1,3),(1,2)], 'timestep': 2},
                {'agent': 1, 'loc': [(1,4)], 'timestep': 2},
                ]
        #constraints += temp 
        #[1.5] Set the path of agent 0 as the highest priority, and order agent 1 to avoid agant 0.(Uncomment when test 1.5)

        #constraints.append({'agent': 0, 'loc': [(3,2)], 'timestep': 1}) 
        #[2.5.3] Forbid agent 0 to move to the left in the first step.
        #[2.5.3] This constraint will not make agent 0 spend more time, but it can be avoided agent 0 blocking the exit of agent 2.
        #[2.5.3] If you add this constraint artificially, prioritized planning can find a solution; 
        #[2.5.3] otherwise, it cannot find a solution even it real exist.

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            for each in range(i + 1 ,self.num_of_agents):
            #[2.1] By default, agents with smaller numbers have higher priority
            #[2.1] So only agents with larger numbers need to be restricted.
                for n in range(0,len(path)):
                    constraints.append({'agent': each, 'loc': [path[n]], 'timestep': n}) 
                    #[2.1] Prohibit low-priority agents from preempting the position of high-priority agents at the same time
                    if(n != 0):
                        constraints.append({'agent': each, 'loc': [path[n],path[n-1]], 'timestep': n}) 
                        #[2.2] Prohibit low-priority agents and high-priority agents from swapping positions at the same time
                constraints.append({'agent': each, 'loc': [len(path)-1,path[-1]], 'timestep': -1}) 
                # [2.3] Prohibit low-priority agents from preempting the goal position of high-priority agents after they arrive.
                # [2.3] "-1" as a special command symbol changes the nature of'loc'.
                # [2.3] 'loc' will be used as a parameter to record the arrival time and target location of high-priority agents.
                    



            ##############################
        #print(constraints)
        self.CPU_time = timer.time() - start_time
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
