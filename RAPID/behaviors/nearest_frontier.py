from ..utils import *

import numpy as np

def nearest_frontier(selfrobot):
        """
        compute a greedy nearest frontier algorithm with an A* path search to the nearest frontier for each agent.
        """

        selfrobot.sense()#first of all sense the env.
        selfrobot.belief_transfer()

        if np.any(selfrobot.target):#si on a une target
            if selfrobot.path_to_target: #If we have a path to our target, we continue this path.
                selfrobot.navigate_through_target_path()
                pass
            else: #if we don't have any path, then compute it with A* for our target
                selfrobot.path_to_target = a_star_search(selfrobot.belief_space["occupancy_grid"], (int(selfrobot.transform.x),int(selfrobot.transform.y)), (selfrobot.target[0], selfrobot.target[1]), traversable_types=selfrobot.traversable_types) #from utils : A* Path calculation

        else: #sinon on va chercher les frontières.
            #frontier detection from belief space

            frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types) #from utils

            if list(frontiers) == None or len(list(frontiers))==0: #si on a pas de frontieres explo finie?
                if euclidian_distance((int(selfrobot.transform.x),int(selfrobot.transform.y)), (int(selfrobot.init_transform.x),int(selfrobot.init_transform.y))) > selfrobot.treshold_for_target:
                    selfrobot.target = (int(selfrobot.init_transform.x),int(selfrobot.init_transform.y))
                    selfrobot.last_plan_time = selfrobot.env.step
                else:
                    selfrobot.finish()
            else:
                #then we take the closest one.
                distance = np.inf
                for f in frontiers:
                    hdist = heuristic_frontier_distance((selfrobot.transform.x, selfrobot.transform.y), (f[0], f[1]), selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types)
                    if hdist < distance :
                        distance = hdist
                        selfrobot.target = tuple(f.tolist()) #set the frontier as new target
                        selfrobot.last_plan_time = selfrobot.env.step