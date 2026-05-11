from ..utils import *



def minpos(selfrobot): #from Bautin, 2012
    """
    Adaptation from MinPos algorithm (Bautin, Simonin, Charpillet : 2012)
    Frontier based behavior where:
    - The frontiers are grouped into clusters
    - each cluster is given a cost depending on the distance and on robots that are closer to this frontier using the wavefront propagation algorithm (WPA)
    - the robot chose the frontier with the lowest cost
    """

    #first of all, sense the environment
    
    #frontier detection
    frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types) #from utils

    if list(frontiers) == None or len(list(frontiers))==0: #si on a pas de frontieres explo finie?
        if (int(selfrobot.transform.x),int(selfrobot.transform.y)) != (int(selfrobot.init_transform.x),int(selfrobot.init_transform.y)):
            selfrobot.target = (int(selfrobot.init_transform.x),int(selfrobot.init_transform.y))
            selfrobot.last_plan_time = selfrobot.env.step
        else:
            selfrobot.finish()
    else:
        cluster_centers = cluster_frontier_cells(selfrobot.belief_space["occupancy_grid"], frontiers, int(selfrobot.vision_range/2), traversable_types=selfrobot.traversable_types) #from utils : make cluster fontiers

        pos_list_float = [pos["position"] for pos in list(selfrobot.belief_space["robot_informations"].values())] #list of float xy position of all robots
        pos_list_int = [(int(x), int(y)) for x,y in pos_list_float] #same list with ints.
        weighted_clusters = wavefront_propagation_algorithm(selfrobot.belief_space["occupancy_grid"], (int(selfrobot.transform.x), int(selfrobot.transform.y)), pos_list_int, cluster_centers, weight_of_closer_robots=selfrobot.env.width, traversable_types=selfrobot.traversable_types) #the penalty for a frontier cluster depends of the size of the env.
        selfrobot.target = min(weighted_clusters, key=weighted_clusters.get) #then we take the cluster with the minimum cost
        selfrobot.last_plan_time = selfrobot.env.step