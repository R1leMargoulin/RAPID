from ..utils import find_frontier_cells, cluster_frontier_cells, wavefront_propagation_algorithm
from .common import return_home_or_finish


def minpos(selfrobot): #from Bautin, 2012
    """
    Adaptation from MinPos algorithm (Bautin, Simonin, Charpillet : 2012)
    Frontier based behavior where:
    - The frontiers are grouped into clusters
    - each cluster is given a cost depending on the distance and on robots that are closer to this frontier using the wavefront propagation algorithm (WPA)
    - the robot chose the frontier with the lowest cost
    """
    frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types)

    if len(frontiers) == 0:
        return_home_or_finish(selfrobot)
    else:
        cluster_centers = cluster_frontier_cells(selfrobot.belief_space["occupancy_grid"], frontiers, int(selfrobot.vision_range/2), traversable_types=selfrobot.traversable_types)

        pos_list_float = [pos["position"] for pos in list(selfrobot.belief_space["robot_informations"].values())]
        pos_list_int = [(int(x), int(y)) for x,y in pos_list_float]
        weighted_clusters = wavefront_propagation_algorithm(selfrobot.belief_space["occupancy_grid"], (int(selfrobot.transform.x), int(selfrobot.transform.y)), pos_list_int, cluster_centers, weight_of_closer_robots=selfrobot.env.width, traversable_types=selfrobot.traversable_types)
        selfrobot.target = min(weighted_clusters, key=weighted_clusters.get)
        selfrobot.last_plan_time = selfrobot.env.step
