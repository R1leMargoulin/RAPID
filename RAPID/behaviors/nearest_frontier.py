import numpy as np

from ..utils import find_frontier_cells, heuristic_frontier_distance
from .common import return_home_or_finish

def nearest_frontier(selfrobot):
    """
    compute a greedy nearest frontier algorithm: the target is the frontier with the lowest heuristic distance.
    """
    selfrobot.belief_transfer()

    frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types)

    if len(frontiers) == 0:
        return_home_or_finish(selfrobot)
        return

    position = (selfrobot.transform.x, selfrobot.transform.y)
    euclid = np.sqrt((position[0] - frontiers[:, 0])**2 + (position[1] - frontiers[:, 1])**2)
    best_score = np.inf
    best_index = None
    for index in np.lexsort((np.arange(len(frontiers)), euclid)):
        if euclid[index] > best_score:
            break
        f = frontiers[index]
        score = heuristic_frontier_distance(position, (f[0], f[1]), selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types)
        if score < best_score or (score == best_score and index < best_index):
            best_score = score
            best_index = index
    if best_index is not None:
        selfrobot.target = tuple(frontiers[best_index].tolist())
        selfrobot.last_plan_time = selfrobot.env.step
