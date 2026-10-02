import logging

import numpy as np

from ..utils import find_frontier_cells, cluster_frontier_cells, euclidian_distance, manhathan_distance, simple_clustering, a_star_cost, max_k
from .common import return_home_or_finish

def action_selection(selfrobot):
    reshape_com_importance_for_action_selection(selfrobot, mode = selfrobot.com_importance_mode)

    interest_points = []
    frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types)
    if len(frontiers) > 0:
        cluster_centers = cluster_frontier_cells(selfrobot.belief_space["occupancy_grid"], frontiers, int(selfrobot.vision_range/2), traversable_types=selfrobot.traversable_types)
        for cc in cluster_centers:
            interest_points.append({"type":"exploration","coordinates":cc, "needed_robots":1})

    #Artifacts ----------------------------
    if "artifacts" in selfrobot.belief_space:
        for art in selfrobot.belief_space["artifacts"]:
            #IMPORTANCE CHECK
            if art +1 <= len(selfrobot.env.interest_points["artifacts"]): #ouais c'est degueu
                selfrobot.env.interest_points["artifacts"][art].check_importance(selfrobot)

            #INTEREST POINT CREATION
            if selfrobot.belief_space["artifacts"][art]["status"] not in ["done", "destroyed"] :
                if euclidian_distance( (selfrobot.init_transform.x, selfrobot.init_transform.y) , selfrobot.belief_space["artifacts"][art]["coordinates"]) >= selfrobot.competences[selfrobot.belief_space["artifacts"][art]["type"]]["distance_treshold"]: #we verify that the treshold is respected
                    interest_points.append({"type": selfrobot.belief_space["artifacts"][art]["type"] ,
                                            "coordinates":selfrobot.belief_space["artifacts"][art]["coordinates"], 
                                            "id":art, 
                                            "needed_robots": selfrobot.belief_space["artifacts"][art]["needed_robots"],
                                            "step": selfrobot.belief_space["artifacts"][art]["step"],
                                            "discovery_time": selfrobot.belief_space["artifacts"][art]["discovery_time"],
                                            "awared": selfrobot.belief_space["artifacts"][art]["awared"]})#adding directly the artifacts in the interest points
    #--------------------------------------
    #------------------------------------------------------------------------------------------


    #barycentre de communications----------
    #liste de toutes les positions des robots

    robots_pos_list = [] #list of float xy position of all robots
    for robot_id in selfrobot.belief_space["robot_informations"]:
        if robot_id != selfrobot.robot_id and selfrobot.belief_space["robot_informations"][robot_id]["status"]!= "finishing":  # ComImportance : est ce que je ferais pas un truc spécifique aux robots?
            if selfrobot.communication_range*3/selfrobot.max_speed.x <= selfrobot.env.step - selfrobot.belief_space["robot_informations"][robot_id]["step"] < 4 * ( np.max(selfrobot.belief_space["occupancy_grid"].shape)/selfrobot.max_speed.x ) : #time = dist/speed
                if euclidian_distance((selfrobot.transform.x, selfrobot.transform.y) ,selfrobot.belief_space["robot_informations"][robot_id]["position"]) >=  selfrobot.competences["communication"]["distance_treshold"]:
                    robots_pos_list.append(selfrobot.belief_space["robot_informations"][robot_id]["position"])
        if len(robots_pos_list)>0:
            if euclidian_distance((selfrobot.transform.x, selfrobot.transform.y) , selfrobot.last_given_position) >=  selfrobot.competences["communication"]["distance_treshold"]:
                    robots_pos_list.append(selfrobot.last_given_position)# TODO ComInfo, la last given position, c'est a double trnchant, je sais pas trop

    communication_clusters = simple_clustering(robots_pos_list, selfrobot.communication_range) #from utils: make simple clusters of robot based on communication range, will return the center of clusters
    for cc in communication_clusters:
            interest_points.append({"type":"communication","coordinates":cc, "needed_robots":1})#adding those clusters in the communication points
            #TODO MultiRobotTask : try different values of needed robots


    #--------------------------------------        

    if len(interest_points) == 0:
        return_home_or_finish(selfrobot, set_finishing_status=True)
        return None

    for ip, utility in zip(interest_points, compute_utilities(selfrobot, interest_points)):
        ip.update({"utility":utility})

    best_action = None
    best_weighted_utility = -np.inf
    for ip in interest_points:
        weighted_utility = ip["utility"] * selfrobot.competences[ip["type"]]["importance"]

        if weighted_utility >= best_weighted_utility:
            best_weighted_utility = weighted_utility
            best_action = ip

    if best_action != None:
        selfrobot.action_to_perform = best_action
        selfrobot.target = (int(selfrobot.action_to_perform["coordinates"][0]), int(selfrobot.action_to_perform["coordinates"][1]))
        selfrobot.last_plan_time = selfrobot.env.step
    else:
        logging.warning("action_selection: no best action found")


def kth_largest(sorted_desc, k):
    """k-th largest element of an array sorted in descending order, 0 if k exceeds its length (same as max_k)."""
    return sorted_desc[k-1] if k <= len(sorted_desc) else 0


def compute_utilities(selfrobot, interest_points):
    infos = selfrobot.belief_space["robot_informations"]
    grid = selfrobot.belief_space["occupancy_grid"]
    others = [robot for robot in infos if robot != selfrobot.robot_id]
    single_robot = len(infos) <= 1

    origins = [(int(selfrobot.transform.x), int(selfrobot.transform.y))]
    eases = [selfrobot.env_ease]
    traversables = [selfrobot.traversable_types]
    for robot in others:
        origins.append((int(infos[robot]["position"][0]), int(infos[robot]["position"][1])))
        eases.append(infos[robot]["env_ease"])
        traversables.append(infos[robot]["traversable_types"])
    targets = [(int(ip["coordinates"][0]), int(ip["coordinates"][1])) for ip in interest_points]
    costs = cost_distance_matrix(origins, targets, grid, eases, traversables, selfrobot.cost_calculation_mode)
    costs = np.maximum(costs, 1).tolist()

    utilities = []
    for n, ip in enumerate(interest_points):
        capability = selfrobot.competences[ip["type"]]["capability"]
        individual_utility = capability/costs[0][n]

        if single_robot:
            other_values = [1.0]
        else:
            other_values = []
            for k, robot in enumerate(others, start=1):
                if ip["needed_robots"] <= 1 or robot in ip["awared"]:
                    other_values.append(infos[robot]["competences"][ip["type"]]["capability"]/costs[k][n])
        other_individual_values = np.array(other_values, dtype=float)

        if len(other_individual_values) >= ip["needed_robots"]:
            collective_sufficiency = float(max_k(other_individual_values, ip["needed_robots"]))
        elif len(other_individual_values) == ip["needed_robots"]-1:
            collective_sufficiency = 1
        else:
            collective_sufficiency = np.inf

        if collective_sufficiency == 0:
            collective_sufficiency = 1e-8 #avoid divide by 0

        required_assist = 0
        if ip["needed_robots"] > 1:
            if len(other_individual_values) >= ip["needed_robots"]-1:
                sorted_values = np.sort(other_individual_values)[::-1]
                bests_others = []
                nbcloser = 0
                for i in range(len(infos)):
                    value = float(kth_largest(sorted_values, i+1))
                    if i < ip["needed_robots"] -1:
                        bests_others.append(value)
                    if value > individual_utility:
                        nbcloser+=1
                if nbcloser >= ip["needed_robots"]:
                    required_assist = -np.inf
                else:
                    required_assist = float(np.sum(bests_others))
            else:
                required_assist = - np.inf

        utilities.append((individual_utility + required_assist) / collective_sufficiency)
    return utilities


def cost_distance_matrix(origins, targets, grid, eases, traversables, mode="euclidian"):
    """cost of going from each origin to each target, as an (origins x targets) float array."""
    if mode == "astar":
        return np.array([[cost_distance_calculation(o, t, grid, eases[r], traversables[r], mode) for t in targets] for r, o in enumerate(origins)], dtype=float).reshape(len(origins), len(targets))
    if mode not in COST_DISTANCE_FUNCTIONS:
        raise ValueError(f"unknown cost calculation mode : {mode}")
    o = np.array(origins, dtype=int).reshape(-1, 2)
    t = np.array(targets, dtype=int).reshape(-1, 2)
    dx = o[:, None, 0] - t[None, :, 0]
    dy = o[:, None, 1] - t[None, :, 1]
    if mode == "euclidian":
        return np.sqrt(dx**2 + dy**2)
    return (np.absolute(dx) + np.absolute(dy)).astype(float)


def reshape_com_importance_for_action_selection(selfrobot, mode="default"):
    if mode not in COM_IMPORTANCE_MODES:
        raise ValueError(f"incorrect importance com mode in agent {selfrobot.robot_id}")

    capability = selfrobot.competences["communication"]["capability"]
    distance_treshold = selfrobot.communication_range 

    longest_infotime = 0
    for robot in selfrobot.belief_space["last_infos_matrix"][selfrobot.robot_id]:
        if robot == selfrobot.robot_id:
            continue
        else:
            infotime = selfrobot.belief_space["last_infos_matrix"][selfrobot.robot_id][robot]
            if selfrobot.env.step - infotime > longest_infotime:
                longest_infotime = selfrobot.env.step - infotime

    com_time = longest_infotime #longest synchro from every robots synchro time # ComImportance
    importance = COM_IMPORTANCE_MODES[mode](selfrobot, com_time)
    selfrobot.shape_competence("communication", capability=capability , importance=importance, distance_treshold=distance_treshold)


def _rule_based_importance(selfrobot, com_time):
    if com_time < 30:
        return 0
    if com_time >= np.sqrt(np.count_nonzero(selfrobot.belief_space["occupancy_grid"] != -1))/np.mean([selfrobot.max_speed.x, selfrobot.max_speed.y]):
        return np.inf
    return 1.5*com_time - selfrobot.env.step


COM_IMPORTANCE_MODES = {
    "default": lambda r, t: np.exp(t/ r.env.width),
    "constant": lambda r, t: 1,
    "constant2": lambda r, t: 2,
    "constant0.5": lambda r, t: 0.5,
    "linear0.5": lambda r, t: 0.5*t - r.env.step/3,
    "linear": lambda r, t: t - r.env.step/2,
    "linear1.5": lambda r, t: 1.5*t - r.env.step/2,
    "linear2": lambda r, t: 2*t - r.env.step/2,
    "polynomial2": lambda r, t: ((t/r.communication_range)**2)-r.env.step,
    "polynomial3": lambda r, t: ((t/r.communication_range)**3)-r.env.step,
    "exponential": lambda r, t: np.exp(t/ r.communication_range)/r.env.step,
    "rule-based": _rule_based_importance,
    "test": lambda r, t: np.exp(t/ r.communication_range),
}


COST_DISTANCE_FUNCTIONS = {
    "euclidian": euclidian_distance,
    "manhathan": manhathan_distance,
}


def cost_distance_calculation(pointA, pointB, grid, env_ease, traversable_types, mode="euclidian"):
    if mode == "astar":
        return float(a_star_cost(grid, pointA, pointB, env_ease, traversable_types))
    if mode in COST_DISTANCE_FUNCTIONS:
        return float(COST_DISTANCE_FUNCTIONS[mode](pointA, pointB))
    raise ValueError(f"unknown cost calculation mode : {mode}")
