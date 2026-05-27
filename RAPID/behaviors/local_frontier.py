from ..utils import *

import numpy as np

def local_frontier(selfrobot):
    """
    adaptation from local frontier algorithm (Gauville, Charpillet : 2019)
    """
    #setup init pos if there is not.
    if not ("traces" in selfrobot.belief_space): #then init the traces in belief space
        init_pos = (int(selfrobot.init_transform.x), int(selfrobot.init_transform.y))
        selfrobot.belief_space["traces"] = {init_pos:selfrobot.env.step} #here we init the trace with a dictionarry: the key is the position the value is the timestamp (sim step)

        #init second chance used as False
        selfrobot.belief_space["second_chance_usage"] = False

    #SENSING
    selfrobot.sense()
    #LOCAL FRONTIER DETECTION -----------------------------------------------------
    vision_range = selfrobot.get_neighbors_pixels(distance=selfrobot.vision_range, stop_at_wall=True, selfrobot_inclusion=True)
    local_frontier_list = []
    for cell in vision_range:
        if not(selfrobot.belief_space["occupancy_grid"][cell[0]][cell[1]] in selfrobot.traversable_types):
            #if it's a wall, we skip this cell.
            continue

        cell_neighbors = get_direct_neighbors(cell, width=selfrobot.env.width, height=selfrobot.env.height) #improvable : pour plus de realisme on pourrait mettre la taille du belief space plutot que directement l'env.

        for cn in cell_neighbors: #maximum 4 neighbors per cell
            if selfrobot.belief_space["occupancy_grid"][cn[0]][cn[1]] == OG_UNKNOWN_CELL: #if the cell has an unknown cell as neighbor, it becomes a frontier.
                #we add the cell to the frontier list if it is a local frontier.
                local_frontier_list.append(cell)
                break
                
    #-------------------------------------------------------------------------------
    if local_frontier_list:
    #go to the most far local frontier from the traces
        max_dist_of_lf = 0
        selected_frontier = None
        mean_traces_coordinates = (int(np.mean([c[0] for c in selfrobot.belief_space["traces"].keys()])), int(np.mean([c[1] for c in selfrobot.belief_space["traces"].keys()]))) #mean coordinates of all the traces.
        for lf in local_frontier_list:
            if euclidian_distance(lf, mean_traces_coordinates)> max_dist_of_lf: #if the distance (we take euclidian) of the LF from the robot is greater, then we select it
                max_dist_of_lf = euclidian_distance(lf, mean_traces_coordinates)
                selected_frontier = lf
        selfrobot.target = selected_frontier
    else: #else if there is no frontier:
        if euclidian_distance((int(selfrobot.transform.x),int(selfrobot.transform.y)), (int(selfrobot.init_transform.x),int(selfrobot.init_transform.y))) <= selfrobot.treshold_for_target: #if we are back at the init pose, the robot has finished.
            if selfrobot.belief_space["second_chance_usage"] == True:
                selfrobot.finish()
            else:
                #we use a second chance:
                selfrobot.belief_space["second_chance_usage"] = True
                
                mean_traces_coordinates = (int(np.mean([c[0] for c in selfrobot.belief_space["traces"].keys()])), int(np.mean([c[1] for c in selfrobot.belief_space["traces"].keys()]))) #mean coordinates of all the traces.
                max_dist = 0
                second_chance_target = None
                for cell in vision_range:
                    if selfrobot.belief_space["occupancy_grid"][cell[0]][cell[1]] != OG_WALL:
                        if euclidian_distance(cell, mean_traces_coordinates)> max_dist:
                            max_dist = euclidian_distance(cell, mean_traces_coordinates)
                            second_chance_target = cell
                selfrobot.target = second_chance_target
                selfrobot.last_plan_time = selfrobot.env.step

        else: #else go back to the previous trace -> set it as target
            # pour les cases voisine de distance ou le robot à pu se déplacer sur un step de simulation (sur une periode de temps donné, on récolte les voisins)
            move_possible_neighbors =  selfrobot.get_neighbors_pixels(distance=int(max(4*selfrobot.max_speed.x, 4*selfrobot.max_speed.y)), stop_at_wall=True, selfrobot_inclusion=False)
            chosen_trace = None
            oldest_timestep = np.inf
            for cell in move_possible_neighbors : #on va prendre la trace la plus ancienne possible dans ce champs
                if cell in selfrobot.belief_space["traces"]: #check if the cell is registered in the traces or we would have an error
                    if selfrobot.belief_space["traces"][cell] < oldest_timestep:
                        chosen_trace = cell
                        oldest_timestep = selfrobot.belief_space["traces"][cell]
            selfrobot.target = chosen_trace #on definit la trace la plus ancienne dans le rayon restreint défini.
            selfrobot.last_plan_time = selfrobot.env.step

    selfrobot.belief_transfer() #belief transfer management.