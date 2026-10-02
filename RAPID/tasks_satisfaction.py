#from .Agents import Robot
import numpy as np


def get_communication_satisfaction(robot):
    oldest_infotime = robot.env.step
    for otherrobot in robot.belief_space["last_infos_matrix"][robot.robot_id]:
        if otherrobot == robot.robot_id:
            continue
        else:
            infotime = robot.belief_space["last_infos_matrix"][robot.robot_id][otherrobot]
            #print(infotime)
            if infotime <= oldest_infotime:
                oldest_infotime = infotime

    return np.exp(-(robot.env.step - oldest_infotime)/robot.env.step) #ca va pas ca reste bloqué à 0.3678794411...
    #return oldest_infotime/robot.env.step

def get_exploration_satisfaction(robot):
    #TODO : faire le compte des nouvelles cellules explorees depuis le dernier adaptation step.

    nb_current_known_cells = np.count_nonzero(robot.belief_space["occupancy_grid"]!=-1)

    if "last_adaptation_exploration_count" not in robot.belief_space:
        robot.belief_space.update({"last_adaptation_exploration_count": 0})
        robot.belief_space.update({"best_new_cells_count": nb_current_known_cells})

    
    nb_new_known_cells = nb_current_known_cells - robot.belief_space["last_adaptation_exploration_count"]

    satisfaction = nb_new_known_cells / robot.belief_space["best_new_cells_count"]

    #updates for BS
    robot.belief_space.update({"last_adaptation_exploration_count": nb_current_known_cells})
    if nb_new_known_cells > robot.belief_space["best_new_cells_count"]:
        robot.belief_space.update({"best_new_cells_count": nb_new_known_cells})

    return satisfaction

    # known_environment_portion = np.count_nonzero(robot.belief_space["occupancy_grid"]!=-1)/(robot.env.width*robot.env.height)
    # return known_environment_portion