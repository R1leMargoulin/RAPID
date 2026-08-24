#from .Agents import Robot
import numpy as np


def get_communication_satisfaction(robot):
    oldest_infotime = 0
    for otherrobot in robot.belief_space["last_infos_matrix"][robot.robot_id]:
        if otherrobot == robot.robot_id:
            continue
        else:
            infotime = robot.belief_space["last_infos_matrix"][robot.robot_id][otherrobot]
            #print(infotime)
            if infotime < oldest_infotime:
                oldest_infotime = infotime

    return oldest_infotime/robot.env.step

def get_exploration_satisfaction(robot):
    known_environment_portion = np.count_nonzero(robot.belief_space["occupancy_grid"]!=-1)/(robot.env.width*robot.env.height)
    return known_environment_portion