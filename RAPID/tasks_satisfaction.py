from Agents import Robot
import numpy as np


def get_communication_satisfaction(robot:Robot):
    oldest_infotime = 0
    for robot in robot.belief_space["last_infos_matrix"][robot.robot_id]:
        if robot == robot.robot_id:
            continue
        else:
            infotime = robot.belief_space["last_infos_matrix"][robot.robot_id][robot]
            #print(infotime)
            if infotime < oldest_infotime:
                oldest_infotime = infotime

    return oldest_infotime/robot.env.step

def get_exploration_satisfaction(robot:Robot):
    known_environment_portion = np.count_nonzero(robot.belief_space["occupancy_grid"]!=-1)/(robot.env.width*robot.env.height)
    return known_environment_portion