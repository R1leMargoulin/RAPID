from RAPID.Environment import Environment, TargetPointEnvironment, ExplorationEnvironment #import of the different environments
from RAPID import Agents

from PIL import Image

import os
import logging
import random
import json


#ENVIRONMENT PARAMETERS----------------------------------------------------------------------------------------------------------------------------------
GROUP_EXPERIMENT_NAME = "minpos_blackboard_freq1_empty_100x100/"
EXPERIMENT_NAME = '3_robots'
RESULT_PATH = "/home/erwan/Documents/tests_simulations/RAPID/experiments/"#select an existing folder
NB_SIMULATION = 10


RENDER = True #display the simulation GUI or not.
SCALING_FACTOR = 3 #will scale the size of the window if render, by the scaling factor

# Environment setupk
SCREEN_WIDTH = 100
SCREEN_HEIGHT = 100
BACKGROUND_COLOR = (200, 200, 200)
#if there is an environment image, it will override the screen width and height.
#ENV_IMAGE_PATH = "/home/erwan/Documents/tests_simulations/RAPID/images_env/forest_100_100.png" #CHANGE THE PATH
ENV_IMAGE_PATH = None
STEP_LIMITATION = None

COMMUNICATION_MODE = "blackboard" #either "blackboard" or "limited"


def main():
        #AGENT PARAMETERS----------------------------------------------------------------------------------------------------------------------------------
    NB_GROUND_AGENTS = 3
    GROUND_AGENTS_INIT_POSITION = None#(0, 0, 0) #(x,y,w) 2D transform. set it to None for random position
    GROUND_AGENTS_MAX_SPEED = (2, 0, 0.5) #(x,y,w) speeds in 2D transform. if None : default value (1,0,0.5)
    GROUND_AGENTS_BEHAVIOR = "minpos" #random behavior if commented
    GROUND_AGENTS_COMMUNICATION_RANGE = 20 #won't be taken into acccount if the com mode is "blackboard"
    GROUND_AGENTS_COMMUNICATION_PERIOD = 1
    GROUND_AGENTS_VISION_RANGE = 20 # default 20.

    #-------------------------------------------------------------------------------------------------------------------------------------------
    #If the path doesn't exists, we create it, be careful with permission issues.
    if not os.path.exists(RESULT_PATH+GROUP_EXPERIMENT_NAME):
        os.mkdir(RESULT_PATH+GROUP_EXPERIMENT_NAME)

    if ENV_IMAGE_PATH:
        img = Image.open(ENV_IMAGE_PATH).convert("L")
    else:
        img = None

    logging.basicConfig(level=logging.INFO)




    #run loop of the experiments, you can change the environment type in the environment definition.
    for s in range(NB_SIMULATION):

        #ENVIRONMENT DEFINITION------------

        #env = ExplorationEnvironment(width=SCREEN_WIDTH, height=SCREEN_HEIGHT, background_color= BACKGROUND_COLOR) #example of an empty exploration environment defined by size
        #env = TargetPointEnvironment(background_color= BACKGROUND_COLOR, env_image=img, amount_of_agents_goal=2) #example definition of a target point environment
        #env = ExplorationEnvironment(render= True, background_color= BACKGROUND_COLOR, env_image=img, full_knowledge=False) #example of exploration environment without rendering.

        
        env = ExplorationEnvironment(width=SCREEN_WIDTH,
                                    height=SCREEN_HEIGHT,
                                    render= RENDER, 
                                    caption= EXPERIMENT_NAME,
                                    background_color= BACKGROUND_COLOR,
                                    env_image=img,
                                    limit_of_steps=STEP_LIMITATION,
                                    scaling_factor=SCALING_FACTOR,
                                    communication_mode=COMMUNICATION_MODE,
                                    ) #example of exploration environment with rendering (there is rendering by default)
        
        #---------------------------------

        #AGENTS Definition----------------
        for i in range(NB_GROUND_AGENTS):
            #random init position if it is not defined
            if GROUND_AGENTS_INIT_POSITION == None:
                agents_init_pos = (random.randrange(0,env.width), random.randrange(0,env.height), random.uniform(0, 2*3.14)) 
            else:
                agents_init_pos = GROUND_AGENTS_INIT_POSITION

            env.add_agent(Agents.Ground(
                                        env, 
                                        robot_id=len(env.agents)+1, 
                                        init_transform=agents_init_pos,
                                        max_speed=GROUND_AGENTS_MAX_SPEED,
                                        behavior_to_use=GROUND_AGENTS_BEHAVIOR,
                                        communication_range=GROUND_AGENTS_COMMUNICATION_RANGE,
                                        communication_period=GROUND_AGENTS_COMMUNICATION_PERIOD,
                                        vision_range=GROUND_AGENTS_VISION_RANGE,
                                    ))
        #---------------------------------

        env.run()
        #make data from the sim.
        create_simulation_data(env, s+1)

def create_simulation_data(env, sim_number:int,): #TODO, passer en CSV sera plus opti si on a beaucoup de données
    result_file_path = RESULT_PATH+GROUP_EXPERIMENT_NAME+EXPERIMENT_NAME+".json"
    if sim_number==1: #if file doesn't exists:
        with open(result_file_path, "w") as outfile:
            outfile.write(json.dumps({})) #then we create an empty json file

    sim_data = { #Here are detailed the different env metrics.
        f"simulation{sim_number}":{ 
        "total_steps":env.step, 
        "goal_reached":env.goal_condition(),
        "agents_data":{},
        "communication_mode": env.communication_mode,
        "size":[env.width, env.height]
        }
    }
    for a in env.agents: #here are detailed the different agent metrics.
        sim_data[f"simulation{sim_number}"]["agents_data"].update({
                                                                        f"robot_{a.robot_id}":{
                                                                            "total_distance_made" : a.total_distance_made,
                                                                            "communication_range" : a.communication_range,
                                                                            "communication_period": a.communication_period,
                                                                            "vision_range": a.vision_range,
                                                                            "max_speed": (a.max_speed.x, a.max_speed.y, a.max_speed.w),
                                                                        }
                                                                    })
    
    with open(result_file_path, "r") as outfile:
        json_from_file = outfile.read()
    
    file_obj =  json.loads(json_from_file)
    file_obj.update(sim_data) #adding the simulation data to the global experiment data

    json_exp_data = json.dumps(file_obj, indent=4) 

    with open(result_file_path, "w") as outfile:
        outfile.write(json_exp_data) #save in the file




if __name__ == "__main__":
    main()