from RAPID.Environment import Environment, TargetPointEnvironment, ExplorationEnvironment #import of the different environments
from RAPID import Agents

from PIL import Image

import logging
import random
import json

# PARAMS ----------------------------------------------------------------------------------------------------------------------------------
EXPERIMENT_NAME = 'multi_robot_forest_100x100'
RESULT_PATH = "./experiments/"#select an existing folder
NB_SIMULATION = 5

# Environment setup
SCREEN_WIDTH = 200
SCREEN_HEIGHT = 200
BACKGROUND_COLOR = (200, 200, 200)
#if there is an environment image, it will override the screen width and height.
ENV_IMAGE_PATH = "/home/erwan/Documents/tests_simulations/RAPID/images_env/forest_100_100.png"
STEP_LIMITATION = 1500

#GroundAgents
NB_GROUND_AGENTS = 5
GROUND_AGENTS_INIT_POSITION = (0, 0, 0) #(x,y,w) 2D transform. set it to None for random position
GROUND_AGENTS_MAX_SPEED = (2, 0, 0.5) #(x,y,w) speeds in 2D transform. if None : default value (1,0,0.5)
GROUND_AGENTS_BEHAVIOR = "nearest_frontier" #random behavior if commented
GROUND_AGENTS_COMMUNICATION_MODE = "blackboard" #don't touch this one, other modes are not implemented yet.
GROUND_AGENTS_VISION_RANGE = 20 # default 20.

#-------------------------------------------------------------------------------------------------------------------------------------------


img = Image.open(ENV_IMAGE_PATH).convert("L")
logging.basicConfig(level=logging.INFO)

def main():
    #run loop of the experiments, you can change the environment type in the environment definition.
    for s in range(NB_SIMULATION):

        #ENVIRONMENT DEFINITION------------

        #env = ExplorationEnvironment(width=SCREEN_WIDTH, height=SCREEN_HEIGHT, background_color= BACKGROUND_COLOR) #example of an empty exploration environment defined by size
        #env = TargetPointEnvironment(background_color= BACKGROUND_COLOR, env_image=img, amount_of_agents_goal=2) #example definition of a target point environment
        #env = ExplorationEnvironment(render= True, background_color= BACKGROUND_COLOR, env_image=img, full_knowledge=False) #example of exploration environment without rendering.

        
        env = ExplorationEnvironment(render= True, 
                                    caption= EXPERIMENT_NAME,
                                    background_color= BACKGROUND_COLOR,
                                    env_image=img,
                                    limit_of_steps=STEP_LIMITATION
                                    ) #example of exploration environment with rendering (there is rendering by default)
        
        #---------------------------------

        #AGENTS Definition----------------
        for i in range(NB_GROUND_AGENTS):
            #random init position
            init_pos = (random.randrange(0,env.width), random.randrange(0,env.height), random.uniform(0, 2*3.14)) 


            env.add_agent(Agents.Ground(
                                        env, 
                                        robot_id=len(env.agents)+1, 
                                        init_transform=init_pos,
                                        max_speed=(2,0,0.5),
                                        behavior_to_use="nearest_frontier",
                                        communication_mode="blackboard",
                                    ))
        #---------------------------------
        env.run()
        #make data from the sim.
        create_simulation_data(env, s+1)

def create_simulation_data(env, exp_number:int,): #TODO, passer en CSV sera plus opti si on a beaucoup de données
    result_file_path = RESULT_PATH+EXPERIMENT_NAME+".json"
    if exp_number==1: #if file doesn't exists:
        with open(result_file_path, "w") as outfile:
            outfile.write(json.dumps({})) #then we create an empty json file

    exp_data = { #Here are detailed the different env metrics.
        f"experiment{exp_number}":{ 
        "total_steps":env.step, 
        "goal_reached":env.goal_condition(),
        "agents_data":{}
        }
    }
    for a in env.agents: #here are detailed the different agent metrics.
        exp_data[f"experiment{exp_number}"]["agents_data"].update({
            f"robot_{a.robot_id}":{
                "total_distance_made" : a.total_distance_made
            }
        })
    
    with open(result_file_path, "r") as outfile:
        json_from_file = outfile.read()
    
    file_obj =  json.loads(json_from_file)
    file_obj.update(exp_data) #adding the simulation data to the global experiment data

    json_exp_data = json.dumps(file_obj, indent=4) 

    with open(result_file_path, "w") as outfile:
        outfile.write(json_exp_data) #save in the file




if __name__ == "__main__":
    main()