from RAPID.Environment import Environment, TargetPointEnvironment, ExplorationEnvironment
from RAPID import Agents

from PIL import Image

import logging
import random



# Define constants for the screen width and height
SCREEN_WIDTH = 100
SCREEN_HEIGHT = 100
BACKGROUND_COLOR = (200, 200, 200)

ENV_IMAGE_PATH = "/home/erwan/Documents/tests_simulations/RAPID/images_env/forest_100_100.png"# CHANGE THE PATH

NB_GROUND_AGENTS = 3

img = Image.open(ENV_IMAGE_PATH).convert("L")


logging.basicConfig(level=logging.DEBUG)

def main():
    
    #env = TargetPointEnvironment(background_color= BACKGROUND_COLOR, env_image=img, amount_of_agents_goal=2, scaling_factor=2)
    #env = ExplorationEnvironment(render= True, background_color= BACKGROUND_COLOR, env_image=img, full_knowledge=False, scaling_factor=4)
    env = ExplorationEnvironment(width=SCREEN_WIDTH, height=SCREEN_HEIGHT, env_image=None, background_color= BACKGROUND_COLOR, full_knowledge=False, scaling_factor=4, communication_mode="limited", render=True)


    for i in range(NB_GROUND_AGENTS):
        #random init position
        init_pos = (random.randrange(0,env.width), random.randrange(0,env.height), random.uniform(0, 2*3.14)) 


        env.add_agent(Agents.Ground(
                                    env, 
                                    robot_id=len(env.agents)+1, 
                                    init_transform=init_pos,
                                    max_speed=(2,0,0.5),
                                    behavior_to_use="minpos", #change to target_djikstra with a target point env.
                                    vision_range=10,
                                    communication_range=30,
                                    communication_period=10,
                                ))
        
    
    env.run()



if __name__ == "__main__":
    main()