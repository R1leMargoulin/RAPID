from RAPID.Environment import Environment, TargetPointEnvironment, ExplorationEnvironment
from RAPID import Agents

from PIL import Image

import logging
import random


SIMULATION_NAME = 'Multi-Robot-Exploration'
EXPERIMENT_NAME = 'multi_robot / Example 01'

# Define constants for the screen width and height
SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
BACKGROUND_COLOR = (200, 200, 200)

ENV_IMAGE_PATH = "/home/erwan/Documents/tests_simulations/RAPID/images_env/test_demo.png"

NB_GROUND_AGENTS = 20

img = Image.open(ENV_IMAGE_PATH).convert("L")


logging.basicConfig(level=logging.DEBUG)

def main():
    #env = Environment(background_color= BACKGROUND_COLOR, env_image=img)
    #env = TargetPointEnvironment(background_color= BACKGROUND_COLOR, env_image=img, amount_of_agents_goal=2)
    env = ExplorationEnvironment(background_color= BACKGROUND_COLOR, env_image=img)


    for i in range(NB_GROUND_AGENTS):
        #random init position
        init_pos = (random.randrange(0,env.width), random.randrange(0,env.height), random.uniform(0, 2*3.14)) 


        env.add_agent(Agents.Ground(
                                    env, 
                                    robot_id=len(env.agents)+1, 
                                    init_transform=init_pos,
                                    max_speed=(1,0,0.5),
                                    behavior_to_use="random"
                                ))
        
    
    env.run()

if __name__ == "__main__":
    main()