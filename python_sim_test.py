from RAPID.Environment import Environment, TargetPointEnvironment
from RAPID import Agents

from PIL import Image

import logging


SIMULATION_NAME = 'Multi-Robot-Exploration'
EXPERIMENT_NAME = 'multi_robot / Example 01'

# Define constants for the screen width and height
SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
BACKGROUND_COLOR = (200, 200, 200)

ENV_IMAGE_PATH = "/home/erwan/Documents/tests_simulations/RAPID/images_env/test_500_500.png"

NB_GROUND_AGENTS = 1

img = Image.open(ENV_IMAGE_PATH).convert("L")


logging.basicConfig(level=logging.DEBUG)

def main():
    #env = Environment(background_color= BACKGROUND_COLOR, env_image=img)

    env = TargetPointEnvironment(background_color= BACKGROUND_COLOR, env_image=img, amount_of_agents_goal=1)


    for i in range(NB_GROUND_AGENTS):
        env.add_agent(Agents.Ground(robot_id=len(env.agents)+1, env=env))
        
    
    env.run()

if __name__ == "__main__":
    main()