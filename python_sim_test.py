from RAPID.Environment import Environment
from RAPID import Agents

from PIL import Image


SIMULATION_NAME = 'Multi-Robot-Exploration'
EXPERIMENT_NAME = 'multi_robot / Example 01'

# Define constants for the screen width and height
SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
BACKGROUND_COLOR = (200, 200, 200)

ENV_IMAGE_PATH = "/home/erwan/Documents/tests_simulations/RAPID/images_env/test_500_500.png"

NB_GROUND_AGENTS = 200

img = Image.open(ENV_IMAGE_PATH).convert("L")


def main():
    env = Environment(background_color= BACKGROUND_COLOR, env_image=img)


    for i in range(NB_GROUND_AGENTS):
        env.add_agent(Agents.Ground(env=env))
        
    
    env.run()

if __name__ == "__main__":
    main()