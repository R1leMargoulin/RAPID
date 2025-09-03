from RAPID.Environment import Environment, TargetPointEnvironment, ExplorationEnvironment
from RAPID import Agents

from PIL import Image

import logging
import random



# Define constants for the screen width and height
SCREEN_WIDTH = 100
SCREEN_HEIGHT = 100
BACKGROUND_COLOR = (200, 200, 200)

ENV_IMAGE_PATH = "/home/erwan/Documents/RAPID/examples/env_images_example/cave_200_200.png"# CHANGE THE PATH

NB_GROUND_AGENTS = 6
NB_AERIAL_AGENTS = 2

pos_list = [[85,190,0], [95,190,0], [105,190,0], [115,190,0], [125,190,0], [100,180,0]]
#pos_list = None

img = Image.open(ENV_IMAGE_PATH)


logging.basicConfig(level=logging.DEBUG)

def main():
    
    #env = TargetPointEnvironment(background_color= BACKGROUND_COLOR, env_image=img, amount_of_agents_goal=2, scaling_factor=2)
    #env = ExplorationEnvironment(render= True, background_color= BACKGROUND_COLOR, env_image=img, full_knowledge=False, scaling_factor=4)
    env = ExplorationEnvironment(width=SCREEN_WIDTH, height=SCREEN_HEIGHT, env_image=img, background_color= BACKGROUND_COLOR, full_knowledge=False, scaling_factor=4, communication_mode="limited", render=True, end_at_full_exploation=False)


    for i in range(NB_GROUND_AGENTS):
        #random init position
        if pos_list :
            init_pos = pos_list[i]
        else:
            init_pos = (random.randrange(0,env.width), random.randrange(0,env.height), random.uniform(0, 2*3.14)) 
        #init_pos = (125, 190, 0)


        env.add_agent(Agents.Ground(
                                    env, 
                                    robot_id=len(env.agents)+1, 
                                    init_transform=init_pos,
                                    max_speed=(1,1,0.5), #warning : adapt max speeds if you use aerial or ground robots
                                    behavior_to_use="action_selection", 
                                    vision_range=10,
                                    communication_range=20,
                                    communication_period=10,
                                    energy_amount=1e6, #place a huge amount for no energy limitations 
                                    altruism=0.3
                                ))
        env.agents[-1].shape_competence("exploration", 1, 1)

    # for i in range(NB_AERIAL_AGENTS):
    #     #random init position
    #     init_pos = (random.randrange(0,env.width), random.randrange(0,env.height), random.uniform(0, 2*3.14)) 
    #     #init_pos = (125, 190, 0)


    #     env.add_agent(Agents.Aerial(
    #                                 env, 
    #                                 robot_id=len(env.agents)+1, 
    #                                 init_transform=init_pos,
    #                                 max_speed=(1,1,0.5), #warning : adapt max speeds if you use aerial or ground robots
    #                                 behavior_to_use="minpos", 
    #                                 vision_range=10,
    #                                 communication_range=30,
    #                                 communication_period=10,
    #                                 energy_amount=1e6 #place a huge amount for no energy limitations 
    #                             ))
    #     env.agents[-1].shape_competence("exploration", 1, 1)
        
    
    env.run()



if __name__ == "__main__":
    main()