from RAPID.Environment import Environment, TargetPointEnvironment, ExplorationEnvironment
from RAPID import Agents

from PIL import Image

import logging
import random



# Define constants for the screen width and height
SCREEN_WIDTH = 100
SCREEN_HEIGHT = 100
BACKGROUND_COLOR = (200, 200, 200)

ENV_IMAGE_PATH = "/home/erwan/Documents/RAPID/examples/env_images_example/labyrinth_100_100.png"# CHANGE THE PATH
#ENV_IMAGE_PATH = "/home/erwan/Documents/RAPID/tests/test_envs/large_indoor_half1.png"# CHANGE THE PATH

#video_saving_path = "/home/erwan/Documents/RAPID/tests/records/4grounds/"
video_saving_path = None

NB_GROUND_AGENTS = 4
NB_AERIAL_AGENTS = 0

#cave
#pos_list = [[85,190,0], [95,190,0], [105,190,0], [115,190,0], [125,190,0], [85,192,0], [95,192,0], [105,192,0], [115,192,0], [125,192,0], [85,188,0], [95,188,0], [105,188,0], [115,188,0], [125,188,0]]

#indoor_large
#pos_list = [[50,150,0], [50,155,0], [50,160,0], [50,165,0], [50,170,0], [53,150,0], [53,155,0], [53,160,0], [53,165,0], [53,170,0], [56,150,0], [56,155,0], [56,160,0], [56,165,0], [56,170,0]]

#turtle map
#pos_list = [[50,10,0], [50,12,0], [50,14,0], [50,16,0], [50,18,0], [53,10,0], [53,12,0], [53,14,0], [53,16,0], [53,18,0], [55,10,0], [55,12,0], [55,14,0], [55,16,0], [55,18,0], [57,10,0], [57,12,0], [57,14,0], [57,16,0], [57,18,0], [60,10,0], [60,12,0], [60,14,0], [60,16,0], [60,18,0], [63,10,0], [63,12,0], [63,14,0], [63,16,0], [63,18,0]]
pos_list = [[5,5,0], [5,8,0], [5,12,0], [5,15,0]]
aerial_pos_list = [[55,150,0], [55,155,0], [55,160,0], [55,165,0]]
#pos_list = None

img = Image.open(ENV_IMAGE_PATH)
if img:
    img = Image.open(ENV_IMAGE_PATH).convert('RGB')

logging.basicConfig(level=logging.DEBUG)

def main():
    
    #env = TargetPointEnvironment(background_color= BACKGROUND_COLOR, env_image=img, amount_of_agents_goal=2, scaling_factor=2)
    #env = ExplorationEnvironment(render= True, background_color= BACKGROUND_COLOR, env_image=img, full_knowledge=False, scaling_factor=4)
    env = ExplorationEnvironment(width=SCREEN_WIDTH, height=SCREEN_HEIGHT, env_image=img, background_color= BACKGROUND_COLOR, full_knowledge=False, scaling_factor=4, communication_mode="limited", communication_reliability=0.5, render=True, end_at_full_exploation=False, save_img_steps=video_saving_path)


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
                                    vision_range=25,
                                    communication_range=35,
                                    communication_period=5,
                                    energy_amount=1e6, #place a huge amount for no energy limitations 
                                    delta_replan= 10
                                ))
        env.agents[-1].shape_competence("exploration", 1, 1)

    for i in range(NB_AERIAL_AGENTS):
        #random init position
        if aerial_pos_list :
            init_pos = aerial_pos_list[i]
        else:
            init_pos = (random.randrange(0,env.width), random.randrange(0,env.height), random.uniform(0, 2*3.14)) 


        env.add_agent(Agents.Aerial(
                                    env, 
                                    robot_id=len(env.agents)+1, 
                                    init_transform=init_pos,
                                    max_speed=(1,1,0.5), #warning : adapt max speeds if you use aerial or ground robots
                                    behavior_to_use="action_selection", 
                                    vision_range=10,
                                    communication_range=30,
                                    communication_period=10,
                                    energy_amount=1e6 #place a huge amount for no energy limitations 
                                ))
        env.agents[-1].shape_competence("exploration", 1, 1)
        
    
    env.run()



if __name__ == "__main__":
    main()