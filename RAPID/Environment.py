import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
from pygame.sprite import Sprite, spritecollide, collide_circle
from pygame.locals import (K_ESCAPE, KEYDOWN)

from .utils import *
from .grid_variables import *

import numpy as np
from PIL import Image
from random import uniform, randrange
import logging
import time

COMMUNICATION_MODE_LIST = ["blackboard", "limited"]


class Environment():
    def __init__(self, render = True, width:int=100, height:int=100, background_color = (200,200,200), caption = f'RAPID', env_image:Image.Image = None, full_knowledge:bool=True, limit_of_steps=None, scaling_factor:int=1, communication_mode="blackboard"):
        """
        Environment Class represents the environment in which the agents are evolving, the user should add agents with the add_agent method before runing the env with the env one.\\
        Params : 
        - render:bool =  display the environment or not
        - width:int = (default 100) = width of the environment
        - heigth:int = (default 100) = height of the environment
        - background_color:(int,int,int) = rgb color of the backgroung
        - caption:str = name given to the env
        - env_image:PIL.Image.Image = image computed into environment (overrides the witdh and height)
        - full_knowledge:bool = the agent gets a copy of the whole environment in it's own memory or in blackboard if there is one.
        - limit_of_steps:int = step limitation in which the agent should reach it's goal.
        - scaling_factor:int = the display (display only) size of the screen is multiply by the scaling factor.
        - communication_mode:str = method of communication in ["blackboard", "limited"] :
            - "blackboard" : all robots share a blackboard in the environment, the knowledge is centralized on this blackboard
            - "limited":  Robots cannot share information on the blackboard, they need to keep their own belief of the environment state and share it with other robots when possible
        """
        self.render = render

        pygame.init()


        self.scaling_factor = scaling_factor
        self.clock = pygame.time.Clock()
        self.start_time = 0

        self.width = width
        self.height = height
        self.background_color = background_color

        self.agents = []
        self.obstacles = []
        self.interest_points = {}
        self.agents_tools = {}

        self.full_knowledge= full_knowledge

        self.limit_of_steps = limit_of_steps

        self.communication_mode = communication_mode


        if(env_image):
            self.width = env_image.size[0]
            self.height = env_image.size[1]
            self.real_occupancy_grid = np.zeros((self.width, self.height))
            
            self.create_env_from_image(env_image)
        else:
            self.real_occupancy_grid = np.zeros((self.width, self.height))
            if self.render:
                #self.screen = pygame.display.set_mode((self.width, self.height)) #BACKUP scaling
                self.screen = pygame.display.set_mode((self.width * self.scaling_factor, self.height * self.scaling_factor))
            else:
                self.screen = None

        
        self.obstacles_group = pygame.sprite.Group(self.obstacles)
        self.agent_group = pygame.sprite.Group()

        # handling of the communication mode string
        if self.communication_mode not in COMMUNICATION_MODE_LIST:
            logging.error(f"unknown communication mode for robot {self.robot_id}.\n list of available communication mode : {COMMUNICATION_MODE_LIST}")
            exit()

        if self.render:
            pygame.display.set_caption(caption)

        self.step = 0
        self.running = True

        pass

    def run(self):
        self.start_time = time.time()
        while self.running:
            # check user input events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                if event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self.running = False
            #will update all agents in the self.agents object
            self.step += 1
            self.update()

            if self.render:
                # draw all changes to the screen
                pygame.display.flip()
                self.clock.tick(24)         # wait until next frame (at 60 FPS)
            if self.limit_of_steps !=None and self.step >= self.limit_of_steps:
                print(f"goal not reach in the limited number of steps. srop at {self.step}")
                self.running = False
            
            print(f"step : {self.step}", end="\r")
        


        pygame.quit()

    def update(self):
        """
        will update all agents in the self.agents object
        """
        if self.render:
            self.screen.fill(self.background_color)

        for a in self.agents:
            a.update(self.screen)

        if self.render:
            for o in self.obstacles:  #BACKUP scaling
            #     self.screen.blit(o.image, o.rect)
                scaled_rect = pygame.Rect(o.rect.x * self.scaling_factor, o.rect.y * self.scaling_factor, o.rect.width * self.scaling_factor, o.rect.height * self.scaling_factor)
                self.screen.blit(pygame.transform.scale(o.image, scaled_rect.size), scaled_rect)

        if(self.goal_condition()):
            print(f"Goal reached in {self.step} steps!")
            self.running = False

        if self.communication_mode == "limited":
            self.limited_communication_update()
            
    def add_agent(self, agent):
        self.agents.append(agent)
        self.agent_group.add(self.agents[-1])

    def create_env_from_image(self, img):
        np_img = np.array(img)
        np_img = ~np_img  # invert black and white cause (255 will be white, but we want our obstacle to be 1 and free cell be 0)
        np_img[np_img > 0] = 1 #all non white cells are considered as obstacles.

        dims = np_img.shape
        self.width = dims[1]
        self.height = dims[0]

        if self.render:
            self.screen = pygame.display.set_mode((self.width * self.scaling_factor, self.height * self.scaling_factor))
        else:
            self.screen = None
        for l in range(len(np_img)) :
            for o in range(len(np_img[l])):
                if np_img[l][o] == 1:
                    self.create_wall(o,l)

    def create_wall(self, coord_x, coord_y):
        self.real_occupancy_grid[coord_x][coord_y] = OG_WALL

        sprite = pygame.sprite.Sprite()
        sprite.image = pygame.Surface((1, 1))
        sprite.image.fill((0, 0, 0))
        sprite.rect = pygame.Rect(coord_x,coord_y, 1,1)
        self.obstacles.append(sprite)
    
    def goal_condition(self):
        return False
    
    def limited_communication_update(self):
        """
        Available only with "limited" communication mode.
        """
        potential_links_dict = {} #in this dict, we'll append all unidirectional links : if agent B is in the com range of agent A, then the link A->B is created.
        for a in self.agents :
            a.connected_robots = [] #reset of the list of connected robots for the agent.
            potential_links_dict.update({str(a.robot_id):[]})
            in_range = spritecollide(a.communication_halo, self.agent_group, False, collide_circle) #we detect collision between communication_halo and other agents
            for robot in in_range:
                potential_links_dict[str(a.robot_id)].append(str(robot.robot_id)) #we append the robot if it's in range, the link A->B is created. 

        #for each agent if it has a connexion, we'll check the reciprocity of it.
        for agent in self.agents:
            for link in potential_links_dict[str(agent.robot_id)]:
                if str(agent.robot_id) in potential_links_dict[link] : #if reciprocity
                    #find the linked robot and append the loop robot to it.                    
                    for r in self.agents:
                        if str(r.robot_id) == link:
                            r.connected_robots.append(agent) #append mutually the robots in their connected robot list
                            agent.connected_robots.append(r)
                            break
                    potential_links_dict[link].remove(str(agent.robot_id)) #we remove the agent in the other agent's list in order to avoid double links.

        if self.render:
            for a in self.agents:
                for cr in a.connected_robots:
                    pygame.draw.line(self.screen, (255, 255, 255), (a.transform.x * self.scaling_factor, a.transform.y * self.scaling_factor), (cr.transform.x * self.scaling_factor, cr.transform.y * self.scaling_factor))
        pass
    

class TargetPointEnvironment(Environment):
    def __init__(self, render = True, width = 100, height = 100, background_color=(200, 200, 200), caption=f'simulation_target_point', env_image = None, limit_of_steps = None, scaling_factor:int=1, communication_mode="blackboard", target_point:tuple[int,int]=None, amount_of_agents_goal=1):
        """"
        Environment Class represents the environment in which the agents are evolving, the user should add agents with the add_agent method before runing the env with the env one.\\
        In this Environment, the Agents has to reach a target point in order to complete the mission.
        params : 
        - render:bool =  display the environment or not
        - width:int = (default 100) = width of the environment
        - heigth:int = (default 100) = height of the environment
        - background_color:(int,int,int) = rgb color of the backgroung
        - caption:str = name given to the env
        - env_image:PIL.Image.Image = image computed into environment (overrides the witdh and height)
        - full_knowledge:bool = the agent gets a copy of the whole environment in it's own memory or in blackboard if there is one.
        - limit_of_steps:int = step limitation in which the agent should reach it's goal.
        - scaling_factor:int = the display (display only) size of the screen is multiply by the scaling factor.
        - communication_mode:str = method of communication in ["blackboard", "limited"] :
            - "blackboard" : all robots share a blackboard in the environment, the knowledge is centralized on this blackboard
            - "limited":  Robots cannot share information on the blackboard, they need to keep their own belief of the environment state and share it with other robots when possible
        - target_point:tuple:(int,int) (default : random) : target points that has to be reached by agents
        - amount_of_agents:int (default : 1) : amount of agents that needs to reach the point in order to complete the mission.
        """

        super().__init__(render, width, height, background_color, caption, env_image, limit_of_steps=limit_of_steps, scaling_factor=scaling_factor, communication_mode=communication_mode)
        if target_point :
            self.init_target_point(x=target_point[0], y=target_point[1])
        else : #s'il n'y a pas de target point, on en génère un aléatoirement:
            self.init_target_point(x = randrange(0, self.width), y = randrange(0, self.height))

        self.amount_of_agent_goal = amount_of_agents_goal

    def update(self):

        super().update()
        if self.render:
            scaled_rect = pygame.Rect(self.target_point.rect.x * self.scaling_factor, self.target_point.rect.y * self.scaling_factor, self.target_point.rect.width * self.scaling_factor, self.target_point.rect.height * self.scaling_factor)
            self.screen.blit(pygame.transform.scale(self.target_point.image, scaled_rect.size), scaled_rect)
            #self.screen.blit(self.target_point.image, self.target_point.rect)BACKUP scaling

    def init_target_point(self, x, y):

        sprite = Sprite()
        sprite.image = pygame.Surface((4, 4))
        sprite.image.fill((255, 0, 0))
        sprite.rect = pygame.Rect(x, y, 4, 4)
        self.target_point = sprite


        while pygame.sprite.spritecollide(self.target_point, self.obstacles_group, False):
            logging.warning("Target point overlaps with an obstacle, reallocating it randomly.")
            self.target_point.rect.center = (randrange(0, self.width), randrange(0, self.height))
        
        self.real_occupancy_grid[self.target_point.rect.centerx][self.target_point.rect.centery] = OG_TARGET_POINT

    def goal_condition(self):
        if len(pygame.sprite.spritecollide(self.target_point, self.agent_group, False)) >= self.amount_of_agent_goal:
            logging.info (f"Goal reached at time : {round(time.time() - self.start_time, 2)}")
            return True
        else:
            return False

class ExplorationEnvironment(Environment):
    def __init__(self, render = True, width = 100, height = 100, background_color=(200, 200, 200), caption=f'simulation', env_image = None, full_knowledge = False, limit_of_steps=None, scaling_factor:int=1, communication_mode="blackboard", exploration_proportion_goal=0.995):
        """
        ExplorationEnvironment Class represents the environment in which the agents are evolving, the user should add agents with the add_agent method before runing the env with the env one.\\
        in this class, there is an exploration map matrix, full of zeros at the beginning of the simulation the goal for agents is to explore all the environment, simulation ends when the matrix is 99% of 1(representing explored cells)\\
        
        Params :
        
        - render:bool =  display the environment or not
        - width:int = (default 100) = width of the environment
        - heigth:int = (default 100) = height of the environment
        - background_color:(int,int,int) = rgb color of the backgroung
        - caption:str = name given to the env
        - env_image:PIL.Image.Image = image computed into environment (overrides the witdh and height)
        - full_knowledge:bool = the agent gets a copy of the whole environment in it's own memory or in blackboard if there is one.
        - limit_of_steps:int = step limitation in which the agent should reach it's goal.
        - scaling_factor:int = the display (display only) size of the screen is multiply by the scaling factor.
        - communication_mode:str = method of communication in ["blackboard", "limited"] :
            - "blackboard" : all robots share a blackboard in the environment, the knowledge is centralized on this blackboard
            - "limited":  Robots cannot share information on the blackboard, they need to keep their own belief of the environment state and share it with other robots when possible
        """
        super().__init__(render, width, height, background_color, caption, env_image, full_knowledge, limit_of_steps, scaling_factor, communication_mode=communication_mode)
        exp_map = np.zeros((self.width, self.height))
        self.interest_points.update({"exploration_map":exp_map})
        #on va pas mettre de fog sur les murs parce que la vision ne les traverse pas, si on a des murs plus épais que 2, alors il y aura toujours de la fog.
        self.interest_points["exploration_map"] += self.real_occupancy_grid

        self.fog_texture = pygame.Surface((1,1))
        self.fog_texture.fill((100, 100, 100))
        
        self.exploration_proportion_goal = exploration_proportion_goal
        self.exploration_completion = 0.0

    def run(self):
        #remove some fog around agents before launching
        for agent in self.agents:
            neighbours = agent.get_neighbors_pixels(distance = 20, stop_at_wall = True, self_inclusion = True)
            self.mark_explored_cells(neighbours)
        super().run()

    def update(self):
        super().update()
        for agent in self.agents:
            neighbours = agent.get_neighbors_pixels(distance = agent.vision_range, stop_at_wall = True, self_inclusion = True)
            self.mark_explored_cells(neighbours)
        #Il faut que les agents effacent la fog autours d'eux maintenant.
        if self.render:
            self.draw_fog()
        pass

    def draw_fog(self):
        unexplored_poses = np.where(self.interest_points["exploration_map"] == 0)
        for i in range(len(unexplored_poses[0])):
            # self.screen.blit(self.fog_texture, pygame.Rect(unexplored_poses[0][i], unexplored_poses[1][i], 1, 1)) #BACKUP scaling
            scaled_rect = pygame.Rect(unexplored_poses[0][i] * self.scaling_factor, unexplored_poses[1][i] * self.scaling_factor, self.scaling_factor, self.scaling_factor)
            self.screen.blit(pygame.transform.scale(self.fog_texture, scaled_rect.size), scaled_rect)
            
        pass
    
    def goal_condition(self):
        # print(np.count_nonzero(self.interest_points["exploration_map"]==1))
        # print(self.width*self.height)
        # print(np.count_nonzero(self.interest_points["exploration_map"]==1)/(self.width*self.height))
        self.exploration_completion = np.count_nonzero(self.interest_points["exploration_map"]==1)/(self.width*self.height)

        # print(f"COMPLETION : {self.exploration_completion}")
        # print(f"BB completion : {np.count_nonzero(self.agents_tools["blackboard"]["occupancy_grid"]!=-1)/(self.width*self.height)}")
        # print(self.exploration_completion)
        if (self.exploration_completion >= self.exploration_proportion_goal):
            return True
        else:
            return False

    def mark_explored_cells(self, cells):
        for cell in cells:
            self.interest_points["exploration_map"][cell[0]-1][cell[1]-1] = 1