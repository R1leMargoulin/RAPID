import pygame
from pygame.sprite import Sprite
from pygame.locals import (K_ESCAPE, KEYDOWN)

from .utils import *

import numpy as np
from PIL import Image
from random import uniform, randrange
import logging
import time



class Environment():
    def __init__(self, width:int=100, height:int=100, background_color = (200,200,200), caption = f'simulation', env_image:Image.Image = None, communication_level="full", full_knowledge:bool=True, limit_of_steps=None):
        """"
        Environment Class represents the environment in which the agents are evolving, the user should add agents with the add_agent method before runing the env with the env one.\\
        Params : 
        - width:int (default 100) = width of the environment
        - heigth:int (default 100) = height of the environment
        - background_color:(int,int,int) = rgb color of the backgroung
        - caption:str = name given to the env
        - env_image:PIL.Image.Image = image computed into environment (overrides the witdh and height)
        - communication_level:str (default "full") = degree of communication between agents:
            - "full : Agents can all communicate no matter distance
            - "range-limited" : communications between robots is limited by a distance radius.
            - "none" : no communication between agents at all
        - full_knowledge:bool (default True) = If True, the full environment map will be given to the robot in it's belief space at it's init.
        """
        #env init
        pygame.init()
        self.clock = pygame.time.Clock()
        self.start_time = 0

        self.width = width
        self.height = height
        self.background_color = background_color

        self.agents = []
        self.obstacles = []
        self.interest_points = {}

        self.communication_level = communication_level
        self.full_knowledge= full_knowledge

        self.limit_of_steps = limit_of_steps
        

        if(env_image):
            self.create_env_from_image(env_image)
        else:
            self.screen = pygame.display.set_mode((self.width, self.height))



        self.obstacles_group = pygame.sprite.Group(self.obstacles)
        self.agent_group = pygame.sprite.Group()

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
            # draw all changes to the screen
            pygame.display.flip()
            self.clock.tick(24)         # wait until next frame (at 60 FPS)
            if self.limit_of_steps !=None and self.step >= self.limit_of_steps:
                self.running = False
        # Done! Time to quit.
        pygame.quit()

    def update(self):
        """
        will update all agents in the self.agents object
        """
        self.screen.fill(self.background_color)

        for a in self.agents:
            a.update(self.screen)

        for o in self.obstacles:
            self.screen.blit(o.image, o.rect)

    def add_agent(self, agent):
        self.agents.append(agent)
        self.agent_group.add(self.agents[-1])

    def create_env_from_image(self, img):
        np_img = np.array(img)
        np_img = ~np_img  # invert black and white cause (255 will be white, but we want our obstacle to be 1 and free cell be 0)
        np_img[np_img > 0] = 1

        dims = np_img.shape
        self.width = dims[1]
        self.height = dims[0]


        self.screen = pygame.display.set_mode((self.width, self.height))
        for l in range(len(np_img)) :
            for o in range(len(np_img[l])):
                if np_img[l][o] == 1:
                    self.create_wall(o,l)

    def create_wall(self, coord_x, coord_y):
        sprite = pygame.sprite.Sprite()
        sprite.image = pygame.Surface((1, 1))
        sprite.image.fill((0, 0, 0))
        sprite.rect = pygame.Rect(coord_x,coord_y, 1,1)
        self.obstacles.append(sprite)
    

class TargetPointEnvironment(Environment):
    def __init__(self, width = 100, height = 100, background_color=(200, 200, 200), caption=f'simulation_target_point', env_image = None, limit_of_steps = None, target_point:tuple[int,int]=None, amount_of_agents_goal=1):
        """"
        Environment Class represents the environment in which the agents are evolving, the user should add agents with the add_agent method before runing the env with the env one.\\
        In this Environment, the Agents has to reach a target point in order to complete the mission.
        Params : 
        - width:int (default 100) = width of the environment
        - heigth:int (default 100) = height of the environment
        - background_color:(int,int,int) = rgb color of the backgroung
        - caption:str = name given to the env
        - env_image:PIL.Image.Image = image computed into environment (overrides the witdh and height)
        - target_point:tuple:(int,int) (default : random) : target points that has to be reached by agents
        - amount_of_agents:int (default : 1) : amount of agents that needs to reach the point in order to complete the mission.
        """

        super().__init__(width, height, background_color, caption, env_image, limit_of_steps=limit_of_steps)
        self.interest_points.update({"target_points":[]})
        if target_point :
            self.init_target_point(x=target_point[0], y=target_point[1])
        else : #s'il n'y a pas de target point, on en génère un aléatoirement:
            self.init_target_point(x = randrange(0, self.width), y = randrange(0, self.height))

        self.amount_of_agent_goal = amount_of_agents_goal

    def update(self):

        super().update()
        self.screen.blit(self.target_point.image, self.target_point.rect)
        if(self.goal_condition()):
            self.running = False

    def init_target_point(self, x, y):

        sprite = Sprite()
        sprite.image = pygame.Surface((4, 4))
        sprite.image.fill((255, 0, 0))
        sprite.rect = pygame.Rect(x, y, 4, 4)
        self.target_point = sprite


        while pygame.sprite.spritecollide(self.target_point, self.obstacles_group, False):
            logging.warning("Target point overlaps with an obstacle, reallocating it randomly.")
            self.target_point.rect.center = (randrange(0, self.width), randrange(0, self.height))
        
        self.interest_points["target_points"].append((self.target_point.rect.centerx, self.target_point.rect.centery))

    def goal_condition(self):
        if len(pygame.sprite.spritecollide(self.target_point, self.agent_group, False)) >= self.amount_of_agent_goal:
            logging.info (f"Goal reached at time : {round(time.time() - self.start_time, 2)}")
            return True
        else:
            return False
