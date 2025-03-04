import pygame
from pygame.locals import (K_ESCAPE, KEYDOWN)

import numpy as np
from PIL import Image
from random import uniform, randrange

class Environment():
    def __init__(self, width:int=100, height:int=100, background_color = (11,11,11), caption = f'simulation', env_image:Image.Image = None):
        #env init
        pygame.init()
        self.clock = pygame.time.Clock()

        self.background_color = background_color
        
        

        self.agents = []
        self.obstacles = []
        

        if(env_image):
            self.create_env_from_image(env_image)
        else:
            self.screen = pygame.display.set_mode((width, height))



        self.obstacles_group = pygame.sprite.Group(self.obstacles)


        pygame.display.set_caption(caption)
        self.running = True

        pass

    def run(self):
        while self.running:
            # check user input events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                if event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self.running = False
            #will update all agents in the self.agents object
            self.update()
            # draw all changes to the screen
            pygame.display.flip()
            self.clock.tick(24)         # wait until next frame (at 60 FPS)

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

    def create_env_from_image(self, img):
        np_img = np.array(img)
        np_img = ~np_img  # invert black and white cause (255 will be white, but we want our obstacle to be 1 and free cell be 0)
        np_img[np_img > 0] = 1

        dims = np_img.shape
        self.screen = pygame.display.set_mode((dims[0], dims[1]))

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
    def __init__(self, width = 100, height = 100, background_color=(11, 11, 11), caption=f'simulation', env_image = None, target_point:tuple[int,int]=None):
        super().__init__(width, height, background_color, caption, env_image)
        if target_point :
            self.target_point = target_point
        else : #s'il n'y a pas de target point, on en génère un aléatoirement:
            self.target_point = (randrange(0, width), randrange(0,height)) #TODO, verifier que le target soit pas dans un mur