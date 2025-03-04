from pygame.sprite import Sprite, spritecollide, collide_circle
from pygame.draw import *
from pygame import Surface, SRCALPHA

from .Environment import Environment

import numpy as np
import random

class Robot(Sprite):
    def __init__(self, env:Environment, size, color, x=0, y=0, w=0):
        super().__init__()

        self.env = env

        # draw agent
        self.surf = Surface((2*size, 2*size), SRCALPHA, 32)
        circle(self.surf, color, (size, size), size)
        self.rect = self.surf.get_rect()

        # default values
        self.speed = 0
        self.max_speed_x = 2.0
        self.max_speed_y = 2.0
        self.max_speed_w = 2.0

        # initial position
        self.x = x
        self.y = y
        self.w = w

        # initial velocity
        self.speed_x = 0
        self.speed_y = 0
        self.speed_w = 0

        # inital values
        self.is_active = True
        self.target = None
        self.energy = 0

        # move agent object on coords
        self.rect.centerx = int(self.x)
        self.rect.centery = int(self.y)



    def update(self, screen):

        # we can't move, just update the screen
        if self.vmax == 0:
            screen.blit(self.surf, self.rect)
            return

        # eat the target if close enough
        if self.target:
            #squared_dist = (self.x - self.target.x) ** 2 + (self.y - self.target.y) ** 2
            pass

        self.random_move_behavior()
        
        screen.blit(self.surf, self.rect)

    def random_move_behavior(self):
        randx = random.uniform(-2.0,2.0)
        randy = random.uniform(-2.0,2.0)
        self.translate( randx, randy)

    def translate(self, speed_x, speed_y):
        # slow down agent if it moves faster than it max velocity


        # update position based on delta x/y
        self.x = self.x + speed_x
        self.y = self.y + speed_y

        
        #detect collisions-------------------------------------------------------------------------------------------
        collisions = spritecollide(self, self.env.obstacles_group, False, collide_circle)

        if (collisions): #is there collision
            sides = []
            for c in collisions:
                if self.rect.midtop[1] > c.rect.midtop[1]:
                    sides.append("top")
                elif self.rect.midleft[0] > c.rect.midleft[0]:
                    sides.append("left")
                elif self.rect.midright[0] < c.rect.midright[0]:
                    sides.append("right")
                else:
                    sides.append("bottom")
            
            if ("top" in sides):
                self.y += 1
            if ("bottom" in sides):
                self.y -= 1
            if ("left" in sides):
                self.x -= 1
            if ("right" in sides):
                self.x += 1
        #-----------------------------------------------------------------------------------------------------------
        
        
        # ensure it stays within the screen window
        self.x = max(self.x, 0)
        self.x = min(self.x, self.env.screen.get_width())
        self.y = max(self.y, 0)
        self.y = min(self.y, self.env.screen.get_height())

        # update graphics
        self.rect.centerx = int(self.x)
        self.rect.centery = int(self.y)


class Ground(Robot):
    def __init__(self, env, x=0, y=0, w=0):
        size = 4
        color = (0, 255, 0)
        super().__init__(env, size, color, x, y, w)
        self.max_speed_x = 1
        self.max_speed_y = 0.0
        self.max_speed_w = 0.5 #en radians

        self.speed_x = self.max_speed_x
        self.speed_y = self.max_speed_y
        self.speed_w = self.max_speed_w


    def update(self, screen):
        self.diff_move_random()
        screen.blit(self.surf, self.rect)


    def diff_move_random(self):
        #random rotation
        rotation = random.uniform(-self.speed_w,self.speed_w)
        self.w += rotation

        #2pi modulo
        self.w = self.w%(2*np.pi)
        
        xmove = self.speed_x * np.cos(self.w)
        ymove = self.speed_x * np.sin(self.w)

        self.translate(xmove, ymove)





class Aerial(Robot):
    def __init__(self, env, x=0, y=0):
        size = 3
        color = (0, 0, 255)
        super().__init__(env, size, color)
        self.vmax = 2.0