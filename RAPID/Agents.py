from pygame.sprite import Sprite, spritecollide, collide_circle
from pygame.draw import *
from pygame import Surface, SRCALPHA

from .Environment import Environment
from .utils import *

import numpy as np
import random

class Robot(Sprite):
    def __init__(self, env:Environment, size, color, transform = (0, 0, 0), max_speed = (2,2,2)):
        super().__init__()

        self.env = env

        # draw agent
        self.surf = Surface((2*size, 2*size), SRCALPHA, 32)
        circle(self.surf, color, (size, size), size)
        self.rect = self.surf.get_rect()

        # default values
        self.speed = 0

        self.max_speed = Transform2d(max_speed[0], max_speed[1], max_speed[2] )

        # initial position

        self.transform = Transform2d(transform[0], transform[1], transform[2])

        # initial velocity
        self.speed = Transform2d(0,0,0)

        # inital values
        self.is_active = True
        self.target = None
        self.energy = 0

        # move agent object on coords
        self.rect.centerx = int(self.transform.x)
        self.rect.centery = int(self.transform.y)



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
        self.transform.x = self.transform.x + speed_x
        self.transform.y = self.transform.y + speed_y

        
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
                self.transform.y += 1
            if ("bottom" in sides):
                self.transform.y -= 1
            if ("left" in sides):
                self.transform.x -= 1
            if ("right" in sides):
                self.transform.x += 1
        #-----------------------------------------------------------------------------------------------------------
        
        
        # ensure it stays within the screen window
        self.transform.x = max(self.transform.x, 0)
        self.transform.x = min(self.transform.x, self.env.screen.get_width())
        self.transform.y = max(self.transform.y, 0)
        self.transform.y = min(self.transform.y, self.env.screen.get_height())

        # update graphics
        self.rect.centerx = int(self.transform.x)
        self.rect.centery = int(self.transform.y)


class Ground(Robot):
    def __init__(self, env, size = 4, color = (0, 255, 0), transform = (0,0,0), max_speed = (1.0,0.0,0.5)):
        color = (0, 255, 0)
        super().__init__(env, size, color, transform=transform, max_speed=max_speed)

        self.speed.x = self.max_speed.x
        self.speed.y = self.max_speed.y
        self.speed.w = self.max_speed.w


    def update(self, screen):
        self.diff_move_random()
        screen.blit(self.surf, self.rect)


    def diff_move_random(self):
        #random rotation
        self.speed.w = random.uniform(-self.max_speed.w ,self.max_speed.w)
        self.transform.w += self.speed.w

        #2pi modulo
        self.transform.w = self.transform.w%(2*np.pi)
        
        xmove = self.speed.x * np.cos(self.transform.w)
        ymove = self.speed.x * np.sin(self.transform.w)

        self.translate(xmove, ymove)





class Aerial(Robot):
    def __init__(self, env, size = 3, color = (0, 0, 255), x=0, y=0):        
        super().__init__(env, size, color)
        self.vmax = 2.0