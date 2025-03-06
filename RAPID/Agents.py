from pygame.sprite import Sprite, spritecollide, collide_circle
from pygame.draw import *
from pygame import Surface, SRCALPHA

from .Environment import Environment
from .utils import *

import numpy as np
import random
from mergedeep import merge


class Robot(Sprite):
    def __init__(self, env:Environment, robot_id:int, size, color, transform = (0, 0, 0), max_speed = (2,2,2)):
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
        #TODO, remettre ces variables plus tard
        # self.target = None
        # self.energy = 0

        # move agent object on coords
        self.rect.centerx = int(self.transform.x)
        self.rect.centery = int(self.transform.y)


        self.belief_space = {"occupancy_grid":np.full((self.env.width, env.height), -1), "interest_points":{}}
        if self.env.full_knowledge:
            self.belief_space["occupancy_grid"] *= 0 #make all cell free
            for o in self.env.obstacles:
                self.belief_space["occupancy_grid"][o.rect.centerx][o.rect.centery] = 1 #draw obstacles in the belief space occupation grid

            self.belief_space["interest_points"].update(self.env.interest_points) #take knowledge of the interest points.






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

    def belief_link(self, robot_id, beliefs):
        """
        Belief transmission from one agent to another. 
        Return : 
        - [False, None] If the beliefs space are the same for the two agents : beliefs are the same.
        - [True, New belief_space] If the beliefs spaces are not the same, a merge is done on the two beliefs.
        """
        if beliefs != self.belief_link:
            merge(self.belief_space, beliefs)
            return True, self.belief_space
        else :
            return False, None

class Ground(Robot):
    def __init__(self, env, robot_id, size = 4, color = (0, 255, 0), transform = (0,0,0), max_speed = (1.0,0.0,1.5)):
        super().__init__(env, robot_id, size, color, transform=transform, max_speed=max_speed)



    def update(self, screen):
        # self.behavior_diff_move_random()
        self.behavior_target_djikstra()
        screen.blit(self.surf, self.rect)


    def behavior_diff_move_random(self):
        #set srobot speed at it's max speed
        self.speed.x = self.max_speed.x
        self.speed.y = self.max_speed.y

        #random rotation
        self.speed.w = random.uniform(-self.max_speed.w ,self.max_speed.w)
        self.transform.w += self.speed.w

        #2pi modulo
        self.transform.w = self.transform.w%(2*np.pi)
        #TODO refaire ce diff drive dégueu
        #calculation of the x and y movement depending of the x direction speed and the w orientation.
        xmove = self.speed.x * np.cos(self.transform.w)
        ymove = self.speed.x * np.sin(self.transform.w)

        self.translate(xmove, ymove)

    def behavior_target_djikstra(self): #TODO: reparer
        self.speed.x = 2
        self.speed.y = 2

        if "target_points" in self.belief_space["interest_points"]:
            if not "djikstra" in self.belief_space:
                self.belief_space.update({"djikstra": djikstra(self.belief_space["occupancy_grid"], self.belief_space["interest_points"]["target_points"][0])}) #fonctionne en full knowledge de la map.
            else : 
                #determination de la cellule discrete de la belief base sur laquelle on est:
                position_actuelle = (int(self.transform.x), int(self.transform.y)) #TODO changer si on decide de varier la taille de la belief map.
                print(f"pos reelle : {self.transform.x, self.transform.y}")
                print(f"pos sur la grille : {position_actuelle[0], position_actuelle[1]}")

                # Directions possibles (4-connectées)
                directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
                voisins = []

                for direction in directions: # voisins potentiels
                    voisin = (position_actuelle[0] + direction[0], position_actuelle[1] + direction[1])

                    # Vérifier si le voisin est valide (pas d'obstacle et a l'inerieur de l'env)
                    if 0 <= voisin[0] < self.belief_space["occupancy_grid"].shape[0] and 0 <= voisin[1] < self.belief_space["occupancy_grid"].shape[1] and self.belief_space["occupancy_grid"][voisin] == 0:
                        voisins.append(voisin)
                        print(f"voisin :{voisin}")
                        print(f"value{self.belief_space["djikstra"][voisin]}")
                        
                # Trouver le voisin avec le coût le plus bas dans la distance_map
                if voisins:
                    meilleur_voisin = min(voisins, key=lambda v: self.belief_space["djikstra"][v])
                    #meilleur_voisin = (meilleur_voisin[1], meilleur_voisin [0]) #repassage des coords numpy à mes coordonnees d'environment
                    direction_vers_voisin = (meilleur_voisin[0] - position_actuelle[0], meilleur_voisin[1] - position_actuelle[1]) #TODO repasser en coordonnees continues
                    print(f"direction vector :{direction_vers_voisin}")
                    angle = np.arctan2(direction_vers_voisin[1], direction_vers_voisin[0])
                    
                    print(f"meilleur voisin :{meilleur_voisin}")
                    print(f"angle :{angle}")
                    #Angle to rotate to go in the neighbor direction
                    tfw = (angle - self.transform.w)%(2*np.pi)  #differance of angle
                    print(f"self.transform.w :{self.transform.w}")
                    print(f"tfw :{tfw}")
                    self.speed.w = min(self.max_speed.w, tfw)

                    self.transform.w += self.speed.w
                    #2pi modulo
                    self.transform.w = self.transform.w%(2*np.pi)
                    #self.speed.x = direction_vers_voisin
                    
                    xmove = self.speed.x * np.cos(self.transform.w)
                    ymove = self.speed.x * np.sin(self.transform.w)
                    print(xmove, ymove)
                    self.translate(xmove, ymove)



                

        pass #TODO

    def frontier_search_behavior(self):
        pass #TODO

class Aerial(Robot):
    def __init__(self, env, robot_id, size = 3, color = (0, 0, 255), transform=(0,0,0)):        
        super().__init__(env, robot_id, size, color, transform=transform)
        self.vmax = 2.0