from pygame.sprite import Sprite, spritecollide, collide_circle
from pygame.draw import *
from pygame import Surface, SRCALPHA

from .Environment import Environment
from .utils import *

import numpy as np
import random
import logging
from mergedeep import merge


class Robot(Sprite):
    def __init__(self, env:Environment, robot_id:int, size, color, init_transform = (0, 0, 0), max_speed = (2,2,2), vision_range=20):#TODO : commenter
        super().__init__()

        self.env = env

        self.robot_id = robot_id

        # draw agent
        self.surf = Surface((2*size, 2*size), SRCALPHA, 32)
        circle(self.surf, color, (size, size), size)
        self.rect = self.surf.get_rect()

        # initial velocity
        self.speed = Transform2d(0,0,0)
        # max speed
        self.max_speed = Transform2d(max_speed[0], max_speed[1], max_speed[2] )

        # initial position
        self.transform = Transform2d(init_transform[0], init_transform[1], init_transform[2])

        self.vision_range = vision_range

        # inital values
        
        #TODO, remettre ces variables plus tard
        self.total_distance_made = 0.0
        # self.target = None
        # self.energy = 0

        # move agent object on coords
        self.rect.centerx = int(self.transform.x)
        self.rect.centery = int(self.transform.y)

        self.behavior_space = []

        self.belief_space = {"occupancy_grid":np.full((self.env.width, env.height), -1), "interest_points":{}}
        if self.env.full_knowledge:
            self.belief_space["occupancy_grid"] *= 0 #make all cell free
            for o in self.env.obstacles:
                self.belief_space["occupancy_grid"][o.rect.centerx][o.rect.centery] = 1 #draw obstacles in the belief space occupation grid

            self.belief_space["interest_points"].update(self.env.interest_points) #take knowledge of the interest points.
        
        #Ready!
        self.is_active = True

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
        #old positions for distance calculation
        old_tfx = self.transform.x
        old_tfy = self.transform.y


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

        self.total_distance_made += np.sqrt((self.transform.x - old_tfx)**2 + (self.transform.y - old_tfy)**2)

        # update graphics
        self.rect.centerx = int(self.transform.x)
        self.rect.centery = int(self.transform.y)

    def sense(self):
        #TODO maybe add interest points.
        #first, get neighbors in order to see the unseen ones.
        neighbors = self.get_neighbors_pixels(distance=self.vision_range, stop_at_wall=True, self_inclusion=True)
        for n in neighbors:
            self.behavior_space["occupancy_grid"][n[0]][n[1]] = self.env.real_occupation_grid[n[0]][n[1]] #get the real value (simulates sensing, note that we could add noise.)

    def get_neighbors_pixels(self, distance:int, stop_at_wall = False, self_inclusion = True):
        """
        get neighbors around the agents:\\
        Params:\\
        - distance:int : until what distance cells are considered as neighbors
        - stop at walls:bool (default False), cells behind a wall are considered as neighbors?
        - self_inclusion: bool (default:True), do we include the cell the agent is on?.
        """
        todo_queue = [(int(self.transform.x), int(self.transform.y))]
        next_queue = []
        neighbors = []

        if self_inclusion:
            neighbors.append(todo_queue[0])

        for i in range(distance):
            while len(todo_queue)>0:
                for direction in DIRECTIONS:
                    neighbor = (todo_queue[0][0] + direction[0], todo_queue[0][1] + direction[1])
                    #if it's already in neighbors, we don't want it:
                    if (neighbor in neighbors):
                        pass
                    else:
                        #if it's out of environment, we won't take it
                        if (0 > neighbor[0] or  neighbor[0] > self.env.width -1) or (0 > neighbor[1] or  neighbor[1] > self.env.width -1):
                            pass
                        else:
                            neighbors.append(neighbor) #we add the cell to neighbors
                            #if we have to stop at a wall and the cells corresponds to the one of a wall, we stop the propagation.
                            if stop_at_wall and (self.env.real_occupation_grid[neighbor[0]][neighbor[1]]==1):
                                pass
                            else:
                                next_queue.append(neighbor)
                todo_queue.pop(0)
            #then for the next distance, the next_queue becomes the todo_queue and we empty the next queue
            todo_queue = next_queue
            next_queue = []

        return neighbors
            
    def belief_transfer(self, robot_id, beliefs): #TODO, implementer le transfert des beliefs plus tard.
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
    def __init__(self, env, robot_id, size = 4, color = (0, 255, 0), init_transform = (0,0,0), max_speed = (1.0,0.0,1.5), behavior_to_use = "random"):
        super().__init__(env, robot_id, size, color, init_transform= init_transform, max_speed=max_speed)
        self.behavior_space = ["random", "target_djikstra"]

        if not( behavior_to_use in self.behavior_space) :
            logging.error(f"Ground robot:init -> behavior_to_use not in the behavior space.\n the behavior should be in {self.behavior_space}")
            exit()
        else : 
            self.behavior = behavior_to_use

    def update(self, screen):
        # self.behavior_diff_move_random()

        match self.behavior:
            case "random":
                self.behavior_diff_move_random()
            case "target_djikstra":
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
        self.speed.x = 1
        self.speed.y = 1

        if "target_points" in self.belief_space["interest_points"]:
            if not "djikstra" in self.belief_space:
                self.belief_space.update({"djikstra": djikstra(self.belief_space["occupancy_grid"], self.belief_space["interest_points"]["target_points"][0])}) #fonctionne en full knowledge de la map.
            else : 
                #determination de la cellule discrete de la belief base sur laquelle on est:
                position_actuelle = (int(self.transform.x), int(self.transform.y)) #TODO changer si on decide de varier la taille de la belief map.

                # Directions possibles (4-connectées)
                directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
                voisins = []

                for direction in directions: # voisins potentiels
                    voisin = (position_actuelle[0] + direction[0], position_actuelle[1] + direction[1])

                    # Vérifier si le voisin est valide (pas d'obstacle et a l'inerieur de l'env)
                    if 0 <= voisin[0] < self.belief_space["occupancy_grid"].shape[0] and 0 <= voisin[1] < self.belief_space["occupancy_grid"].shape[1] and self.belief_space["occupancy_grid"][voisin] == 0:
                        voisins.append(voisin)
                        
                # Trouver le voisin avec le coût le plus bas dans la distance_map
                if voisins:
                    meilleur_voisin = min(voisins, key=lambda v: self.belief_space["djikstra"][v])
                    #meilleur_voisin = (meilleur_voisin[1], meilleur_voisin [0]) #repassage des coords numpy à mes coordonnees d'environment
                    direction_vers_voisin = (meilleur_voisin[0] - position_actuelle[0], meilleur_voisin[1] - position_actuelle[1]) #TODO repasser en coordonnees continues

                    angle = np.arctan2(direction_vers_voisin[1], direction_vers_voisin[0])
                    

                    #Angle to rotate to go in the neighbor direction
                    tfw = (angle - self.transform.w)%(2*np.pi)  #differance of angle

                    #self.speed.w = min(self.max_speed.w, tfw) #TODO regérer la limite d'angle
                    self.speed.w = tfw

                    self.transform.w += self.speed.w
                    #2pi modulo
                    self.transform.w = self.transform.w%(2*np.pi)
                    #self.speed.x = direction_vers_voisin
                    
                    xmove = self.speed.x * np.cos(self.transform.w)
                    ymove = self.speed.x * np.sin(self.transform.w)

                    self.translate(xmove, ymove)


    def frontier_search_behavior(self):
        pass #TODO

class Aerial(Robot):
    def __init__(self, env, robot_id, size = 3, color = (0, 0, 255), transform=(0,0,0)):        
        super().__init__(env, robot_id, size, color, transform=transform)
        self.vmax = 2.0