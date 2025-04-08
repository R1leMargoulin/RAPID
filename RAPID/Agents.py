from pygame.sprite import Sprite, spritecollide, collide_circle
from pygame.transform import scale
from pygame.draw import *
from pygame import Surface, SRCALPHA, Rect

from .Environment import Environment
from .utils import *
from .grid_variables import *

import numpy as np
import random
import logging
# from mergedeep import merge


class Robot(Sprite):
    def __init__(self, env:Environment, robot_id:int, size, color, init_transform = (0, 0, 0), max_speed = (2,2,2), vision_range=20, communication_range = 40, communication_period = 10):#TODO rendre abstrait
        """
        Robot class are our agents representing robots.

        params:
        - env:Environment = RAPID environment the robot is in
        - robot_id:int = identifier of the robot
        - size:int = size of the robot
        - color:(int,int,int) = rgb color of the robot
        - init_transform:(float,float,float) = 2D transform of the robot:
            - x:float = x position
            - y:float = y position
            - w:float = w rotation in radian around the z axis (yaw).
        - max_speed:(float,float,float) 2d transform representation of the maximum speeds for all components.
        - vision range:int = distance a robot can sense to
        - communication_range:int =  when communication is limited, the robot can share information within robots in the communication radius.
        - communication_period:int = when agent share it's beliefs: number of steps the agent needs to wait before it can communicate again
        """
        super().__init__()
        # inital values
        self.env = env
        self.robot_id = robot_id

        # pygame agent components
        self.surf = Surface((4*size, 4*size), SRCALPHA, 32)
        circle(self.surf, color, (2*size, 2*size), 4*size)
        self.rect = Rect(0, 0, size, size)

        # Geometry
        self.speed = Transform2d(0,0,0)
        self.max_speed = Transform2d(max_speed[0], max_speed[1], max_speed[2] )
        self.transform = Transform2d(init_transform[0], init_transform[1], init_transform[2])
        self.init_transform = Transform2d(init_transform[0], init_transform[1], init_transform[2])

        #vision
        self.vision_range = vision_range

        #communication
        self.communication_mode = self.env.communication_mode
        self.communication_range = communication_range
        self.communication_period = communication_period
        self.time_from_last_communication = 0

    
        #Metrics
        self.total_distance_made = 0.0
        # self.energy = 0

        # move agent object on coords
        self.rect.centerx = int(self.transform.x)
        self.rect.centery = int(self.transform.y)


        #internal memory vars
        self.target = None
        self.path_to_target = None

        self.behavior_space = [] # To fill in the init of child classes

        #init of communication method and of the environment knowledge/beliefs
        if self.communication_mode == "blackboard":
            if "blackboard" in self.env.agents_tools:
                pass
            else:
                self.env.agents_tools["blackboard"]={} #create the BB in the env.
                self.env.agents_tools["blackboard"]["occupancy_grid"]=np.full((self.env.width, env.height), OG_UNKNOWN_CELL) #Create the occupancy grid belief in the BB
                self.env.agents_tools["blackboard"]["robot_positions"]={} #create the robot position dict belief in the BB
                if self.env.full_knowledge:
                    self.env.agents_tools["blackboard"]["occupancy_grid"] = self.env.real_occupancy_grid #make the blackboard equans to the env grid if the env is known

        elif self.communication_mode == "limited":
            #then we need to create a communication halo object for the agent
            self.communication_halo = Sprite()
            self.communication_halo.rect = Rect((self.transform.x,self.transform.y), (0,0)).inflate(self.communication_range*2, self.communication_range*2)
            halo_image = Surface(self.communication_halo.rect.size, SRCALPHA)
            circle(halo_image, (200,200,0, 85), (self.communication_range, self.communication_range), self.communication_range)

            self.communication_halo.image = halo_image

            self.connected_robots = []
        
        #creation of the belief space whatever the communication mode
        self.belief_space = {"occupancy_grid":np.full((self.env.width, env.height), OG_UNKNOWN_CELL), "interest_points":{}}
        self.belief_space["robot_positions"] = {}
        self.belief_space["robot_positions"].update({ self.robot_id:{"position":(self.transform.x, self.transform.y), "step":self.env.step} }) #we add the step in order to keep the most recent known position when merging.
        if self.env.full_knowledge:
            self.belief_space["occupancy_grid"] = self.env.real_occupancy_grid

        
        self.imdone = False #if true, the robot will consider it's mission is over, it stops its activity.
        
        #Ready!
        self.is_active = True

    def update(self, screen):
        
        if self.env.render:
            scaled_rect = Rect(self.rect.x * self.env.scaling_factor, self.rect.y * self.env.scaling_factor, self.rect.width * self.env.scaling_factor, self.rect.height * self.env.scaling_factor)
            screen.blit(scale(self.surf, scaled_rect.size), scaled_rect)

        if self.env.communication_mode == "limited":
            if self.env.render:
                halo_scaled_rect = Rect(self.communication_halo.rect.x * self.env.scaling_factor, self.communication_halo.rect.y * self.env.scaling_factor, self.communication_halo.rect.width * self.env.scaling_factor, self.communication_halo.rect.height * self.env.scaling_factor)
                screen.blit(scale(self.communication_halo.image, halo_scaled_rect.size), halo_scaled_rect)
            
    def translate(self, speed_x, speed_y):
        #old positions for distance calculation
        old_tfx = self.transform.x
        old_tfy = self.transform.y


        # update position based on delta x/y
        self.transform.x = self.transform.x + speed_x
        self.transform.y = self.transform.y + speed_y

        
        #detect and handle collisions------------------------------------------------------------------------------------
        collisions = spritecollide(self, self.env.obstacles_group, False, collide_circle)

        if (collisions): #is there collision
            sides = []
            for c in collisions:
                if self.rect.midtop[1] > c.rect.midtop[1]:
                    sides.append("top")
                if self.rect.midleft[0] > c.rect.midleft[0]:
                    sides.append("left")
                if self.rect.midright[0] < c.rect.midright[0]:
                    sides.append("right")
                if self.rect.midtop[1] < c.rect.midtop[1]:
                    sides.append("bottom")
            
            if ("top" in sides):
                self.transform.y += 2*max(np.abs(self.speed.x), np.abs(self.speed.y))
            if ("bottom" in sides):
                self.transform.y -= 2*max(np.abs(self.speed.x), np.abs(self.speed.y))
            if ("left" in sides):
                self.transform.x += 2*max(np.abs(self.speed.x), np.abs(self.speed.y))
            if ("right" in sides):
                self.transform.x -= 2*max(np.abs(self.speed.x), np.abs(self.speed.y))
        #-----------------------------------------------------------------------------------------------------------
        
        
        # ensure we stay within the screen window
        self.transform.x = max(self.transform.x, 0)
        self.transform.x = min(self.transform.x, self.env.width-1)
        self.transform.y = max(self.transform.y, 0)
        self.transform.y = min(self.transform.y, self.env.height-1)

        self.total_distance_made += np.sqrt((self.transform.x - old_tfx)**2 + (self.transform.y - old_tfy)**2)

        # update positions of pygame objects
        self.rect.centerx = int(self.transform.x)
        self.rect.centery = int(self.transform.y)


        self.belief_space["robot_positions"].update({ self.robot_id:{"position":(self.transform.x, self.transform.y), "step":self.env.step} }) #we add the step in order to keep the most recent known position when merging.

        if self.env.communication_mode == "limited":
            self.communication_halo.rect.centerx = int(self.transform.x)
            self.communication_halo.rect.centery = int(self.transform.y)

    def navigate_through_target_path(self): #TODO rendre abstraite
        pass

    def sense(self):
        #first, get neighbors in order to see the unseen ones.
        neighbors = self.get_neighbors_pixels(distance=self.vision_range, stop_at_wall=True, self_inclusion=True)

        for n in neighbors:
            self.belief_space["occupancy_grid"][n[0]][n[1]] = self.env.real_occupancy_grid[n[0]][n[1]] #get the real value (simulates sensing, note that we could add noise.)

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

        #instead of asking all cells if it's within the distance,we operate a propagation depending on the vision range.
        for i in range(distance):
            while len(todo_queue)>0:
                for direction in DIRECTIONS:
                    neighbor = (todo_queue[0][0] + direction[0], todo_queue[0][1] + direction[1])
                    #if it's already in neighbors, we don't want it:
                    if (neighbor in neighbors):
                        pass
                    else:
                        #if it's out of environment, we won't take it
                        if (0 > neighbor[0] or  neighbor[0] > self.env.width -1) or (0 > neighbor[1] or  neighbor[1] > self.env.height-1):
                            pass
                        else:
                            neighbors.append(neighbor) #we add the cell to neighbors
                            #if we have to stop at a wall and the cells corresponds to the one of a wall, we stop the propagation.
                            if stop_at_wall and (self.env.real_occupancy_grid[neighbor[0]][neighbor[1]]==1):
                                pass
                            else:
                                next_queue.append(neighbor)
                todo_queue.pop(0)
            #then for the next distance, the next_queue becomes the todo_queue and we empty the next queue
            todo_queue = next_queue
            next_queue = []

        return neighbors
            
    def belief_transfer(self): 
        """
        va transférer la table de beliefs (la grille d'occupation uniquement pour le moment) à tous les voisins de communiation.
        """
         #belief sharing handling
        if self.time_from_last_communication < self.communication_period: #verif of the communication period
            self.time_from_last_communication +=1
        else:
            if self.communication_mode == "limited":
                if self.connected_robots:
                    for robot in self.connected_robots:
                        robot.recieve_belief(self.belief_space) #envoie des beliefs à tous les robots voisins.
                    self.time_from_last_communication = 0
                else:
                    self.time_from_last_communication +=1

            elif self.communication_mode == "blackboard":
                self.env.agents_tools["blackboard"]["occupancy_grid"] = np.maximum.reduce([self.env.agents_tools["blackboard"]["occupancy_grid"], self.belief_space["occupancy_grid"]]) #Maj de la grille d'occupation
                self.env.agents_tools["blackboard"]["robot_positions"].update({self.robot_id:self.belief_space["robot_positions"][self.robot_id]}) #Maj de la position perso du robot pour le blackboard
                self.belief_space = self.env.agents_tools["blackboard"] #on tire le blackboard dans nos beliefs space une fois l'avoir mis a jour.
                self.time_from_last_communication = 0

    def recieve_belief(self, sender_belief_space):
        #dans un premiers temps, on ne partage que la grille d'occupation.
        #en supposant que le sensing de chaque agent est correct (on y mettra des probabilités plus tard, en ajoutant un layer) on peut simplement merge les deux grilles en prennant le max de chacune
        #car -1 = unknown, 0 = free, 1 = obstacle, et quand c'est plus grand c'est des points d'interets.
        self.belief_space["occupancy_grid"] = np.maximum.reduce([self.belief_space["occupancy_grid"], sender_belief_space["occupancy_grid"]])
        for robot_pos in sender_belief_space["robot_positions"]:
            if not (robot_pos in self.belief_space["robot_positions"]):
                self.belief_space["robot_positions"].update({robot_pos: sender_belief_space["robot_positions"][robot_pos]})

            elif sender_belief_space["robot_positions"][robot_pos]["step"] > self.belief_space["robot_positions"][robot_pos]["step"]:
                self.belief_space["robot_positions"].update({robot_pos: sender_belief_space["robot_positions"][robot_pos]})

            else:
                pass


class Ground(Robot):

    def __init__(self, env, robot_id, size = 1, color = (0, 255, 0), init_transform = (0,0,0), max_speed = (1.0,0.0,1.5),vision_range=20, communication_range = 40, communication_period = 10, behavior_to_use = "random"):
        super().__init__(env, robot_id, size, color, init_transform= init_transform, max_speed=max_speed, vision_range=vision_range, communication_range=communication_range, communication_period=communication_period)
        self.behavior_space = ["random", "target_djikstra", "nearest_frontier", "minpos", "local_frontier"]

        #handle behavior space string
        if not( behavior_to_use in self.behavior_space) :
            logging.error(f"Ground robot:init -> behavior_to_use not in the behavior space.\n the behavior should be in {self.behavior_space}")
            exit()
        else : 
            self.behavior = behavior_to_use

    def update(self, screen):
        # self.behavior_diff_move_random()
        if not self.imdone:
            match self.behavior:
                case "random":
                    self.behavior_diff_move_random()
                case "target_djikstra":
                    self.behavior_target_djikstra()
                case "nearest_frontier":
                    self.nearest_frontier_search_behavior()
                case "minpos":
                    self.minpos_behavior()
                case "local_frontier":
                    self.local_frontier_behavior()
        super().update(screen)

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

    def behavior_target_djikstra(self): #TODO: voir en blackboard aussi.
        self.speed.x = 1
        self.speed.y = 1

        if OG_TARGET_POINT in self.belief_space["occupancy_grid"]:
            tp_coords = np.where(self.belief_space["occupancy_grid"] == OG_TARGET_POINT)
            if not "djikstra" in self.belief_space:
                self.belief_space.update({"djikstra": djikstra(self.belief_space["occupancy_grid"],(tp_coords[0][0], tp_coords[1][0]))}) #fonctionne en full knowledge de la map.
            else : 
                #determination de la cellule discrete de la belief base sur laquelle on est:
                position_actuelle = (int(self.transform.x), int(self.transform.y)) #TODO changer si on decide de varier la taille de la belief map.

                # Directions possibles (4-connectées)
                directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
                voisins = []

                for direction in directions: # voisins potentiels
                    voisin = (position_actuelle[0] + direction[0], position_actuelle[1] + direction[1])

                    # Vérifier si le voisin est valide (pas d'obstacle et a l'inerieur de l'env)
                    if 0 <= voisin[0] < self.belief_space["occupancy_grid"].shape[0] and 0 <= voisin[1] < self.belief_space["occupancy_grid"].shape[1] and self.belief_space["occupancy_grid"][voisin] == OG_FREE_CELL:
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

    def nearest_frontier_search_behavior(self):
        """
        compute a greedy nearest frontier algorithm with an A* path search to the nearest frontier for each agent.
        """
        self.sense()#first of all sense the env.
        self.belief_transfer()

        if np.any(self.target):#si on a une target
            if self.path_to_target: #If we have a path to our target, we continue this path.
                self.navigate_through_target_path()
                pass
            else: #if we don't have any path, then compute it with A* for our target
                self.path_to_target = a_star_search(self.belief_space["occupancy_grid"], (int(self.transform.x),int(self.transform.y)), (self.target[0], self.target[1])) #from utils : A* Path calculation

        else: #sinon on va chercher les frontières.
            #frontier detection from belief space
            frontiers = find_frontier_cells(self.belief_space["occupancy_grid"]) #from utils

            if not frontiers: #si on a pas de frontieres explo finie?
                if (int(self.transform.x),int(self.transform.y)) != (int(self.init_transform.x),int(self.init_transform.y)):
                    self.target = (int(self.init_transform.x),int(self.init_transform.y))
                else:
                    self.imdone = True
            else:
                #then we take the closest one.
                distance = np.inf
                for f in frontiers:
                    hdist = heuristic_frontier_distance((self.transform.x, self.transform.y), (f[0], f[1]), self.belief_space["occupancy_grid"])
                    if hdist < distance :
                        distance = hdist
                        self.target = tuple(f.tolist()) #set the frontier as new target
            
    def minpos_behavior(self):
        """
        Adaptation from MinPos algorithm (Bautin, Simonin, Charpillet : 2012)
        Frontier based behavior where:
        - The frontiers are grouped into clusters
        - each cluster is given a cost depending on the distance and on robots that are closer to this frontier using the wavefront propagation algorithm (WPA)
        - the robot chose the frontier with the lowest cost
        """
        #first of all, sense the environment
        self.sense()#first of all sense the env.
        self.belief_transfer() #after sensing, transfer beliefs.

        if np.any(self.target):#si on a une target
            if self.path_to_target: #If we have a path to our target, we continue this path.
                self.navigate_through_target_path()
                pass
            else: #if we don't have any path, then compute it with our target
                self.path_to_target = a_star_search(self.belief_space["occupancy_grid"], (int(self.transform.x),int(self.transform.y)), (self.target[0], self.target[1])) #from utils : A* Path calculation

        else: #sinon on va chercher les frontières.
            #frontier detection
            frontiers = find_frontier_cells(self.belief_space["occupancy_grid"]) #from utils

            if list(frontiers) == None or len(list(frontiers))==0: #si on a pas de frontieres explo finie?
                if (int(self.transform.x),int(self.transform.y)) != (int(self.init_transform.x),int(self.init_transform.y)):
                    self.target = (int(self.init_transform.x),int(self.init_transform.y))
                else:
                    self.imdone = True
            else:
                cluster_centers = cluster_frontier_cells(self.belief_space["occupancy_grid"], frontiers, self.vision_range) #from utils : make cluster fontiers

                pos_list_float = [pos["position"] for pos in list(self.belief_space["robot_positions"].values())] #list of float xy position of all robots
                pos_list_int = [(int(x), int(y)) for x,y in pos_list_float] #same list with ints.

                weighted_clusters = wavefront_propagation_algorithm(self.belief_space["occupancy_grid"], (int(self.transform.x), int(self.transform.y)), pos_list_int, cluster_centers, weight_of_closer_robots=self.env.width) #the penalty for a frontier cluster depends of the size of the env.
                self.target = min(weighted_clusters, key=weighted_clusters.get) #then we take the cluster with the minimum cost

    def local_frontier_behavior(self):
        """
        adaptation from local frontier algorithm (Gauville, Charpillet : 2019)
        """
        #setup init pos if there is not.
        if not ("traces" in self.belief_space): #then init the traces in belief space
            init_pos = (int(self.init_transform.x), int(self.init_transform.y))
            self.belief_space["traces"] = {init_pos:self.env.step} #here we init the trace with a dictionarry: the key is the position the value is the timestamp (sim step)

            #init second chance used as False
            self.belief_space["second_chance_usage"] = False

        #SENSING
        self.sense()

        if np.any(self.target):#if we have a target target navigate (A*) to the target if there is one (set a trace at every update even when navigating)
            if self.path_to_target: #If we have a path to our target, we continue this path.
                self.navigate_through_target_path() #continue on the target path
                if (int(self.transform.x), int(self.transform.y)) not in self.belief_space["traces"].keys():
                    self.belief_space["traces"].update({(int(self.transform.x), int(self.transform.y)):self.env.step})#add the new current position to the traces

            else: #if we don't have any path,only a target, then compute it with A* for our target                
                self.path_to_target = a_star_search(self.belief_space["occupancy_grid"], (int(self.transform.x),int(self.transform.y)), (self.target[0], self.target[1])) #from utils : A* Path calculation
                if not self.path_to_target:
                    #self.behavior_diff_move_random()
                    self.target = None
        else:
            #LOCAL FRONTIER DETECTION -----------------------------------------------------
            vision_range = self.get_neighbors_pixels(distance=self.vision_range, stop_at_wall=True, self_inclusion=True)
            local_frontier_list = []
            for cell in vision_range:
                if self.belief_space["occupancy_grid"][cell[0]][cell[1]] == OG_WALL:
                    #if it's a wall, we skip this cell.
                    continue

                cell_neighbors = get_direct_neighbors(cell, width=self.env.width, height=self.env.height) #improvable : pour plus de realisme on pourrait mettre la taille du belief space plutot que directement l'env.

                for cn in cell_neighbors: #maximum 4 neighbors per cell
                    if self.belief_space["occupancy_grid"][cn[0]][cn[1]] == OG_UNKNOWN_CELL: #if the cell has an unknown cell as neighbor, it becomes a frontier.
                        #we add the cell to the frontier list if it is a local frontier.
                        local_frontier_list.append(cell)
                        break
                        
            #-------------------------------------------------------------------------------
            if local_frontier_list:
            #go to the most far local frontier from the traces
                max_dist_of_lf = 0
                selected_frontier = None
                mean_traces_coordinates = (int(np.mean([c[0] for c in self.belief_space["traces"].keys()])), int(np.mean([c[1] for c in self.belief_space["traces"].keys()]))) #mean coordinates of all the traces.
                for lf in local_frontier_list:
                    if euclidian_distance(lf, mean_traces_coordinates)> max_dist_of_lf: #if the distance (we take euclidian) of the LF from the robot is greater, then we select it
                        max_dist_of_lf = euclidian_distance(lf, mean_traces_coordinates)
                        selected_frontier = lf
                self.target = selected_frontier
            else: #else if there is no frontier:
                if (int(self.transform.x), int(self.transform.y)) == (int(self.init_transform.x), int(self.init_transform.y)): #if we are back at the init pose, the robot has finished.
                    if self.belief_space["second_chance_usage"] == True:
                        self.imdone = True
                        #pass #TODO : Exploration DONE
                    else:
                        #we use a second chance:
                        self.belief_space["second_chance_usage"] = True
                        
                        mean_traces_coordinates = (int(np.mean([c[0] for c in self.belief_space["traces"].keys()])), int(np.mean([c[1] for c in self.belief_space["traces"].keys()]))) #mean coordinates of all the traces.
                        max_dist = 0
                        second_chance_target = None
                        for cell in vision_range:
                            if self.belief_space["occupancy_grid"][cell[0]][cell[1]] != OG_WALL:
                                if euclidian_distance(cell, mean_traces_coordinates)> max_dist:
                                    max_dist = euclidian_distance(cell, mean_traces_coordinates)
                                    second_chance_target = cell
                        self.target = second_chance_target

                else: #else go back to the previous trace -> set it as target
                    # pour les cases voisine de distance ou le robot à pu se déplacer sur un step de simulation (sur une periode de temps donné, on récolte les voisins)
                    move_possible_neighbors =  self.get_neighbors_pixels(distance=int(max(4*self.max_speed.x, 4*self.max_speed.y)), stop_at_wall=True, self_inclusion=False)
                    chosen_trace = None
                    oldest_timestep = np.inf
                    for cell in move_possible_neighbors : #on va prendre la trace la plus ancienne possible dans ce champs
                        if cell in self.belief_space["traces"]: #check if the cell is registered in the traces or we would have an error
                            if self.belief_space["traces"][cell] < oldest_timestep:
                                chosen_trace = cell
                                oldest_timestep = self.belief_space["traces"][cell]
                    self.target = chosen_trace #on definit la trace la plus ancienne dans le rayon restreint défini.

        self.belief_transfer() #belief transfer management.
    
    def navigate_through_target_path(self):
        def make_the_move(waypoint):
            direction = (waypoint[0] - int(self.transform.x), waypoint[1] - int(self.transform.y)) #TODO repasser en coordonnees continues

            angle = np.arctan2(direction[1], direction[0])
            
            #Angle to rotate to go in the neighbor direction
            tfw = (angle - self.transform.w)%(2*np.pi)  #differance of angle

            #self.speed.w = min(self.max_speed.w, tfw) #TODO regérer la limite d'angle
            self.speed.w = tfw

            self.transform.w += self.speed.w
            #2pi modulo
            self.transform.w = self.transform.w%(2*np.pi)
            #self.speed.x = direction_vers_voisin

            self.speed.x = min(euclidian_distance((0,0), direction), self.max_speed.x)
            
            xmove = self.speed.x * np.cos(self.transform.w)
            ymove = self.speed.x * np.sin(self.transform.w)

            self.translate(xmove, ymove)

        #we should be nearby the first point of the path, else we delete it and we'll compute an other one:
        if euclidian_distance((int(self.transform.x), int(self.transform.y)), (self.path_to_target[0][0], self.path_to_target[0][1])) <= 5: #if we are more than 5 away from the path, we forget the target it in order to recalculate a new one
            if self.path_to_target[0] == self.target:
                waypoint = self.path_to_target[0]
                make_the_move(waypoint)
                
                self.target = None
                self.path_to_target = None
            else:
                self.path_to_target.pop(0)
                waypoint = self.path_to_target[0]

                make_the_move(waypoint)

                pass
        else:
            #Path not accurate.
            self.behavior_diff_move_random() #random move to maybe select another frontier.
            self.path_to_target = None
            self.target = None
class Aerial(Robot): #TODO : IMPLEMETER
    def __init__(self, env, robot_id, size = 3, color = (0, 0, 255), transform=(0,0,0)):        
        super().__init__(env, robot_id, size, color, transform=transform)
        self.vmax = 2.0