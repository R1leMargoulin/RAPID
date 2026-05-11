from pygame.sprite import Sprite, spritecollide, collide_circle
from pygame.transform import scale
from pygame.draw import *
from pygame import Surface, SRCALPHA, Rect

from .Environment import Environment
from .Graph import Graph, Node
from .Artifacts import Artifact
from .utils import *
from .grid_variables import *



import numpy as np
import random
from munkres import Munkres
from sklearn.cluster import KMeans
import logging
from copy import deepcopy



class Robot(Sprite):
    def __init__(self, env:Environment, robot_id:int, size, color, init_transform = (0, 0, 0), max_speed = (2,2,2), vision_range=20, communication_range = 40, communication_period = 10, energy_amount = 1000, energy_cost_per_cell = 1, delta_replan=20, write_logs=False, graph_mode=False, graph_delta=50):
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
        - energy_amount: int = energy unit available for the robot, the robot will stops if it reaches zero.
        - energy_cost_per_cell: int = energy consumption
        - delta_replan : int = time (in sim steps) from a planning until the robot will replan a new goal in case it recieve new knowledge from an other team member.
        - write_logs : bool = if true, will save log in RAM for simulation stats.
        - graph_mode : Boolean = true if robots should use graphs over occupancy grid. Occupancy grid will be used locally for naviation purposes only, but won't be communicate. if one robot is using graph_mode, ALL ROBOTS SHOULD DO AS WELL.
        """
        self.status = "init"
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
        self.new_communication = False

        #replan
        self.delta_replan = delta_replan
        self.last_plan_time = 0

        #energy
        self.energy_max_amount = energy_amount
        self.energy_amount = energy_amount
        self.energy_cost_per_cell = energy_cost_per_cell

        #ease in the env, will be as a classical ground robot by default:
        self.env_ease = {
            OG_FREE_CELL_GROUP_NAME:1,
            OG_WALL_GROUP_NAME:0,
            OG_HIGH_WALL_GROUP_NAME:0,
            OG_SAND_GROUP_NAME:0.4,
            OG_WATER_GROUP_NAME:0,
            OG_GRASS_GROUP_NAME:0.6
        }
        self.traversable_types = list(filter(lambda k: self.env_ease[k] != 0, self.env_ease)) #find the cells that the robot can eventually traverse
        for i in range(len(self.traversable_types)):#we have the string name of the cells types, lets get the int values
            self.traversable_types[i] = ENV_CELL_TYPES[self.traversable_types[i]]

        self.competences = {"exploration":{"capability": 1, "importance":1, "distance_treshold":0, "dispersion":1},
                            "communication":{"capability": 1, "importance":1, "distance_treshold":self.communication_range, "dispersion":0}} #to add depending of the case and the robot
    
        #Metrics
        self.total_distance_made = 0.0
        # self.energy = 0

        # move agent object on coords
        self.rect.centerx = int(self.transform.x)
        self.rect.centery = int(self.transform.y)


        #internal memory vars
        self.target = None
        self.treshold_for_target = 1
        self.path_to_target = None
        self.action_to_perform=None

        self.behavior_space = [] # To fill in the init of child classes

        #init of communication method and of the environment knowledge/beliefs
        if self.communication_mode == "blackboard":
            if "blackboard" in self.env.agents_tools:
                pass
            else:
                self.env.agents_tools["blackboard"]={} #create the BB in the env.
                self.env.agents_tools["blackboard"]["occupancy_grid"]=np.full((self.env.width, env.height), OG_UNKNOWN_CELL) #Create the occupancy grid belief in the BB
                self.env.agents_tools["blackboard"]["robot_informations"]={} #create the robot position dict belief in the BB
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
        self.belief_space = {"self_id": self.robot_id, "occupancy_grid":np.full((self.env.width, env.height), OG_UNKNOWN_CELL), "artifacts":{}, "robot_informations":{}, "last_infos_matrix": {self.robot_id:{self.robot_id:0}}}
        self.belief_space["robot_informations"].update({ self.robot_id:{"position":(self.transform.x, self.transform.y), "step":self.env.step, "competences":self.competences, "env_ease":self.env_ease, "traversable_types":self.traversable_types} }) #we add the step in order to keep the most recent known position when merging.
        if self.env.full_knowledge:
            self.belief_space["occupancy_grid"] = self.env.real_occupancy_grid

        self.graph_mode = graph_mode
        self.graph_delta = graph_delta
        self.last_graph_generation = self.env.step
        self.graph = None
        if graph_mode:
            self.graph_mode = True
            self.belief_space["graph"] = Graph(self.belief_space["occupancy_grid"], agent_id = self.robot_id, traversable_types=self.traversable_types+[-1], nodes_distance_treshold=self.vision_range/2)
            #self.graph = Graph(self.belief_space["occupancy_grid"], agent_id = self.robot_id, traversable_types=self.traversable_types+[-1]) #Graph generation
            #self.graph.plot_voronoi_graph(img=self.env.env_image, display_zones=True) #virer ca


        self.last_given_position = (int(self.transform.x), int(self.transform.y))
        
        self.imdone = False #if true, the robot will consider it's mission is over, it stops its activity.

        self.logging = write_logs
        self.logs = {}

        self.com_importance_mode = "default" #ComImportance
        
        #Ready!
        self.status = "ready"
        self.is_active = True

    def update(self):
        """
        Update function of the agent, will be called at each simulation step.
        """
        self.sense()#first of all sense the env.

        #TODO: Graphs : gestion graphs si necessaire
        if self.graph_mode:
            if self.env.step - self.last_graph_generation > self.graph_delta:
                self.belief_space["graph"].update_graph(self.belief_space["occupancy_grid"], agent_id=self.robot_id, traversable_types=self.traversable_types)
                self.last_graph_generation = self.env.step
                
                self.belief_space["graph"].plot_voronoi_graph(img=self.env.env_image, display_zones=True)#TODO : virer ca
        #TODO : Graphs : modifier dans les prises de decision en fonction

        #TODO : Graphs : modif le belief transfer en fonction de si on fait des graphs ou pas.
        self.belief_transfer() #after sensing, transfer beliefs if applicable
        if np.any(self.target):
            self.navigate()
        elif self.action_to_perform != None:
            self.perform_target_action()
        else:
            self.behave() #in order to determine what to do.
            if not self.imdone and np.any(self.target):
                self.path_to_target = a_star_search(self.belief_space["occupancy_grid"], (int(self.transform.x),int(self.transform.y)), (self.target[0], self.target[1]), traversable_types=self.traversable_types) #from utils : A* Path calculation
                if self.path_to_target != None:
                    self.navigate_through_target_path() 


        if not(self.imdone):
            #print(f"robot {self.robot_id}: status {self.status}, target {self.target}")

            self.belief_space["robot_informations"][self.robot_id].update({ "position":(self.transform.x, self.transform.y), "competences":self.competences, "env_ease":self.env_ease, "traversable_types":self.traversable_types, "status": self.status, "step":self.env.step }) #self beliefs update
            self.belief_space["last_infos_matrix"][self.robot_id][self.robot_id] = self.env.step
                
           
            if (self.energy_amount / self.energy_max_amount) <= 0:
                self.finish()

            if self.status == "destroyed":
                self.finish()
            
            if self.new_communication and self.env.step - self.last_plan_time > self.delta_replan:
                #print(f"robot {self.robot_id} : replan, step {self.env.step}, last com {self.time_from_last_communication}, last plan {self.last_plan_time}")
                self.target = None
                self.path_to_target = None
                self.action_to_perform = None
                self.new_communication = False
                self.last_plan_time = self.env.step
            
            if self.logging:
                self.write_logs()
            
            #print(self.robot_id, self.action_to_perform, self.target, (int(self.transform.x), int(self.transform.y)))

    def render(self, screen):
        """
        Display rendering for the simulation
        """
        if self.env.communication_mode == "limited":
            halo_scaled_rect = Rect(self.communication_halo.rect.x * self.env.scaling_factor, self.communication_halo.rect.y * self.env.scaling_factor, self.communication_halo.rect.width * self.env.scaling_factor, self.communication_halo.rect.height * self.env.scaling_factor)
            screen.blit(scale(self.communication_halo.image, halo_scaled_rect.size), halo_scaled_rect)

        scaled_rect = Rect(self.rect.x * self.env.scaling_factor, self.rect.y * self.env.scaling_factor, self.rect.width * self.env.scaling_factor, self.rect.height * self.env.scaling_factor)
        screen.blit(scale(self.surf, scaled_rect.size), scaled_rect)

    def behave(self):
        """Has to be overloaded in other robots types, in order to implement the behaviors handlable by the robot"""
        raise Exception(f"The behave method has to be redefined for the agent {self.robot_id} of type {self.type} ")

    def navigate(self):
        if self.path_to_target: #If we have a path to our target, we continue this path.
            self.navigate_through_target_path()
            pass
        else: #if we don't have any path, then compute it with our target
            #print("target", self.target)
            self.path_to_target = a_star_search(self.belief_space["occupancy_grid"], (int(self.transform.x),int(self.transform.y)), (self.target[0], self.target[1]), traversable_types=self.traversable_types) #from utils : A* Path calculation
            if not(self.path_to_target):
                self.target = None
                self.action_to_perform = None

    def finish(self):
        """called by behavior when the work is considered done."""
        self.imdone = True
        self.rect.centerx = -1 #tp hors de la map pour les collisions
        self.rect.centery = -1

    def translate(self, speed_x, speed_y):
        """
        CAREFUL, called by the method "move" only. every robot movement has to be called by the "move" method.
        Will apply the "move" computation results in the actual simulation.
        """

        #old positions for distance calculation
        old_tfx = self.transform.x
        old_tfy = self.transform.y


        energy_consumption =self.energy_cost_per_cell * np.sqrt((speed_x)**2+(speed_y)**2)


        current_cell_type  = self.env.real_occupancy_grid[int(self.transform.x)][int(self.transform.y)]# get the current cell type in order to adapt the speed depending of the traversability ease of the robot
        current_cell_type_name = list(ENV_CELL_TYPES.keys())[list(ENV_CELL_TYPES.values()).index(int(current_cell_type))]
        movement_ease = self.env_ease[current_cell_type_name]

        #noise to mvt
        noise = round(random.uniform(0.0, 0.05),5)

        # update position based on delta x/y and the movement ease depending of the type of the cell we are on
        self.transform.x = self.transform.x + speed_x * movement_ease * (1-noise)
        self.transform.y = self.transform.y + speed_y * movement_ease * (1-noise)

        
        #detect and handle collisions------------------------------------------------------------------------------------
        #OBSTACLE COLLISION
        for cell_type in filter(lambda k: self.env_ease[k] == 0, self.env_ease): #for all cells type with a traversability ease of 0 (obstacles)
            if cell_type in self.env.cell_feature_groups:
                #collisions = spritecollide(self, self.env.cell_feature_groups[cell_type], False)
                collisions = not(int(self.env.real_occupancy_grid[int(self.transform.x)][int(self.transform.y)]) in self.traversable_types)
                if (collisions): #is there collision
                    sides = []
                    self.transform.x = old_tfx
                    self.transform.y = old_tfy

        #-----------------------------------------------------------------------------------------------------------
        
        
        # ensure we stay within the screen window
        self.transform.x = max(self.transform.x, 0)
        self.transform.x = min(self.transform.x, self.env.width-1)
        self.transform.y = max(self.transform.y, 0)
        self.transform.y = min(self.transform.y, self.env.height-1)

        distance_made = np.sqrt((self.transform.x - old_tfx)**2 + (self.transform.y - old_tfy)**2)

        #energy transition
        self.energy_amount -= energy_consumption

        self.total_distance_made += distance_made

        # update positions of pygame objects
        self.rect.centerx = int(self.transform.x)
        self.rect.centery = int(self.transform.y)

        #AGENTS COLLISION detection : 
        agent_collision = spritecollide(self, self.env.agent_group, False)
        #print(agent_collision)
        if (len(agent_collision)> 1) :
            self.transform.x = old_tfx
            self.transform.y = old_tfy
            self.rect.centerx = int(self.transform.x)
            self.rect.centery = int(self.transform.y)



        if self.env.communication_mode == "limited":
            self.communication_halo.rect.centerx = int(self.transform.x)
            self.communication_halo.rect.centery = int(self.transform.y)

    def sense(self):
        """
        Get the cells around the robot in order to update local occupancy grid
        """
        #first, get neighbors in order to see the unseen ones.
        neighbors = self.get_neighbors_pixels(distance=self.vision_range, stop_at_wall=True, self_inclusion=True)

        for n in neighbors:
            self.belief_space["occupancy_grid"][n[0]][n[1]] = self.env.real_occupancy_grid[n[0]][n[1]] #get the real value (simulates sensing, note that we could add noise.)

        #artifact detection
        for a in self.env.interest_points["artifacts"]:
            if a.coordinates in neighbors:
                if a.id in self.belief_space["artifacts"]:
                    discovery_time = self.belief_space["artifacts"][a.id]["discovery_time"] #keeping the discov time
                    awared = self.belief_space["artifacts"][a.id]["awared"] #keeping awared
                    self.belief_space["artifacts"].update({ a.id:{"name":a.name, "type":a.type, "status":a.status, "coordinates":a.coordinates, "step":self.env.step, "needed_robots":a.needed_robots, "discovery_time": discovery_time, "awared":awared}}) 
                else:
                    self.belief_space["artifacts"].update({ a.id:{"name":a.name, "type":a.type, "status":a.status, "coordinates":a.coordinates, "step":self.env.step, "needed_robots":a.needed_robots, "discovery_time": self.env.step, "awared": [self.robot_id]}}) 

    def get_neighbors_pixels(self, distance:int, stop_at_wall = False, self_inclusion = True):
        """
        get neighbors around the agents:\\
        Params:\\
        - distance:int : until what distance cells are considered as neighbors
        - stop at walls:bool (default False), cells behind a wall are considered as neighbors?
        - self_inclusion: bool (default:True), do we include the cell the agent is on?.
        """
        now_queue = [(int(self.transform.x), int(self.transform.y))]
        next_queue = []
        neighbors = []

        if self_inclusion:
            neighbors.append(now_queue[0])

        #instead of asking all cells if it's within the distance,we operate a propagation depending on the vision range.
        for i in range(distance):
            while len(now_queue)>0:
                for direction in DIRECTIONS:
                    neighbor = (now_queue[0][0] + direction[0], now_queue[0][1] + direction[1])
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
                            if stop_at_wall and not(self.env.real_occupancy_grid[neighbor[0]][neighbor[1]] in self.traversable_types) and self.env.real_occupancy_grid[neighbor[0]][neighbor[1]] in BLOCKING_SENSOR_TYPES:
                                pass
                            else:
                                next_queue.append(neighbor)
                now_queue.pop(0)
            #then for the next distance, the next_queue becomes the now_queue and we empty the next queue
            now_queue = next_queue
            next_queue = []

        return neighbors
            
    def belief_transfer(self): 
        """
        will transfer belief space to all communication neighbors.
        """
         #belief sharing handling
        if self.time_from_last_communication < self.communication_period: #verif of the communication period
            self.time_from_last_communication +=1
        else:
            if self.communication_mode == "limited":
                if self.connected_robots:
                    for robot in self.connected_robots:
                        if self.robot_id != robot.robot_id:
                            BS_copy = deepcopy(self.belief_space)
                            robot.recieve_belief(BS_copy) #envoie des beliefs à tous les robots voisins, faut ptet faire une copie...
                            self.time_from_last_communication = 0
                    self.last_given_position = (int(self.transform.x), int(self.transform.y))
                else:
                    self.time_from_last_communication +=1

            elif self.communication_mode == "blackboard":
                self.env.agents_tools["blackboard"]["occupancy_grid"] = np.maximum.reduce([self.env.agents_tools["blackboard"]["occupancy_grid"], self.belief_space["occupancy_grid"]]) #Maj de la grille d'occupation
                self.env.agents_tools["blackboard"]["robot_informations"].update({self.robot_id:self.belief_space["robot_informations"][self.robot_id]}) #Maj des infos perso du robot pour le blackboard
                self.belief_space = self.env.agents_tools["blackboard"] #on tire le blackboard dans nos beliefs space une fois l'avoir mis a jour.
                self.time_from_last_communication = 0
                self.last_given_position = (int(self.transform.x), int(self.transform.y))

    def recieve_belief(self, sender_belief_space):
        """
        Method called from an other robots (in the b"belief_transfer" method) which is communication neighbor in order to share it's knowledge.
        
        :param sender_belief_space: Belief space of the sender
        """

        #dans un premiers temps, on ne partage que la grille d'occupation.
        #en supposant que le sensing de chaque agent est correct (on y mettra des probabilités plus tard, en ajoutant un layer) on peut simplement merge les deux grilles en prennant le max de chacune
        #car -1 = unknown, 0 = free, 1 = obstacle, et quand c'est plus grand c'est des points d'interets.

        #ROBOTS INFOS-----------------------------------------------------------
        if not self.graph_mode:
            self.belief_space["occupancy_grid"] = np.maximum.reduce([self.belief_space["occupancy_grid"], sender_belief_space["occupancy_grid"]])
        else:
            self.belief_space["graph"].merge_graph(sender_belief_space["graph"]) #TODO : Graph merging has to be improved, currently not working


        for robot_infos in sender_belief_space["robot_informations"]: #robot positions update based on the newest timestamp
            if not (robot_infos in self.belief_space["robot_informations"]):
                self.belief_space["robot_informations"].update({robot_infos: sender_belief_space["robot_informations"][robot_infos]})

                #update in infos matrix as well
                self.belief_space["last_infos_matrix"][self.robot_id].update({robot_infos : self.belief_space["robot_informations"][robot_infos]["step"]}) #test


            elif sender_belief_space["robot_informations"][robot_infos]["step"] > self.belief_space["robot_informations"][robot_infos]["step"]:
                self.belief_space["robot_informations"].update({robot_infos: sender_belief_space["robot_informations"][robot_infos]})

                #update in infos matrix as well
                self.belief_space["last_infos_matrix"][self.robot_id].update({robot_infos : self.belief_space["robot_informations"][robot_infos]["step"]}) #test
        #ROBOTS INFOS-----------------------------------------------------------



        # LAST COM MATRIX----------------------------------------------------
        for agent in set(list(self.belief_space["last_infos_matrix"].keys()) + list(sender_belief_space["last_infos_matrix"].keys())): #boucle de verif s'il y a les memes agents
            if not(agent in  self.belief_space["last_infos_matrix"]):
                newrobot = {agent:0}
                for r in self.belief_space["last_infos_matrix"]:
                    newrobot.update({r:0}) #on considere qu'ils n'ont jamais communique, donc on met la step 0 par defaut avec tous les agents
                    self.belief_space["last_infos_matrix"][r].update({agent:0}) #pour la symetrie
                self.belief_space["last_infos_matrix"].update({agent : newrobot})
            
        # maj des coms du sender et reciever dans la matrice
        self.belief_space["last_infos_matrix"][sender_belief_space["self_id"]].update({self.robot_id : self.belief_space["robot_informations"][agent]["step"]})
        self.belief_space["last_infos_matrix"][self.robot_id].update({sender_belief_space["self_id"] : self.belief_space["robot_informations"][agent]["step"]})     
        #fusion
        for agent in set(list(self.belief_space["last_infos_matrix"].keys()) + list(sender_belief_space["last_infos_matrix"].keys())):
            if agent in sender_belief_space["last_infos_matrix"]:
                for com in sender_belief_space["last_infos_matrix"][agent]:
                    if self.belief_space["last_infos_matrix"][agent][com] < sender_belief_space["last_infos_matrix"][agent][com]:
                        self.belief_space["last_infos_matrix"][agent][com] = sender_belief_space["last_infos_matrix"][agent][com]
        # LAST COM MATRIX----------------------------------------------------
            

        #ARTIFACTS-----------------------------------------------------------
        for artifact in sender_belief_space["artifacts"]: #artifact update based on the newest timestamp
            if not (artifact in self.belief_space["artifacts"]):
                awared = sender_belief_space["artifacts"][artifact]["awared"]
                awared.append(self.robot_id)
                self.belief_space["artifacts"].update({artifact: sender_belief_space["artifacts"][artifact]})
                self.belief_space["artifacts"][artifact].update({"awared": awared})
                #self.belief_space["artifacts"][artifact].update({"discovery_time": self.env.step}) #each robot has it's own discovery time of the artifact. => the discovery time is supposed to be local
            
            elif sender_belief_space["artifacts"][artifact]["step"] > self.belief_space["artifacts"][artifact]["step"]:
                #keeping the global discovery time
                discovery_time = np.min([self.belief_space["artifacts"][artifact]["discovery_time"], sender_belief_space["artifacts"][artifact]["discovery_time"]])

                #update
                self.belief_space["artifacts"].update({artifact: sender_belief_space["artifacts"][artifact]})
                self.belief_space["artifacts"][artifact].update({"discovery_time": discovery_time})

            if sender_belief_space["artifacts"][artifact]["awared"] != self.belief_space["artifacts"][artifact]["awared"]:
                #merging the robots id of robots awared of the tasks
                sender_awared = sender_belief_space["artifacts"][artifact]["awared"]
                self_awared = self.belief_space["artifacts"][artifact]["awared"]
                global_awared = list(set(self_awared + sender_awared))

                self.belief_space["artifacts"][artifact].update({"awared": global_awared})

        #ARTIFACTS-----------------------------------------------------------

        self.new_communication = True
        # self.target = None
        # self.path_to_target = None

    def move(self, vector_x, vector_y):
        """Method that has to be redefined for each type of robot because they don't have the same movement mechanism."""
        print("move has to be implemented in the class.")

    def shape_competence(self, type, capability, importance, distance_treshold = 0, dispersion = 1):
        """
        Shape the competence for a robot
        
        :param type: String, nature of task
        :param capability: Float [0,1], measure how the robot is capable of performing the action of the given type
        :param importance: Float, Measure the perception of the robot of the importance of the given task type
        :param distance_treshold: distance where the robot will be able to detect the task, for example in communication, we don't want to take into account robots in range. default 0
        :param dispersion: Not currently used, may be removed soon TODO
        """
        self.competences.update({type:{"capability": capability, "importance":importance, "distance_treshold":distance_treshold, "dispersion":dispersion}})
    
    def perform_target_action(self):
        """simulate an artifact action with an interaction."""
        if self.action_to_perform["type"] == "exploration":
            self.action_to_perform = None
        elif self.action_to_perform["type"] == "communication":
            self.action_to_perform = None     
        else: #we'll consider than everything else is consider as an artifact
            for a in self.env.interest_points["artifacts"]:
                if a.id == self.action_to_perform["id"] and (euclidian_distance((int(a.coordinates[0]), int(a.coordinates[1])), (int(self.transform.x), int(self.transform.y)) ) < a.needed_robots):
                    
                    result = a.interact(self.competences[self.action_to_perform["type"]]["capability"])
                    if result :
                        self.belief_space["artifacts"][self.action_to_perform["id"]]["status"] = "done"
                        self.belief_space["artifacts"][self.action_to_perform["id"]]["step"] = self.env.step + 1
                        self.action_to_perform = None
                        self.belief_transfer()
                        return None
                    else:
                        #self.action_to_perform = None
                        return None
                # else:
                #     self.action_to_perform = None
                #     return None
            #si on ne voit pas l'artefact une fois sur place
            if euclidian_distance((int(self.belief_space["artifacts"][self.action_to_perform["id"]]["coordinates"][0]), int(self.belief_space["artifacts"][self.action_to_perform["id"]]["coordinates"][1])), (int(self.transform.x), int(self.transform.y))) <= self.vision_range:
                self.belief_space["artifacts"][self.action_to_perform["id"]]["status"] = "done"
                self.belief_space["artifacts"][self.action_to_perform["id"]]["step"] = self.env.step
                self.action_to_perform = None

    def behavior_diff_move_random(self): #could be called wiggle
        #set srobot speed at it's max speed
        self.speed.x = self.max_speed.x
        self.speed.y = self.max_speed.y

        #random rotation
        self.speed.w = random.uniform(-self.max_speed.w ,self.max_speed.w)
        self.transform.w += self.speed.w

        #2pi modulo
        self.transform.w = self.transform.w%(2*np.pi)


        #calculation of the x and y movement depending of the x direction speed and the w orientation.
        xmove = self.speed.x * np.cos(self.transform.w)
        ymove = self.speed.x * np.sin(self.transform.w)

        self.translate(xmove, ymove)

    def behavior_stay(self):
        self.belief_transfer()

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
                self.path_to_target = a_star_search(self.belief_space["occupancy_grid"], (int(self.transform.x),int(self.transform.y)), (self.target[0], self.target[1]), traversable_types=self.traversable_types) #from utils : A* Path calculation

        else: #sinon on va chercher les frontières.
            #frontier detection from belief space

            frontiers = find_frontier_cells(self.belief_space["occupancy_grid"], traversable_types=self.traversable_types) #from utils

            if list(frontiers) == None or len(list(frontiers))==0: #si on a pas de frontieres explo finie?
                if (int(self.transform.x),int(self.transform.y)) != (int(self.init_transform.x),int(self.init_transform.y)):
                    self.target = (int(self.init_transform.x),int(self.init_transform.y))
                    self.last_plan_time = self.env.step
                else:
                    self.finish()
            else:
                #then we take the closest one.
                distance = np.inf
                for f in frontiers:
                    hdist = heuristic_frontier_distance((self.transform.x, self.transform.y), (f[0], f[1]), self.belief_space["occupancy_grid"], traversable_types=self.traversable_types)
                    if hdist < distance :
                        distance = hdist
                        self.target = tuple(f.tolist()) #set the frontier as new target
                        self.last_plan_time = self.env.step
            
    def minpos_behavior(self): #from Bautin, 2012
        """
        Adaptation from MinPos algorithm (Bautin, Simonin, Charpillet : 2012)
        Frontier based behavior where:
        - The frontiers are grouped into clusters
        - each cluster is given a cost depending on the distance and on robots that are closer to this frontier using the wavefront propagation algorithm (WPA)
        - the robot chose the frontier with the lowest cost
        """

        #first of all, sense the environment
       
        #frontier detection
        frontiers = find_frontier_cells(self.belief_space["occupancy_grid"], traversable_types=self.traversable_types) #from utils

        if list(frontiers) == None or len(list(frontiers))==0: #si on a pas de frontieres explo finie?
            if (int(self.transform.x),int(self.transform.y)) != (int(self.init_transform.x),int(self.init_transform.y)):
                self.target = (int(self.init_transform.x),int(self.init_transform.y))
                self.last_plan_time = self.env.step
            else:
                self.finish()
        else:
            cluster_centers = cluster_frontier_cells(self.belief_space["occupancy_grid"], frontiers, int(self.vision_range/2), traversable_types=self.traversable_types) #from utils : make cluster fontiers

            pos_list_float = [pos["position"] for pos in list(self.belief_space["robot_informations"].values())] #list of float xy position of all robots
            pos_list_int = [(int(x), int(y)) for x,y in pos_list_float] #same list with ints.
            weighted_clusters = wavefront_propagation_algorithm(self.belief_space["occupancy_grid"], (int(self.transform.x), int(self.transform.y)), pos_list_int, cluster_centers, weight_of_closer_robots=self.env.width, traversable_types=self.traversable_types) #the penalty for a frontier cluster depends of the size of the env.
            self.target = min(weighted_clusters, key=weighted_clusters.get) #then we take the cluster with the minimum cost
            self.last_plan_time = self.env.step

    def behavior_rdv(self): #from Bramblett, 2022
        def cluster_env():
            # unknowns = np.column_stack(np.where(self.belief_space["occupancy_grid"]==-1))
            robots = list(self.belief_space["robot_informations"].keys())
            frontiers = find_frontier_cells(self.belief_space["occupancy_grid"], traversable_types=self.traversable_types)
            kmeans_clusters = KMeans(n_clusters=len(robots), random_state=0, n_init="auto").fit(frontiers)

            return kmeans_clusters

        def explore_subbehavior(delta = 1):
            art_found = []
            if "artifacts" in self.belief_space: #check artifacts for pi4 condition
                    for art in self.belief_space["artifacts"]:
                        #INTEREST POINT CREATION
                        if self.belief_space["artifacts"][art]["status"] not in ["done", "destroyed"] and self.current_clustering.predict(self.belief_space["artifacts"][art]["coordinates"]) == self.allocated_cluster:
                            art_found.append({"id": art, "type":self.belief_space["artifacts"][art]["type"]})

            #Pi1 condition in the paper
            if np.abs(self.env.step - self.rdvtime) < 1.5 * self.max_speed.x * a_star_cost(self.belief_space["occupancy_grid"], start = (int(self.transform.x), int(self.transform.y)), goal = (int(self.rdvspot[0]), int(self.rdvspot[1])), env_ease=self.env_ease): #if the time until rdvtime is shorter than 1.5* time to go for it, then, pass in rdv mode
                self.rdvstate = "rendezvous"
            #Pi4 condition in the paper
            elif len(art_found) > 0:
                self.action_to_perform = art_found[0] #TODO : Use equation 10 of the paper to decide if and which task to use????
                self.rdvstate = "search"

            elif self.target == None: #else, we stay in the explore state and recompute a target if necessary
                #sobel detection for frontier
                frontiers = find_frontier_cells(self.belief_space["occupancy_grid"], traversable_types= self.traversable_types)
                if len(frontiers)>0:
                    fcosts = []
                    for f in frontiers:
                        prediction = self.current_clustering.predict([f])[0]
                        cluster_of_pred = (int(self.current_clustering.cluster_centers_[prediction][0]), int(self.current_clustering.cluster_centers_[prediction][1]))

                        if cluster_of_pred == self.allocated_cluster: #the cell is in our custer

                            cost = euclidian_distance((self.transform.x, self.transform.y), f) #eq.6, case2
                        else: #celll not in allocated cluster
                            alloc_cluster_coords = (int(self.allocated_cluster[0]), int(self.allocated_cluster[1]))
                            cost =  euclidian_distance((self.transform.x, self.transform.y), f) + delta*euclidian_distance(f, alloc_cluster_coords) #eq.6, case1
                        fcosts.append(cost)
                    
                    explopoint = frontiers[np.argmin(fcosts)] #eq. 7
                    self.target  = (int(explopoint[0]), int(explopoint[1]))
                else:
                    self.rdvstate = "rendezvous"
                #print(self.target)

            #explo plus proche, OU ALORS, on garde le kmeans et on fait un predict sur les nouvelles frontieres du sobel???

            #explore until rdv time limitation

        def rendezvous_subbehavior():

            # Set a goal point?
            #check if others are here (with the rdv time limitation)
            missing_robot = []
            if self.env.step < 10: #pour le tout debut de mission, que les robots aient le temps de se donner l'info qu'ils existent^^
                return
        
            for robot in self.belief_space["robot_informations"]:
                #if (euclidian_distance((self.transform.x, self.transform.y), self.belief_space["robot_informations"][robot]["position"])<self.communication_range/2 ) and  (self.belief_space["robot_informations"][robot]["step"] > self.env.step - 40): #we check that the robot is actually here and not an old position
                if (self.belief_space["robot_informations"][robot]["step"] > self.env.step - 20): #test de condition sans distance, juste info recente
                    continue
                else:
                    missing_robot.append(robot) #if it's not here, he is missing at this point.
            if len(missing_robot) > 0 and euclidian_distance((self.transform.x, self.transform.y), self.rdvspot) > self.communication_range/2: #goto rdv # TODO TODO TODO TODO TODO mettre un treshold d'acceptation de target pour s'arreter si jamais
                #self.treshold_for_target = self.communication_range/2
                self.target = self.rdvspot
                return

            if len(missing_robot) == 0 or self.rdvtime <= self.env.step :
                #then with the robots that are here : 
                #identify actions and explo clusters
                if self.bid == None or (self.env.step - self.bid["step"]) >= 100:#no bid, or bid too old.
                    frontier_bids = {}
                    artifacts_bids = {}
                    frontiers = find_frontier_cells(self.belief_space["occupancy_grid"], traversable_types = self.traversable_types)
                    if len(frontiers) >1:

                        self.current_clustering = cluster_env() #KMeans cluster object
                        cluster_centers = self.current_clustering.cluster_centers_ 

                        for cc in cluster_centers:
                            cost = euclidian_distance((self.transform.x, self.transform.y), (int(cc[0]), int(cc[1])))
                            frontier_bids.update({(int(cc[0]), int(cc[1])):(1/cost)}) #then high cost will make a small bid.
                        if "artifacts" in self.belief_space:
                            for art in self.belief_space["artifacts"]:
                                #check capability
                                if self.belief_space["artifacts"][art]["status"] not in ["destroyed", "done"]:
                                    type = self.belief_space["artifacts"][art]["type"]
                                    capability = self.competences[type]
                                    artifacts_bids.update({art:(capability/cost)})
                    self.bid = {"frontiers":frontier_bids, "artifacts":artifacts_bids, "step": self.env.step}
                    self.belief_space["robot_informations"][self.robot_id].update({"bids":self.bid})
                    #avec ca, la communication devrait automatiquement partager les bids, vu que les robots envoient leurs infos du belief space en entier.
                missing_bids = False
                for robot in self.belief_space["robot_informations"]: #TODO : Harmoniser les bids entre les robots.
                    if robot in missing_robot:
                        continue
                    if not("bids" in list(self.belief_space["robot_informations"][robot].keys())):
                        missing_bids = True
                        break
                    elif self.belief_space["robot_informations"][robot]["bids"]["step"] <= self.env.step - 100:
                            missing_bids = True
                            break
                if not missing_bids: #here, we use hungarian algorithm, so we have a one-shot auction
                    #TODO : faire une matrice avec les bids des robot non-missing, puis identifier la tâche à effectuer pour le robot faisant le calcul.
                    robots_present = []
                    for r in self.belief_space["robot_informations"]:
                        if not(r in missing_robot):
                            robots_present.append(r)
                    #robots_present = [r for r in self.belief_space["robot_informations"] if r not in missing_robot]
                    tasks_frontiers = list(self.bid["frontiers"].keys())
                    tasks_artifacts = list(self.bid["artifacts"].keys())

                    # Initialiser la matrice d'affectation (bids)
                    exploration_matrix = np.zeros((len(robots_present), len(tasks_frontiers)))
                    artifact_matrix = np.zeros((len(robots_present), len(tasks_artifacts)))
                    # Remplir la matrice avec les bids des robots pour chaque tâche
                    for i, robot in enumerate(robots_present):
                        # artefacts 
                        if len(tasks_artifacts) >0:
                            for j, task in enumerate(tasks_artifacts):
                                artifact_matrix[i, j] = self.belief_space["robot_informations"][robot]["bids"]["artifacts"][task]
                        # frontières
                        if len(tasks_frontiers) >0:
                            for j, task in enumerate(tasks_frontiers):
                                if task not in list(self.belief_space["robot_informations"][robot]["bids"]["frontiers"].keys()):
                                    #we need to infer which cluster the bid is for
                                    for cluster in list(self.belief_space["robot_informations"][robot]["bids"]["frontiers"]):
                                        if self.current_clustering.predict([cluster]) == self.current_clustering.predict([task]):
                                            exploration_matrix[i,j] = self.belief_space["robot_informations"][robot]["bids"]["frontiers"][cluster]
                                else:
                                    exploration_matrix[i,j] = self.belief_space["robot_informations"][robot]["bids"]["frontiers"][task]

                    # hungarian affectations robot -> tache :  du robot self
                    #artefacts
                    if len(tasks_artifacts) >0:
                        m_artifact = Munkres()
                        artifact_indices = m_artifact.compute(-artifact_matrix)

                        for r_idx, t_idx in artifact_indices:
                            if robots_present[r_idx] == self.robot_id:
                                #robot_id = robots_present[r_idx]
                                task_id = tasks_artifacts[t_idx]
                                self.action_to_perform = {"id": task_id, "type":self.belief_space["artifacts"][task_id]["type"]}
                                self.rdvstate = "search"
                                self.bid = None
                                break
                    
                    if len(find_frontier_cells(self.belief_space["occupancy_grid"], traversable_types=self.traversable_types))==0 and len(tasks_artifacts)==0 : #if there is no frontier and no task anymore, we finish
                        self.rdvstate = "finish"
                        return

                    # explo
                    if len(tasks_frontiers) >0:
                        m_frontier = Munkres()
                        cluster_indices = m_frontier.compute(-exploration_matrix)
                        for r_idx, t_idx in cluster_indices:
                            if robots_present[r_idx] == self.robot_id:
                                task_id = tasks_frontiers[t_idx]
                                self.allocated_cluster = task_id
                                if self.rdvstate != "search":
                                    self.rdvstate = "explore"
                                    self.bid = None
                                break
                    
                    
                    #SETUP NEXT RDV SPOT
                    

                    partition = np.zeros(self.belief_space["occupancy_grid"].shape, dtype=int)

                    # partition[self.belief_space["occupancy_grid"] == -1] = self.current_clustering.labels_ + 1
                    frontiers = find_frontier_cells(self.belief_space["occupancy_grid"])

                    frows = [f[0] for f in frontiers]
                    fcols = [f[1] for f in frontiers]
                    partition[frows, fcols] = -1

                    unknown_mask = partition == -1
                    unknown_coords = np.argwhere(unknown_mask)  # cellules inconnues actuelles

                    partition[unknown_mask] = self.current_clustering.predict(unknown_coords) + 1 # test
                    #je traduis du mieux que je peux le code matlab de bramblett sur le gitub. Elle a l'air de faire une moyenne ponderee des centroides
                    #par la taille des partitions.

                    
                    unk_part = partition[partition != 0] 
                    labels, a_counts = np.unique(unk_part, return_counts=True)
                    c_loc = self.current_clustering.cluster_centers_ 

                    np_rdvspot = np.round(np.sum(c_loc[labels - 1] * a_counts[:, np.newaxis], axis=0) / len(unk_part)).astype(int) #TODO, il faut que je ne prenne plus en compte les zones inconnues inaccessibles. Il faudrait que je les purge en fait...
                    self.rdvspot = (int(np_rdvspot[0]), int(np_rdvspot[1]))
                    if not(self.belief_space["occupancy_grid"][self.rdvspot] in self.traversable_types):
                        self.rdvspot = find_nearest_free(self.belief_space["occupancy_grid"], self.rdvspot, traversable_types=self.traversable_types) #si le rdv est un mur ou une case inconnue, alors, on 
            
                    
                    #Note :  je fais les rdv de manière decentralisee, normalement chaque robot attends d'avoir l'info que les autres sont dans le cluster
                    #donc EN THEORIE tout le monde a la meme map, les clusters et donc les points de rdv devraient etre les memes.....
                    #en pratique, on verra^^

                    self.rdvtime = self.env.step + np.max(self.belief_space["occupancy_grid"].shape)*2 #TODO maybe set a better incrementation value.
            
        def search_subbehavior():
            artifact_coordinates = self.belief_space["artifacts"][self.action_to_perform["id"]]["coordinates"]
            if euclidian_distance((int(self.transform.x), int(self.transform.y)), artifact_coordinates) < self.vision_range:
                self.rdvstate = "exploit"
            else:
                self.target = artifact_coordinates

        def exploit_subbehavior():
            artifact_status = self.belief_space["artifacts"][self.action_to_perform["id"]]["status"]
            #TODO : EQUATION 9 to check if rdv is locally optimal, then make the transition if so
            if artifact_status in ["done", "destroyed"]:
                self.rdvstate = "explore"
            #make the job until done or rdv time limitation

        def finish_subbehavior():
            if euclidian_distance((int(self.transform.x),int(self.transform.y)), (int(self.init_transform.x),int(self.init_transform.y))) > self.treshold_for_target:
                self.target = (int(self.init_transform.x),int(self.init_transform.y))
                self.last_plan_time = self.env.step
                return None
            else:
                self.finish()
                return None

        if "rdvstate" not in self.__dict__.keys(): #for initialisation, we set rdv originally
            self.rdvstate = "rendezvous"
            self.bid = None
            self.rdvspot = (int(self.transform.x), int(self.transform.y))
            self.rdvtime = 20
            self.allocated_cluster = None
            self.current_clustering = None

            #we have to cheat here cause this method requires the robot to know each other at the beggining of the mission.
            for r in self.env.agents:
                if r.robot_id != self.robot_id:
                    BS_copy = deepcopy(self.belief_space)
                    r.recieve_belief(BS_copy)

        if self.rdvstate == "explore":
            explore_subbehavior()
        elif self.rdvstate == "rendezvous":
            rendezvous_subbehavior()
        elif self.rdvstate == "search":
            search_subbehavior()
        elif self.rdvstate == "exploit":
            exploit_subbehavior()
        elif self.rdvstate == "finish":
            finish_subbehavior()


        pass

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
        #LOCAL FRONTIER DETECTION -----------------------------------------------------
        vision_range = self.get_neighbors_pixels(distance=self.vision_range, stop_at_wall=True, self_inclusion=True)
        local_frontier_list = []
        for cell in vision_range:
            if not(self.belief_space["occupancy_grid"][cell[0]][cell[1]] in self.traversable_types):
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
                    self.finish()
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
                    self.last_plan_time = self.env.step

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
                self.last_plan_time = self.env.step

        self.belief_transfer() #belief transfer management.
    
    def navigate_through_target_path(self):
        def make_the_move(waypoint):
            direction = (waypoint[0] - int(self.transform.x), waypoint[1] - int(self.transform.y))

            self.move(direction[0], direction[1])

        #we should be nearby the first point of the path, else we delete it and we'll compute an other one:
        if euclidian_distance((int(self.transform.x), int(self.transform.y)), (self.path_to_target[0][0], self.path_to_target[0][1])) <= 5: #if we are more than 5 away from the path, we forget the target it in order to recalculate a new one
            if euclidian_distance(self.path_to_target[0],self.target) <= self.treshold_for_target:
                waypoint = self.path_to_target[0]
                make_the_move(waypoint)
                
                self.target = None #forget the target and the path
                self.treshold_for_target = 1 #we reset the treshold at default value each time a traject is over.
                self.path_to_target = None
            else:
                self.path_to_target.pop(0)
                waypoint = self.path_to_target[0]

                make_the_move(waypoint)

                pass
        else:
            #Path not accurate.
            self.behavior_diff_move_random() #random move to maybe select another frontier.
            self.path_to_target = None #forget the target and the path
            self.target = None

    def behavior_action_selection(self): 
        #reshape importance of communication depending of the time from last communication:
        #print(f"robot {self.robot_id} : last com : {self.time_from_last_communication}")



       
        # fonction a part, je met un parametre en string "default" par defaut et un mode pour chaque expe tentée?
        self.reshape_com_importance_for_action_selection(mode = self.com_importance_mode)

        #self.check_communication_importance()

        interest_points = [] #we will add all of our interest points here
        #interest points identification -----------------------------------------------------------
        #exploration frontiers ----------------------------
        frontiers = find_frontier_cells(self.belief_space["occupancy_grid"], traversable_types=self.traversable_types) #from utils
        if list(frontiers) != None or len(list(frontiers))!=0:
            cluster_centers = cluster_frontier_cells(self.belief_space["occupancy_grid"], frontiers, int(self.vision_range/2), traversable_types=self.traversable_types) #from utils : make cluster of fontiers to reduce computation time
            for cc in cluster_centers:
                interest_points.append({"type":"exploration","coordinates":cc, "needed_robots":1})
        #--------------------------------------

        #Artifacts ----------------------------
        if "artifacts" in self.belief_space:
            for art in self.belief_space["artifacts"]:
                #IMPORTANCE CHECK
                if art +1 <= len(self.env.interest_points["artifacts"]): #ouais c'est degueu
                    self.env.interest_points["artifacts"][art].check_importance(self)

                #INTEREST POINT CREATION
                if self.belief_space["artifacts"][art]["status"] not in ["done", "destroyed"] :
                    if euclidian_distance( (self.init_transform.x, self.init_transform.y) , self.belief_space["artifacts"][art]["coordinates"]) >= self.competences[self.belief_space["artifacts"][art]["type"]]["distance_treshold"]: #we verify that the treshold is respected
                        interest_points.append({"type": self.belief_space["artifacts"][art]["type"] ,
                                                "coordinates":self.belief_space["artifacts"][art]["coordinates"], 
                                                "id":art, 
                                                "needed_robots": self.belief_space["artifacts"][art]["needed_robots"],
                                                "step": self.belief_space["artifacts"][art]["step"],
                                                "discovery_time": self.belief_space["artifacts"][art]["discovery_time"],
                                                "awared": self.belief_space["artifacts"][art]["awared"]})#adding directly the artifacts in the interest points
        #--------------------------------------
        #------------------------------------------------------------------------------------------


        #barycentre de communications----------
        #liste de toutes les positions des robots

        robots_pos_list = [] #list of float xy position of all robots
        for robot_id in self.belief_space["robot_informations"]:
            if robot_id != self.robot_id and self.belief_space["robot_informations"][robot_id]["status"]!= "finishing":  # ComImportance : est ce que je ferais pas un truc spécifique aux robots?
                if self.communication_range*4 <= self.env.step - self.belief_space["robot_informations"][robot_id]["step"] : # < np.max(self.belief_space["occupancy_grid"].shape)/self.max_speed.x :
                    if euclidian_distance((self.transform.x, self.transform.y) ,self.belief_space["robot_informations"][robot_id]["position"]) >=  self.competences["communication"]["distance_treshold"]:
                        robots_pos_list.append(self.belief_space["robot_informations"][robot_id]["position"])
            if len(robots_pos_list)>0:
                if euclidian_distance((self.transform.x, self.transform.y) , self.last_given_position) >=  self.competences["communication"]["distance_treshold"]:
                        robots_pos_list.append(self.last_given_position)# TODO ComInfo, la last given position, c'est a double trnchant, je sais pas trop

        communication_clusters = simple_clustering(robots_pos_list, self.communication_range) #from utils: make simple clusters of robot based on communication range, will return the center of clusters
        for cc in communication_clusters:
                #if euclidian_distance( (self.init_transform.x, self.init_transform.y) , cc) >= self.competences["communication"]["distance_treshold"]: #we verify that the distance treshold is respected
                interest_points.append({"type":"communication","coordinates":cc, "needed_robots":1})#adding those clusters in the communication points
                #TODO MultiRobotTask : try different values of needed robots


        #--------------------------------------        

        #if we have no interest point anymore, we consider the mission done.*
        if len(interest_points) == 0: # or (len(interest_points)==1 and interest_points[0]["type"] == "base_station_com"):
            if euclidian_distance((int(self.transform.x),int(self.transform.y)), (int(self.init_transform.x),int(self.init_transform.y))) > self.treshold_for_target:
                self.status = "finishing"
                self.target = (int(self.init_transform.x),int(self.init_transform.y))
                self.last_plan_time = self.env.step
                return None
            else:
                self.finish()
                return None

        #utility calculation-----------------------------------------------------------------------            
        for ip in interest_points:
            #individual utility
            #cost = euclidian_distance(ip["coordinates"], (self.transform.x, self.transform.y)) #euclidian distance for the moment (C in the model)
            cost = a_star_cost(self.belief_space["occupancy_grid"], (int(self.transform.x), int(self.transform.y)), (int(ip["coordinates"][0]), int(ip["coordinates"][1])), self.env_ease, traversable_types=self.traversable_types)
            if cost < 1:
                cost = 1 #avoid divide by 0

            capability = self.competences[ip["type"]]["capability"] #I'll cnsider that the type of the IP will be named the same than the competence (mu in the model)

            individual_utility = capability/cost
            #global feasability
            other_individual_values = np.array([])
            for robot in self.belief_space["robot_informations"]: #the key value of this dict is robot id
                if len(self.belief_space["robot_informations"]) <=1:
                    other_individual_values = np.append(other_individual_values, 1.0)
                    break
                if robot == self.robot_id :
                    continue
                else:
                    #ligne de l'enfer sorry
                    other_robot_pos = (int(self.belief_space["robot_informations"][robot]["position"][0]),int(self.belief_space["robot_informations"][robot]["position"][1]))

                    #ocost = euclidian_distance(ip["coordinates"], other_robot_pos)
                    ocost = a_star_cost(self.belief_space["occupancy_grid"], other_robot_pos, (int(ip["coordinates"][0]), int(ip["coordinates"][1])), self.belief_space["robot_informations"][robot]["env_ease"], traversable_types=self.belief_space["robot_informations"][robot]["traversable_types"])
                    if ocost <1:
                        ocost = 1 #avoid divide by 0

                    ocapability = self.belief_space["robot_informations"][robot]["competences"][ip["type"]]["capability"]

                    oobsolecence = self.env.step - self.belief_space["robot_informations"][robot]["step"] 

                    #TODO, MultiRobotTask check ca
                    if ip["needed_robots"] <=1:
                        other_individual_values = np.append(other_individual_values, (ocapability/ocost))
                    else:
                        #self.belief_space["robot_informations"][robot]["step"]
                        if  robot in ip["awared"] : #check if the robot knows about the task or not
                            other_individual_values = np.append(other_individual_values, (ocapability/ocost))
                
                     #capacite des autres sur l'ip 
            
            #collective_sufficiency = float(np.max(other_individual_values)) #backup

            #for MultiRobotTask::
            if len(other_individual_values) >= ip["needed_robots"]:
                collective_sufficiency = float(max_k(other_individual_values, ip["needed_robots"]))
            elif len(other_individual_values) == ip["needed_robots"]-1:
                collective_sufficiency = 1 #TODO TODO TODO: MultiRobotTask: je ne suis pas sur sur de cette valeur la, quesque j'ai foutu?????
            else:
                collective_sufficiency = np.inf
            #collective_sufficiency = testproduct

            if collective_sufficiency == 0:
                collective_sufficiency = 1e-8 #avoid divide by 0



            bests_others = [] #TODO MultiRobotTask: test ca
            required_assist = 0
            if ip["needed_robots"] > 1: 
                # #required_assist = 1
                if len(other_individual_values) >= ip["needed_robots"]-1:
                    nbcloser = 0
                    for i in range (ip["needed_robots"] -1):
                        value = float(max_k(other_individual_values, i+1))
                        bests_others.append(value)
                        if value > individual_utility:
                            nbcloser+=1

                    #TODO TODO TODO TODO TODO : MultiRobotTask: il faut vraiment ajuster le required assist
                    required_assist = (float(np.sum(bests_others)) - (1+nbcloser - ip["needed_robots"])) * ((self.env.step - ip["discovery_time"])/self.env.step) #TODO ajuster le delta discovery
                else:
                    required_assist = - individual_utility
                
                #required_assist = 1 + float(max_k(other_individual_values, ip["needed_robots"] -1)) #equivalent to the commented above...
            

            # if required_assist !=0:
            #     required_assist += self.env.step - ip["step"]
        
            
            utility = ((individual_utility + required_assist) / (collective_sufficiency ))
            #utility = ( self.competences[ip["type"]]["importance"] * individual_utility) / collective_sufficiency

            ip.update({"utility":utility})
            #ip.update({"utility":collective_utility})

        #------------------------------------------------------------------------------------------
        best_action = None
        best_weighted_utility = -np.inf
        for ip in interest_points:
            #tuning params-----------------------------------------------------------------------------
            weighted_utility = ip["utility"] * self.competences[ip["type"]]["importance"]

            if weighted_utility >= best_weighted_utility:
                best_weighted_utility = weighted_utility
                best_action = ip
        
        #action perform
        if best_action != None:
            self.action_to_perform = best_action
            self.target = (int(self.action_to_perform["coordinates"][0]), int(self.action_to_perform["coordinates"][1]))
            self.last_plan_time = self.env.step
        else:
            print("problem")
    
    def reshape_com_importance_for_action_selection(self, mode="default"):
        capability = self.competences["communication"]["capability"] #same, doesnt change
        distance_treshold = self.communication_range 

        longest_infotime = 0
        for robot in self.belief_space["last_infos_matrix"][self.robot_id]:
            if robot == self.robot_id:
                continue
            else:
                #if self.belief_space["last_infos_matrix"][self.robot_id][robot] < self.env.width :#treshold to regulate : definir quel treshold est pertinent maintenant
                infotime = self.belief_space["last_infos_matrix"][self.robot_id][robot]
                #print(infotime)
                if self.env.step - infotime > longest_infotime:
                    longest_infotime = self.env.step - infotime

        com_time = longest_infotime #longest synchro from every robots synchro time # ComImportance
        #com_time =  self.time_from_last_communication #time of last communication with any robots
    
        
        #ComInfo : faire un truc coherent pour lancer les expes. Bien identifier ce qui marche, et ce qui ne marche pas.
        
        if mode == "default":
            importance = np.exp(com_time/ self.env.width) #value to be changed
        elif mode =="linear":
            importance = 1.5*com_time - self.env.step #/ self.env.width  #???????????????????
        elif mode =="polynomial2":
            importance = ((com_time/self.communication_range)**2)-self.env.step
        elif mode =="polynomial3":
            importance = ((com_time/self.communication_range)**3)-self.env.step
        elif mode =="exponential":
            importance = np.exp(com_time/ self.communication_range)/self.env.step #value to be changed
        elif mode =="rule-based":
            if com_time < 30: #treshold for no need at all
                importance = 0
            elif com_time >= np.sqrt(np.count_nonzero(self.belief_space["occupancy_grid"] != -1))/np.mean([self.max_speed.x, self.max_speed.y]): # s'adapte en fonction de la taille de l'env decouvert.
                #print("aaa")
                importance = np.inf
            else:
                importance = 1.5*com_time - self.env.step #linear otherwise
        elif mode =="test":
            importance = np.exp(com_time/ self.communication_range)  #value to be changed
        else:
            raise Exception(f"incorrect importance com mode in agent {self.robot_id}")

        

        self.shape_competence("communication", capability=capability , importance=importance, distance_treshold=distance_treshold)


    def write_logs(self):
        """will keep logs in ram at each steps for simulation stats"""
        step = self.env.step
        if self.action_to_perform:
            action = self.action_to_perform['type']
        else:
            action = None

        self.logs.update({
            step:{
                "action":action,
                "transform":{"x":self.transform.x, "y":self.transform.y, "w":self.transform.w},
                "last_infos_matrix" : deepcopy(self.belief_space["last_infos_matrix"]),
                "target": self.target
            }
        })



class Ground(Robot):

    def __init__(self, env, robot_id, size = 1, color = (0, 255, 0), init_transform = (0,0,0), max_speed = (1.0,0.0,1.5),vision_range=20, communication_range = 40, communication_period = 10, behavior_to_use = "random", energy_amount = 1000, energy_cost_per_cell = 1, delta_replan=20, write_logs=False, graph_mode=False, graph_delta=50):
        super().__init__(env, robot_id, size, color, init_transform= init_transform, max_speed=max_speed, vision_range=vision_range, communication_range=communication_range, communication_period=communication_period, energy_amount = energy_amount, energy_cost_per_cell = energy_cost_per_cell, delta_replan=delta_replan, write_logs=write_logs, graph_mode=graph_mode, graph_delta=graph_delta)
        self.behavior_space = ["random", "target_djikstra", "nearest_frontier", "minpos", "local_frontier", "action_selection", "rendezvous"]

        #traversability ease in the env 
        self.env_ease = {
            OG_FREE_CELL_GROUP_NAME:1,
            OG_WALL_GROUP_NAME:0,
            OG_HIGH_WALL_GROUP_NAME:0,
            OG_SAND_GROUP_NAME:0.4,
            OG_WATER_GROUP_NAME:0,
            OG_GRASS_GROUP_NAME:0.6
        }

        self.traversable_types = list(filter(lambda k: self.env_ease[k] != 0, self.env_ease)) #find the cells that the robot can eventually traverse
        for i in range(len(self.traversable_types)):#we have the string name of the cells types, lets get the int values
            self.traversable_types[i] = ENV_CELL_TYPES[self.traversable_types[i]]


        #handle behavior space string
        if not( behavior_to_use in self.behavior_space) :
            logging.error(f"Ground robot:init -> behavior_to_use not in the behavior space.\n the behavior should be in {self.behavior_space}")
            exit()
        else : 
            self.behavior = behavior_to_use

    def behave(self):
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
                case "action_selection":
                    self.behavior_action_selection()
                case "rendezvous":
                    self.behavior_rdv()

    def move(self, vector_x, vector_y):
        angle = np.arctan2(vector_y, vector_x)
            
        #Angle to rotate to go in the neighbor direction
        tfw = (angle - self.transform.w)%(2*np.pi)  #differance of angle

        self.speed.w = tfw

        self.transform.w += self.speed.w
        #2pi modulo
        self.transform.w = self.transform.w%(2*np.pi)
        #self.speed.x = direction_vers_voisin

        self.speed.x = min(euclidian_distance((0,0), (vector_x,vector_y)), self.max_speed.x)
        
        xmove = self.speed.x * np.cos(self.transform.w)
        ymove = self.speed.x * np.sin(self.transform.w)

        self.translate(xmove, ymove)


class Aerial(Robot):
    def __init__(self, env, robot_id, size = 1, color = (255, 0, 0), init_transform = (0,0,0), max_speed = (1.0,1.0,1.5),vision_range=20, communication_range = 40, communication_period = 10, behavior_to_use = "random", energy_amount = 1000, energy_cost_per_cell = 1, delta_replan=20, write_logs=False, graph_mode=False, graph_delta=50):
        super().__init__(env, robot_id, size, color, init_transform= init_transform, max_speed=max_speed, vision_range=vision_range, communication_range=communication_range, communication_period=communication_period, energy_amount = energy_amount, energy_cost_per_cell = energy_cost_per_cell, delta_replan=delta_replan, write_logs=write_logs, graph_mode=graph_mode, graph_delta=graph_delta)
        self.behavior_space = ["random", "target_djikstra", "nearest_frontier", "minpos", "local_frontier", "action_selection"]

        #traversability ease in the env 
        self.env_ease = {
            OG_FREE_CELL_GROUP_NAME:1,
            OG_WALL_GROUP_NAME:1,
            OG_HIGH_WALL_GROUP_NAME:0,
            OG_SAND_GROUP_NAME:1,
            OG_WATER_GROUP_NAME:1,
            OG_GRASS_GROUP_NAME:1
        }
        self.belief_space["robot_informations"][self.robot_id]["env_ease"] = self.env_ease

        self.traversable_types = list(filter(lambda k: self.env_ease[k] != 0, self.env_ease)) #find the cells that the robot can eventually traverse
        for i in range(len(self.traversable_types)):#we have the string name of the cells types, lets get the int values
            self.traversable_types[i] = ENV_CELL_TYPES[self.traversable_types[i]]

        self.belief_space["robot_informations"][self.robot_id]["traversable_types"] = self.traversable_types

        #handle behavior space string
        if not( behavior_to_use in self.behavior_space) :
            logging.error(f"Ground robot:init -> behavior_to_use not in the behavior space.\n the behavior should be in {self.behavior_space}")
            exit()
        else : 
            self.behavior = behavior_to_use
    
    def behave(self):
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
                case "action_selection":
                    self.behavior_action_selection()

    def move(self, vector_x, vector_y):
        self.speed.x = min(self.max_speed.x, vector_x)
        self.speed.y = min(self.max_speed.y, vector_y)

        self.translate(self.speed.x, self.speed.y)

class BaseStation(Robot):
    class BaseStationArtifact(Artifact):
        def __init__(self, env, id, name, coordinates, associated_agent:Robot, size=1, color = (255,0,0)):
            type = "base_station_com"
            super().__init__(env, id, name, type, coordinates, size, color)
            self.base = associated_agent
        
        def check_importance(self, robot:Robot):
            oldest_com_time = self.env.step
            if self.base.robot_id in robot.belief_space["last_infos_matrix"]:
                for agent in robot.belief_space["last_infos_matrix"][self.base.robot_id]:
                    if robot.belief_space["last_infos_matrix"][self.base.robot_id][agent] < oldest_com_time:
                        oldest_com_time = robot.belief_space["last_infos_matrix"][self.base.robot_id][agent]
            

            #importance = np.exp(((self.env.step - oldest_com_time)*self.base.return_priority ) - (((self.env.width + self.env.height)/2))) /self.env.step #/(np.sqrt(self.env.step))) #longest time of any agent news / mean of env size in term of width&height
            #importance = np.exp(((self.env.step - oldest_com_time) - ((self.env.width + self.env.height)/2))/(np.sqrt(self.env.step))) #longest time of any agent news / mean of env size in term of width&height


            #importance = ((np.max([0.0,(self.env.step - oldest_com_time) - ((self.env.width + self.env.height)/2)]))**(1 + self.base.return_priority)) / (self.env.step) # TO KEEP
            #importance = ((np.max([0.0,(self.env.step - oldest_com_time)]) - ((self.env.width + self.env.height)/2+self.base.return_priority))**2) / self.env.step
            importance = np.max([0.0,(((self.env.step - oldest_com_time)*self.base.return_priority ) - (self.env.width )/2)])**3 /self.env.step**2



            

            total_timestamps = 0
            total_base_timestamp = 0
            for agent in robot.belief_space["last_infos_matrix"][robot.robot_id]:
                if agent != self.base.robot_id:
                    total_timestamps += robot.belief_space["last_infos_matrix"][robot.robot_id][agent] #we add all timesteps of the other robots except the base
            if self.base.robot_id in robot.belief_space["last_infos_matrix"]:
                for agent in robot.belief_space["last_infos_matrix"][self.base.robot_id]:
                    if agent != self.base.robot_id:
                        total_base_timestamp += robot.belief_space["last_infos_matrix"][robot.robot_id][agent] #we add all timesteps of the other robots except the base
            
            total_deltas_com = total_timestamps - ((len(robot.belief_space["last_infos_matrix"])-1) * self.env.step)  # delte = la somme des timestamps - n * le max des steps possible (soit le temps actuel) et n = le nb de robot dans la flotte sans la base.
            total_base_deltas_com = total_base_timestamp - ((len(robot.belief_space["last_infos_matrix"])-1) * self.env.step)

            capability = total_deltas_com / (total_base_deltas_com + 1e-8)
            #-----------------------------------------------------------------

            
            
            #robot.shape_competence(self.type, capability=robot.competences[self.type]["capability"], importance=importance, distance_treshold = 2)
            robot.shape_competence(self.type, capability=capability, importance=importance, distance_treshold = 2)
     
        def interact(self, competence):
            if self.env.goal_condition():
                self.base.imdone = True
                self.destroy()
                return True
            else:
                return False

    def __init__(self, env, robot_id, size = 1, color = (0, 0, 255), init_transform = (0,0,0), max_speed = (0.0 ,0.0 , 0.0),vision_range=20, communication_range = 40, communication_period = 1, behavior_to_use = "random", energy_amount = 1000, energy_cost_per_cell = 1, delta_replan=20, write_logs=False, return_priority=1):
        super().__init__(env, robot_id, size, color, init_transform= init_transform, max_speed=max_speed, vision_range=vision_range, communication_range=communication_range, communication_period=communication_period, energy_amount = energy_amount, energy_cost_per_cell = energy_cost_per_cell, delta_replan=delta_replan, write_logs=write_logs)
        self.behavior_space = ["stay"]
        self.return_priority = return_priority

        #traversability ease in the env 
        self.env_ease = {
            OG_FREE_CELL_GROUP_NAME:0,
            OG_WALL_GROUP_NAME:0,
            OG_HIGH_WALL_GROUP_NAME:0,
            OG_SAND_GROUP_NAME:0,
            OG_WATER_GROUP_NAME:0,
            OG_GRASS_GROUP_NAME:0
        }

        self.traversable_types = list(filter(lambda k: self.env_ease[k] != 0, self.env_ease)) #find the cells that the robot can eventually traverse
        for i in range(len(self.traversable_types)):#we have the string name of the cells types, lets get the int values
            self.traversable_types[i] = ENV_CELL_TYPES[self.traversable_types[i]]

        self.shape_competence("exploration", capability=0, importance=1) #that robot can't explore, so exp capability is set to zero
        self.shape_competence("communication", capability=0, importance=1) # as it can't move, communication purpose mvt capability is also settled to 0

        #handle behavior space string
        if not( behavior_to_use in self.behavior_space) :
            logging.error(f"Ground robot:init -> behavior_to_use not in the behavior space.\n the behavior should be in {self.behavior_space}")
            exit()
        else : 
            self.behavior = behavior_to_use
        self.artifact = None
        self.create_artifact()
        self.shape_competence(self.artifact.type, capability=0, importance=0)
    
    def behave(self):
        if not(self.imdone):
            self.behavior_stay()
            staying = 0
            for r in self.env.agents:
                if self.robot_id != r.robot_id:
                    if r.imdone:
                        staying +=1
            if staying != 0:
                self.artifact.destroy()
                self.imdone = True

    def create_artifact(self):
        """
        The base station specificly place a BS artifact on itself at creation
        """
        self.artifact = self.BaseStationArtifact(env=self.env,
                                                 id=len(self.env.interest_points["artifacts"]),
                                                 name="base_station",
                                                 coordinates=(self.transform.x, self.transform.y),
                                                 associated_agent=self)
        self.env.interest_points["artifacts"].append(self.artifact)