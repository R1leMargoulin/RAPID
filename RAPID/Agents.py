from pygame.sprite import Sprite, spritecollide
from pygame.transform import scale
from pygame.draw import circle
from pygame import Surface, SRCALPHA, Rect

from .Environment import Environment
from .Artifacts import Artifact
from .utils import Transform2d, a_star_search, euclidian_distance, DIRECTIONS
from .grid_variables import (ENV_CELL_TYPES, BLOCKING_SENSOR_TYPES, OG_UNKNOWN_CELL, OG_FREE_CELL_GROUP_NAME,
                             OG_WALL_GROUP_NAME, OG_HIGH_WALL_GROUP_NAME, OG_SAND_GROUP_NAME,
                             OG_WATER_GROUP_NAME, OG_GRASS_GROUP_NAME)
from .behaviors import action_selection, local_frontier, rendezvous, minpos, nearest_frontier, random_wiggle


import numpy as np
import random
from copy import deepcopy


def _ease(free, wall, high_wall, sand, water, grass):
    return {OG_FREE_CELL_GROUP_NAME: free, OG_WALL_GROUP_NAME: wall, OG_HIGH_WALL_GROUP_NAME: high_wall,
            OG_SAND_GROUP_NAME: sand, OG_WATER_GROUP_NAME: water, OG_GRASS_GROUP_NAME: grass}


def _traversable_types(env_ease):
    """Int values of the cell types with a non-zero traversability ease."""
    return [ENV_CELL_TYPES[name] for name, ease in env_ease.items() if ease != 0]



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
        self.env_ease = _ease(1, 0, 0, 0.4, 0, 0.6)
        self.traversable_types = _traversable_types(self.env_ease)

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
        if graph_mode:
            from .Graph import Graph
            self.belief_space["graph"] = Graph(self.belief_space["occupancy_grid"], agent_id = self.robot_id, traversable_types=self.traversable_types+[-1], nodes_distance_treshold=self.vision_range/2)


        self.last_given_position = (int(self.transform.x), int(self.transform.y))
        self._neighbors_cache = None
        
        self.imdone = False #if true, the robot will consider it's mission is over, it stops its activity.

        self.logging = write_logs
        self.logs = {}

        self.com_importance_mode = "default" #ComImportance
        self.cost_calculation_mode = "euclidian"
        
        #Ready!
        self.status = "ready"
        self.is_active = True

    def update(self):
        """
        Update function of the agent, will be called at each simulation step.
        """
        self.sense()#first of all sense the env.

        if self.graph_mode:
            if self.env.step - self.last_graph_generation > self.graph_delta:
                self.belief_space["graph"].update_graph(self.belief_space["occupancy_grid"], agent_id=self.robot_id, traversable_types=self.traversable_types)
                self.last_graph_generation = self.env.step

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
            self.belief_space["robot_informations"][self.robot_id].update({ "position":(self.transform.x, self.transform.y), "competences":self.competences, "env_ease":self.env_ease, "traversable_types":self.traversable_types, "status": self.status, "step":self.env.step }) #self beliefs update
            self.belief_space["last_infos_matrix"][self.robot_id][self.robot_id] = self.env.step
                
           
            if self.energy_amount <= 0:
                self.finish()

            if self.status == "destroyed":
                self.finish()
            
            if self.new_communication and self.env.step - self.last_plan_time > self.delta_replan:
                self.target = None
                self.path_to_target = None
                self.action_to_perform = None
                self.new_communication = False
                self.last_plan_time = self.env.step
            
            if self.logging:
                self.write_logs()

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
        raise NotImplementedError(f"The behave method has to be redefined for the agent {self.robot_id} of type {type(self).__name__}")

    def _set_mobility(self, env_ease):
        """Set the traversability ease and the matching traversable cell types, and publish them in the robot's own belief."""
        self.env_ease = env_ease
        self.traversable_types = _traversable_types(env_ease)
        infos = self.belief_space["robot_informations"][self.robot_id]
        infos["env_ease"] = self.env_ease
        infos["traversable_types"] = self.traversable_types

    def _set_behavior(self, behavior_to_use):
        if behavior_to_use not in self.behavior_space:
            raise ValueError(f"{type(self).__name__}: behavior_to_use '{behavior_to_use}' is not in the behavior space, it should be in {self.behavior_space}")
        self.behavior = behavior_to_use

    def navigate(self):
        if self.path_to_target: #If we have a path to our target, we continue this path.
            self.navigate_through_target_path()
        else: #if we don't have any path, then compute it with our target
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
            if cell_type in self.env.present_cell_types:
                collisions = not(int(self.env.real_occupancy_grid[int(self.transform.x)][int(self.transform.y)]) in self.traversable_types)
                if (collisions): #is there collision
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
        if self.env.robot_block:
            agent_collision = spritecollide(self, self.env.agent_group, False)
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
        neighbors = self.get_neighbors_pixels(distance=self.vision_range, stop_at_wall=True, self_inclusion=True)

        idx = np.asarray(neighbors)
        self.belief_space["occupancy_grid"][idx[:, 0], idx[:, 1]] = self.env.real_occupancy_grid[idx[:, 0], idx[:, 1]] #get the real value (simulates sensing, note that we could add noise.)

        #artifact detection
        artifacts = self.env.interest_points["artifacts"]
        if artifacts:
            neighbors_set = set(neighbors)
            known_artifacts = self.belief_space["artifacts"]
            for a in artifacts:
                if a.coordinates in neighbors_set:
                    known = known_artifacts.get(a.id)
                    if known is not None:
                        discovery_time, awared, done_time = known["discovery_time"], known["awared"], known["donetime"]
                    else:
                        discovery_time, awared, done_time = self.env.step, [self.robot_id], None
                    known_artifacts[a.id] = {"name":a.name, "type":a.type, "status":a.status, "coordinates":a.coordinates, "step":self.env.step, "needed_robots":a.needed_robots, "discovery_time": discovery_time, "awared":awared, "donetime":done_time}

    def get_neighbors_pixels(self, distance:int, stop_at_wall = False, self_inclusion = True):
        """
        get neighbors around the agents:\\
        Params:\\
        - distance:int : until what distance cells are considered as neighbors
        - stop at walls:bool (default False), cells behind a wall are considered as neighbors?
        - self_inclusion: bool (default:True), do we include the cell the agent is on?.

        Returns an immutable tuple (the last result is cached, the real grid being static).
        """
        start = (int(self.transform.x), int(self.transform.y))
        key = (start, distance, stop_at_wall, self_inclusion, tuple(self.traversable_types))
        cached = self._neighbors_cache
        if cached is not None and cached[0] == key:
            return cached[1]

        grid = self.env.real_occupancy_grid
        width = self.env.width
        height = self.env.height
        traversable = set(self.traversable_types)
        blocking = set(BLOCKING_SENSOR_TYPES)

        now_queue = [start]
        neighbors = []
        seen = set()
        if self_inclusion:
            neighbors.append(start)
            seen.add(start)

        for _ in range(distance):
            next_queue = []
            for x, y in now_queue:
                for dx, dy in DIRECTIONS:
                    nx = x + dx
                    ny = y + dy
                    neighbor = (nx, ny)
                    if neighbor in seen or not (0 <= nx < width and 0 <= ny < height):
                        continue
                    neighbors.append(neighbor)
                    seen.add(neighbor)
                    if stop_at_wall:
                        value = grid[nx, ny]
                        if value not in traversable and value in blocking:
                            continue
                    next_queue.append(neighbor)
            now_queue = next_queue

        result = tuple(neighbors)
        self._neighbors_cache = (key, result)
        return result
            
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
                            robot.recieve_belief(self._belief_message())
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

    def _belief_message(self):
        """Copy of the part of the belief space read by recieve_belief (the occupancy grid is not copied, the receiver builds a new one)."""
        shared = {key: self.belief_space[key] for key in ("self_id", "robot_informations", "artifacts", "last_infos_matrix")}
        if self.graph_mode:
            shared["graph"] = self.belief_space["graph"]
        message = deepcopy(shared)
        message["occupancy_grid"] = self.belief_space["occupancy_grid"]
        return message

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
        for agent in self.belief_space["last_infos_matrix"].keys() | sender_belief_space["last_infos_matrix"].keys():
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

                #donetime
                if sender_belief_space["artifacts"][artifact]["donetime"] != self.belief_space["artifacts"][artifact]["donetime"]:
                    #merging the robots id of robots awared of the tasks
                    if sender_belief_space["artifacts"][artifact]["donetime"] != None:
                        global_donetime = sender_belief_space["artifacts"][artifact]["donetime"]
                    else: 
                        global_donetime = self.belief_space["artifacts"][artifact]["donetime"]
                    self.belief_space["artifacts"][artifact].update({"donetime": global_donetime})

                
                #update
                self.belief_space["artifacts"].update({artifact: sender_belief_space["artifacts"][artifact]})
                self.belief_space["artifacts"][artifact].update({"discovery_time": discovery_time})
                

            if sender_belief_space["artifacts"][artifact]["awared"] != self.belief_space["artifacts"][artifact]["awared"]:
                #merging the robots id of robots awared of the tasks
                sender_awared = sender_belief_space["artifacts"][artifact]["awared"]
                self_awared = self.belief_space["artifacts"][artifact]["awared"]
                global_awared = list(set(self_awared + sender_awared))

                self.belief_space["artifacts"][artifact].update({"awared": global_awared})
            
            #donetime
            if sender_belief_space["artifacts"][artifact]["donetime"] != self.belief_space["artifacts"][artifact]["donetime"]:
                #merging the robots id of robots awared of the tasks
                if sender_belief_space["artifacts"][artifact]["donetime"] != None:
                    global_donetime = sender_belief_space["artifacts"][artifact]["donetime"]
                else: 
                    global_donetime = self.belief_space["artifacts"][artifact]["donetime"]
                self.belief_space["artifacts"][artifact].update({"donetime": global_donetime})
            
            

        #ARTIFACTS-----------------------------------------------------------

        self.new_communication = True
        # self.target = None
        # self.path_to_target = None

    def move(self, vector_x, vector_y):
        """Method that has to be redefined for each type of robot because they don't have the same movement mechanism."""
        raise NotImplementedError(f"move has to be implemented in {type(self).__name__}")

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
                if a.id == self.action_to_perform["id"] and (euclidian_distance((int(a.coordinates[0]), int(a.coordinates[1])), (int(self.transform.x), int(self.transform.y)) ) < 1.5*a.needed_robots):
                    
                    result = a.interact(self.competences[self.action_to_perform["type"]]["capability"])
                    if result :
                        self.belief_space["artifacts"][self.action_to_perform["id"]]["donetime"] = self.env.step
                        self.belief_space["artifacts"][self.action_to_perform["id"]]["status"] = "done"
                        self.belief_space["artifacts"][self.action_to_perform["id"]]["step"] = self.env.step + 1
                        self.action_to_perform = None
                        self.belief_transfer()
                        return None
                    else:
                        return None
            #si on ne voit pas l'artefact une fois sur place
            if euclidian_distance((int(self.belief_space["artifacts"][self.action_to_perform["id"]]["coordinates"][0]), int(self.belief_space["artifacts"][self.action_to_perform["id"]]["coordinates"][1])), (int(self.transform.x), int(self.transform.y))) <= self.vision_range:
                self.belief_space["artifacts"][self.action_to_perform["id"]]["status"] = "done"
                self.belief_space["artifacts"][self.action_to_perform["id"]]["step"] = self.env.step
                self.action_to_perform = None

    def stay(self):
        self.belief_transfer()

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
        else:
            #Path not accurate.
            random_wiggle.random_wiggle(self) #random move to maybe select another frontier.
            self.path_to_target = None #forget the target and the path
            self.target = None

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
                "last_infos_matrix" : {k: dict(v) for k, v in self.belief_space["last_infos_matrix"].items()},
                "known_environment_portion" : np.count_nonzero(self.belief_space["occupancy_grid"]!=-1)/(self.env.width*self.env.height),
                "target": self.target
            }
        })



class Ground(Robot):
    BEHAVIORS = {
        "random": random_wiggle.random_wiggle,
        "nearest_frontier": nearest_frontier.nearest_frontier,
        "minpos": minpos.minpos,
        "local_frontier": local_frontier.local_frontier,
        "action_selection": action_selection.action_selection,
        "rendezvous": rendezvous.rendezvous,
    }

    def __init__(self, env, robot_id, size = 1, color = (0, 255, 0), init_transform = (0,0,0), max_speed = (1.0,0.0,1.5),vision_range=20, communication_range = 40, communication_period = 10, behavior_to_use = "random", energy_amount = 1000, energy_cost_per_cell = 1, delta_replan=20, write_logs=False, graph_mode=False, graph_delta=50):
        super().__init__(env, robot_id, size, color, init_transform= init_transform, max_speed=max_speed, vision_range=vision_range, communication_range=communication_range, communication_period=communication_period, energy_amount = energy_amount, energy_cost_per_cell = energy_cost_per_cell, delta_replan=delta_replan, write_logs=write_logs, graph_mode=graph_mode, graph_delta=graph_delta)
        self.behavior_space = list(self.BEHAVIORS)
        self._set_mobility(_ease(1, 0, 0, 0.4, 0, 0.6))
        self._set_behavior(behavior_to_use)

    def behave(self):
        if not self.imdone:
            self.BEHAVIORS[self.behavior](self)

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
    BEHAVIORS = {
        "random": random_wiggle.random_wiggle,
        "nearest_frontier": nearest_frontier.nearest_frontier,
        "minpos": minpos.minpos,
        "local_frontier": local_frontier.local_frontier,
        "action_selection": action_selection.action_selection,
    }
    def __init__(self, env, robot_id, size = 1, color = (255, 0, 0), init_transform = (0,0,0), max_speed = (1.0,1.0,1.5),vision_range=20, communication_range = 40, communication_period = 10, behavior_to_use = "random", energy_amount = 1000, energy_cost_per_cell = 1, delta_replan=20, write_logs=False, graph_mode=False, graph_delta=50):
        super().__init__(env, robot_id, size, color, init_transform= init_transform, max_speed=max_speed, vision_range=vision_range, communication_range=communication_range, communication_period=communication_period, energy_amount = energy_amount, energy_cost_per_cell = energy_cost_per_cell, delta_replan=delta_replan, write_logs=write_logs, graph_mode=graph_mode, graph_delta=graph_delta)
        self.behavior_space = list(self.BEHAVIORS)
        self._set_mobility(_ease(1, 1, 0, 1, 1, 1))
        self._set_behavior(behavior_to_use)
    
    def behave(self):
        if not self.imdone:
            self.BEHAVIORS[self.behavior](self)

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

        self._set_mobility(_ease(0, 0, 0, 0, 0, 0))

        self.shape_competence("exploration", capability=0, importance=1) #that robot can't explore, so exp capability is set to zero
        self.shape_competence("communication", capability=0, importance=1) # as it can't move, communication purpose mvt capability is also settled to 0

        self._set_behavior(behavior_to_use)
        self.artifact = None
        self.create_artifact()
        self.shape_competence(self.artifact.type, capability=0, importance=0)
    
    def behave(self):
        if not(self.imdone):
            self.stay()
            if any(r.imdone for r in self.env.agents if r is not self):
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