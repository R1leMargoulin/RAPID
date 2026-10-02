import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
from pygame.sprite import Sprite
from pygame.locals import (K_ESCAPE, KEYDOWN)

from .utils import *
from .grid_variables import *
from .Artifacts import *

import random
import numpy as np
from PIL import Image
from random import uniform, randrange
import logging
import time

COMMUNICATION_MODE_LIST = ["blackboard", "limited"]


class Environment():
    def __init__(self, render = True, width:int=100, height:int=100, background_color = (200,200,200), caption = f'RAPID', env_image:Image.Image = None, full_knowledge:bool=True, robot_block = True, limit_of_steps=None, scaling_factor:int=1, communication_mode="blackboard", communication_reliability = 1, save_img_steps = None, verbose = True):
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
        - communication_reliability: float in [0,1] = Probability for the agents to be communication neighbors when they are in communication range.
        - save_img_steps: String = if not None, image of the simulation will be saved in the string path given
        - verbose:bool = print the step counter during the simulation.
        """
        self.render = render
        self.verbose = verbose

        pygame.init()


        self.scaling_factor = scaling_factor
        self.clock = pygame.time.Clock()
        self.start_time = 0

        self.width = width
        self.height = height
        self.background_color = background_color

        self.agents = [] #List of agents that are in the env, supposed to be a list of RAPID.Agents.Robot objects
        self.cell_feature_groups = {}
        self.present_cell_types = set()
        self.interest_points = {"artifacts":[]}
        self.artifacts_by_id = {}
        self.agents_tools = {}

        self.full_knowledge= full_knowledge
        self.robot_block = robot_block

        self.limit_of_steps = limit_of_steps

        self.communication_mode = communication_mode
        self.communication_reliability = communication_reliability

        self.agent_group = pygame.sprite.Group()

        self.save_img_steps = save_img_steps

        if(env_image):
            self.width = env_image.size[0]
            self.height = env_image.size[1]
            self.real_occupancy_grid = np.zeros((self.width, self.height))
            self.env_image = env_image
            
            self.create_env_from_image(env_image)
        else:
            self.real_occupancy_grid = np.zeros((self.width, self.height))
            if self.render:
                #self.screen = pygame.display.set_mode((self.width, self.height)) #BACKUP scaling
                self.screen = pygame.display.set_mode((self.width * self.scaling_factor, self.height * self.scaling_factor))
            else:
                self.screen = None


        # handling of the communication mode string
        if self.communication_mode not in COMMUNICATION_MODE_LIST:
            raise ValueError(f"unknown communication mode '{self.communication_mode}', available modes : {COMMUNICATION_MODE_LIST}")

        if self.render:
            pygame.display.set_caption(caption)

        self.step = 0
        self.running = True

    def run(self):
        """Runs the simulation"""
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
            
            if self.verbose:
                print(f"step : {self.step}", end="\r")
        


        pygame.quit()

    def update(self):
        """
        Handle every update method of agents and artifacts.
        """
        if self.save_img_steps != None and self.render:
            pygame.image.save(self.screen, self.save_img_steps+str(self.step)+".png")
            
        if self.render:
            self.screen.fill(self.background_color)

            for group in self.cell_feature_groups:
                for o in self.cell_feature_groups[group]:
                    scaled_rect = pygame.Rect(o.rect.x * self.scaling_factor, o.rect.y * self.scaling_factor, o.rect.width * self.scaling_factor, o.rect.height * self.scaling_factor)
                    self.screen.blit(pygame.transform.scale(o.image, scaled_rect.size), scaled_rect)

        artifacts = self.interest_points["artifacts"]
        for a in artifacts:
            if a.status != "destroyed":
                a.update(self.screen)
        artifacts[:] = [a for a in artifacts if a.status != "destroyed"]
        self.artifacts_by_id = {a.id: a for a in artifacts}

        for a in self.agents: 
            a.update()

        if self.render:
            for a in self.agents: 
                a.render(self.screen)

        if(self.end_condition()):
            print(f"Simulation done in {self.step} steps! \n Goal Reached : {self.goal_condition()}")
            self.running = False

        if self.communication_mode == "limited":
            self.limited_communication_update()
        
    def register_artifact(self, artifact):
        self.interest_points["artifacts"].append(artifact)
        self.artifacts_by_id[artifact.id] = artifact

    def add_agent(self, agent):
        """
        Add a new agent in the environment. An agent HAS to be added to be taken into account in the simulation.
        """
        self.agents.append(agent)
        self.agent_group.add(self.agents[-1])

    def create_env_from_image(self, img):
        """
        Will use an image to create the environment and it's obstacles.
        
        :param img: PIL loaded image in RGB format
        """
        np_img = np.array(img)

        dims = np_img.shape
        self.width = dims[1]
        self.height = dims[0]

        if self.render:
            self.screen = pygame.display.set_mode((self.width * self.scaling_factor, self.height * self.scaling_factor))
        else:
            self.screen = None
        r, g, b = np_img[..., 0], np_img[..., 1], np_img[..., 2]
        cell_kinds = (
            (r < 10) & (g < 10) & (b < 10),
            (r > 180) & (g < 100) & (b < 100),
            (r > 180) & (g > 180) & (b < 100),
            (r < 100) & (g < 100) & (b > 180),
            (r < 100) & (g > 180) & (b < 100),
        )
        kind = np.zeros(r.shape, dtype=np.int8)
        for k in range(len(cell_kinds) - 1, -1, -1):
            kind[cell_kinds[k]] = k + 1
        for l, o in np.argwhere(kind > 0):
            color = (r[l, o], g[l, o], b[l, o])
            match kind[l, o]:
                case 1:
                    self.create_cell(o,l, type=OG_WALL, group_name=OG_WALL_GROUP_NAME, color=color)
                case 2:
                    self.create_cell(o,l, type=OG_HIGH_WALL, group_name=OG_HIGH_WALL_GROUP_NAME, color=color)
                case 3:
                    self.create_cell(o,l, type=OG_SAND, group_name=OG_SAND_GROUP_NAME, color=color, visibility=0.5)
                case 4:
                    self.create_cell(o,l, type=OG_WATER, group_name=OG_WATER_GROUP_NAME, color=color, visibility=0.5)
                case 5:
                    self.create_cell(o,l, type=OG_GRASS, group_name=OG_GRASS_GROUP_NAME, color=color, visibility=0.5)

    def create_cell(self, coord_x, coord_y, type, group_name:str, color, visibility = 1):
        """
        create cells in the env with a proper sprite. 
        it will create a proper sprite object and add it in the proper group. In addition, the object will be added to the 
        """
        byte_visibility = int(visibility * 255)
        self.real_occupancy_grid[coord_x][coord_y] = type
        self.present_cell_types.add(group_name)

        sprite = pygame.sprite.Sprite()
        sprite.image = pygame.Surface((1, 1), pygame.SRCALPHA)
        sprite.image.fill((color[0], color[1], color[2], byte_visibility))
        sprite.rect = pygame.Rect(coord_x,coord_y, 1,1)

        self.cell_feature_groups.setdefault(group_name, pygame.sprite.Group()).add(sprite)
    
    def goal_condition(self):
        """
        Depends of the environment type, will return if the goal of the environment is reached or not.
        """
        return False
    
    def end_condition(self):
        """
        Will return True if the simulation is considered as finished. The simulation will then stop at the next step update.
        """
        return all(a.imdone for a in self.agents)
    
    def limited_communication_update(self):
        """
        Available only with "limited" communication mode. Will handle the communication links between agents.
        """
        for a in self.agents:
            a.connected_robots = []

        reach = self._communication_reach()
        for i, j in np.argwhere(np.triu(reach & reach.T, 1)): #one draw per reciprocal pair
            if not random.uniform(0,1) > self.communication_reliability:
                self.agents[i].connected_robots.append(self.agents[j])
                self.agents[j].connected_robots.append(self.agents[i])

        if self.render:
            for a in self.agents:
                for cr in a.connected_robots:
                    pygame.draw.line(self.screen, (255, 255, 255), (a.transform.x * self.scaling_factor, a.transform.y * self.scaling_factor), (cr.transform.x * self.scaling_factor, cr.transform.y * self.scaling_factor))

    def _communication_reach(self):
        """reach[i, j] is True when agent j is within the communication range of agent i (distance between the cell centres)."""
        if not self.agents:
            return np.zeros((0, 0), dtype=bool)
        halo_centers = np.array([a.communication_halo.rect.center for a in self.agents])
        agent_centers = np.array([a.rect.center for a in self.agents])
        ranges = np.array([a.communication_range for a in self.agents])
        distances_squared = ((halo_centers[:, None, :] - agent_centers[None, :, :]) ** 2).sum(axis=2)
        return distances_squared <= ranges[:, None] ** 2

    def break_robot(self, robot_id):
        next(a for a in self.agents if a.robot_id == robot_id).status = "destroyed"
    

class TargetPointEnvironment(Environment):
    def __init__(self, render = True, width = 100, height = 100, background_color=(200, 200, 200), caption=f'simulation_target_point', env_image = None, limit_of_steps = None, scaling_factor:int=1, communication_mode="blackboard", target_point:tuple[int,int]=None, amount_of_agents_goal=1, save_img_steps = None, verbose = True, end_at_full_exploation=True, full_knowledge=True, robot_block=True, communication_reliability=1):
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
        - end_at_full_exploation:bool (default True) : if False, the simulation ends when all robots are done instead of when the target is reached.
        - full_knowledge, robot_block, communication_reliability : see Environment.
        """
        self.end_at_full_exploation = end_at_full_exploation

        super().__init__(render, width, height, background_color, caption, env_image, full_knowledge=full_knowledge, robot_block=robot_block, limit_of_steps=limit_of_steps, scaling_factor=scaling_factor, communication_mode=communication_mode, communication_reliability=communication_reliability, save_img_steps=save_img_steps, verbose=verbose)
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
        """
        defines a target point that has to be reached by the robots.
        """
        sprite = Sprite()
        sprite.image = pygame.Surface((4, 4))
        sprite.image.fill((255, 0, 0))
        sprite.rect = pygame.Rect(x, y, 4, 4)
        self.target_point = sprite


        while self._target_overlaps_wall():
            logging.warning("Target point overlaps with an obstacle, reallocating it randomly.")
            self.target_point.rect.center = (randrange(0, self.width), randrange(0, self.height))
        
        self.real_occupancy_grid[self.target_point.rect.centerx][self.target_point.rect.centery] = OG_TARGET_POINT

    def _target_overlaps_wall(self):
        rect = self.target_point.rect
        x0, y0 = max(rect.x, 0), max(rect.y, 0)
        return bool(np.any(self.real_occupancy_grid[x0:rect.right, y0:rect.bottom] == OG_WALL))

    def goal_condition(self):
        if len(pygame.sprite.spritecollide(self.target_point, self.agent_group, False)) >= self.amount_of_agent_goal:
            logging.info (f"Goal reached at time : {round(time.time() - self.start_time, 2)}")
            return True
        else:
            return False
        
    def end_condition(self):
        if self.end_at_full_exploation:
           return self.goal_condition()
        else:
            return all(a.imdone for a in self.agents)


class FogEnvironment(Environment):
    """
    Base class of the environments where the agents progressively uncover an exploration map.\\
    Subclasses define goal_condition(); the simulation ends on the goal if end_at_goal is True, otherwise when all robots are done.
    """
    def __init__(self, render = True, width = 100, height = 100, background_color=(200, 200, 200), caption=f'simulation', env_image = None, full_knowledge = False, robot_block=True, limit_of_steps=None, scaling_factor:int=1, communication_mode="blackboard", communication_reliability = 1, end_at_goal = True, save_img_steps = None, verbose = True):
        super().__init__(render, width, height, background_color, caption, env_image, full_knowledge, robot_block, limit_of_steps, scaling_factor, communication_mode=communication_mode, communication_reliability=communication_reliability, save_img_steps=save_img_steps, verbose=verbose)
        self.end_at_goal = end_at_goal

        self.interest_points["exploration_map"] = np.zeros((self.width, self.height))
        #on va pas mettre de fog sur les murs parce que la vision ne les traverse pas, si on a des murs plus épais que 2, alors il y aura toujours de la fog.
        self.interest_points["exploration_map"] += self.real_occupancy_grid

        self.explorable_zone_types = [OG_FREE_CELL, OG_GRASS, OG_SAND, OG_WATER]

        self.fog_texture = pygame.Surface((1,1), pygame.SRCALPHA)
        self.fog_texture.fill((100, 100, 100, 150))

        self.explorable_cell_number = np.count_nonzero(np.isin(self.real_occupancy_grid, self.explorable_zone_types))

    @property
    def end_at_full_clear(self):
        return self.end_at_goal

    @end_at_full_clear.setter
    def end_at_full_clear(self, value):
        self.end_at_goal = value

    def run(self):
        self._clear_fog_around_agents()
        super().run()

    def update(self):
        super().update()
        self._clear_fog_around_agents()
        if self.render:
            self.draw_fog()

    def _clear_fog_around_agents(self):
        for agent in self.agents:
            neighbours = agent.get_neighbors_pixels(distance = agent.vision_range, stop_at_wall = True, self_inclusion = True)
            self.mark_explored_cells(neighbours)

    def draw_fog(self):
        """
        Draw fog only if the environment is rendered
        """
        unexplored_poses = np.where(np.isin(self.interest_points["exploration_map"], self.explorable_zone_types))#check for each element of the Occ grid if it's an explorable zone.
        for i in range(len(unexplored_poses[0])):
            scaled_rect = pygame.Rect(unexplored_poses[0][i] * self.scaling_factor, unexplored_poses[1][i] * self.scaling_factor, self.scaling_factor, self.scaling_factor)
            self.screen.blit(pygame.transform.scale(self.fog_texture, scaled_rect.size), scaled_rect)

    def goal_condition(self):
        """
        True when every artifact has been handled.
        """
        return len(self.interest_points["artifacts"]) == 0

    def end_condition(self):
        if self.end_at_goal:
            return self.goal_condition()
        return all(a.imdone for a in self.agents)

    def mark_explored_cells(self, cells):
        """
        update the globally seen cells.
        """
        if len(cells) == 0:
            return
        idx = np.asarray(cells)
        self.interest_points["exploration_map"][idx[:, 0], idx[:, 1]] = 1

    def _add_artifact(self, artifact_class, name_prefix, type, coords, **kwargs):
        artifact_id = len(self.interest_points["artifacts"])
        artifact = artifact_class(self, id=artifact_id, name=f"{name_prefix}{artifact_id}", type=type, coordinates=coords, **kwargs)
        self.register_artifact(artifact)


class ExplorationEnvironment(FogEnvironment):
    def __init__(self, render = True, width = 100, height = 100, background_color=(200, 200, 200), caption=f'simulation', env_image = None, full_knowledge = False, robot_block=True, limit_of_steps=None, scaling_factor:int=1, communication_mode="blackboard", communication_reliability = 1, exploration_proportion_goal=0.995, end_at_full_exploation=True, save_img_steps = None, verbose = True):
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
        - communication_reliability: float in [0,1] = Probability for the agents to be communication neighbors when they are in communication range.
        - save_img_steps: String = if not None, image of the simulation will be saved in the string path given
        - end_at_full_exploation:bool(Default True) = if False, the simulation ends when all robots are in the "done" (imdone) state, otherwise, ends when the exploration proportion goal is reached.
        - exploration_proportion_goal : float in [0,1] = if at least this proportion of the environment is explored, the goal condition of the env will be true.
        - verbose:bool = print the step counter during the simulation.
        """
        super().__init__(render, width, height, background_color, caption, env_image, full_knowledge, robot_block, limit_of_steps, scaling_factor, communication_mode=communication_mode, communication_reliability=communication_reliability, end_at_goal=end_at_full_exploation, save_img_steps=save_img_steps, verbose=verbose)

        self.exploration_proportion_goal = exploration_proportion_goal
        self.exploration_completion = 0.0

        self.explorable_cell_number = self.width* self.height

    @property
    def end_at_full_exploation(self):
        return self.end_at_goal

    @end_at_full_exploation.setter
    def end_at_full_exploation(self, value):
        self.end_at_goal = value

    def goal_condition(self):
        self.exploration_completion = np.count_nonzero(self.interest_points["exploration_map"])/self.explorable_cell_number
        return self.exploration_completion >= self.exploration_proportion_goal


class MineClearingEnvironment(FogEnvironment):
    class Mine(Artifact):
        def __init__(self, env, id, name, type, coordinates, explosion_proba=0.01, size=1, color = (255,0,0)):
            super().__init__(env, id, name, type, coordinates, size, color)
            self.explosion_proba = explosion_proba
            self.life_points=100

        def interact(self, competence):
            """
            competence should be a float in [0,1]

            return dict: {"cleared":bool, "explosion":bool}
            """
            self.life_points -= 10*competence

            explosion = uniform(0,1) <=  self.explosion_proba #probability that the mine exploses

            if explosion:
                self.destroy() ##TODO REMAKE THIS
                for robot in self.env.agents:
                    if euclidian_distance(self.coordinates, (robot.transform.x, robot.transform.y)) < 2.0:
                        robot.status = "destroyed"
                self.destroy()

            if self.life_points <=0:
                self.destroy()
                return True
            else:
                return False


    def __init__(self, render = True, width = 100, height = 100, background_color=(200, 200, 200), caption=f'simulation', env_image = None, full_knowledge = False, robot_block=True, limit_of_steps=None, scaling_factor:int=1, communication_mode="blackboard", communication_reliability = 1, end_at_full_clear = True, fog = True, save_img_steps = None, verbose = True):
        """
        MineClearingEnvironment Class : the agents have to clear every mine of the environment (see ExplorationEnvironment for the common params).\\
        Params specific to this class :
        - end_at_full_clear:bool(Default True) = if False, the simulation ends when all robots are in the "done" (imdone) state, otherwise, ends when every mine is cleared.
        - fog:bool = currently ignored.
        """
        super().__init__(render, width, height, background_color, caption, env_image, full_knowledge, robot_block, limit_of_steps, scaling_factor, communication_mode=communication_mode, communication_reliability=communication_reliability, end_at_goal=end_at_full_clear, save_img_steps=save_img_steps, verbose=verbose)

    def add_mine(self, coords):
        self._add_artifact(self.Mine, "mine", "mine", coords)

    def add_agent(self, agent):
        agent.shape_competence("mine", 0.9, 1.0) #adding default mine competence values
        return super().add_agent(agent)


class WasteCleaningEnvironment(FogEnvironment):
    class Waste(Artifact):
        def __init__(self, env, id, name, type, coordinates, size=1, color = (255,0,0)):
            super().__init__(env, id, name, type, coordinates, size, color)
            self.life_points=100

        def interact(self, competence):
            """
            competence should be a float in [0,1]

            return dict: {"cleared":bool, "explosion":bool}
            """
            self.life_points -= 10*competence

            if self.life_points <=0:
                self.destroy()
                return True
            else:
                return False


    def __init__(self, render = True, width = 100, height = 100, background_color=(200, 200, 200), caption=f'simulation', env_image = None, full_knowledge = False, robot_block=True, limit_of_steps=None, scaling_factor:int=1, communication_mode="blackboard", communication_reliability = 1, end_at_full_clear = True, fog = True, save_img_steps = None, verbose = True):
        """
        WasteCleaningEnvironment Class : the agents have to clean every waste of the environment (see ExplorationEnvironment for the common params).\\
        Params specific to this class :
        - end_at_full_clear:bool(Default True) = if False, the simulation ends when all robots are in the "done" (imdone) state, otherwise, ends when every waste is cleaned.
        - fog:bool = currently ignored.
        """
        super().__init__(render, width, height, background_color, caption, env_image, full_knowledge, robot_block, limit_of_steps, scaling_factor, communication_mode=communication_mode, communication_reliability=communication_reliability, end_at_goal=end_at_full_clear, save_img_steps=save_img_steps, verbose=verbose)

    def add_waste(self, coords):
        self._add_artifact(self.Waste, "waste", "clean", coords)

    def add_agent(self, agent):
        agent.shape_competence("clean", 0.9, 1.0) #adding default mine competence values
        return super().add_agent(agent)


class MultiRobotTasksEnvironment(FogEnvironment):
    class MultiRobotArtifact(Artifact):
        def __init__(self, env, id, name, type, coordinates, size=1, color = (255,0,0), needed_robots=2):
            super().__init__(env, id, name, type, coordinates, size, color, needed_robots)
            self.life_points = 100
            self.interacted = []

        def interact(self, competence):
            self.interacted.append(competence)
            if self.life_points <=0:
                self.destroy()
                return True
            else:
                return False

        def update(self, screen):
            if len(self.interacted) >= self.needed_robots:
                self.life_points = self.life_points - (10*np.sum(self.interacted))
            self.interacted = []
            return super().update(screen)


    def __init__(self, render = True, width = 100, height = 100, background_color=(200, 200, 200), caption=f'simulation', env_image = None, full_knowledge = False, robot_block=True, limit_of_steps=None, scaling_factor:int=1, communication_mode="blackboard", communication_reliability = 1, end_at_full_clear = True, fog = True, save_img_steps = None, verbose = True):
        """
        MultiRobotTasksEnvironment Class : the agents have to complete tasks that need several robots at once (see ExplorationEnvironment for the common params).\\
        Params specific to this class :
        - end_at_full_clear:bool(Default True) = if False, the simulation ends when all robots are in the "done" (imdone) state, otherwise, ends when every task is done.
        - fog:bool = currently ignored.
        """
        super().__init__(render, width, height, background_color, caption, env_image, full_knowledge, robot_block, limit_of_steps, scaling_factor, communication_mode=communication_mode, communication_reliability=communication_reliability, end_at_goal=end_at_full_clear, save_img_steps=save_img_steps, verbose=verbose)

    def add_artifact(self, coords, needed_robots=2):
        self._add_artifact(self.MultiRobotArtifact, "multi", "multi_robot_task", coords, needed_robots=needed_robots)

    def add_agent(self, agent):
        agent.shape_competence("multi_robot_task", 1.0, 1.5) #adding default mine competence values
        return super().add_agent(agent)
