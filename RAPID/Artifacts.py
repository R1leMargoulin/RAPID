from pygame.sprite import Sprite, spritecollide, collide_circle
from pygame.transform import scale
from pygame.draw import *
from pygame import Surface, SRCALPHA, Rect


class Artifact:
    def __init__(self, env, id, name, type, coordinates, size = 1, color=(255,0,0)):
        """
        Artifact placed on the environment, that can be detected and that the robots can interact with
        
        :param env: environment obj
        :param id: int ID of the artifact
        :param name: string name of the artifact
        :param type: string type of the artifact (has to be the same type name of the competence given to the robot to interact with it)
        :param coordinates: (float, float): x,y coordinates
        :param size: Int size for the simulation
        :param color: (int, int, int) RGB color for the simulation
        """
        self.env = env

        self.id = id
        self.name = name
        self.type = type
        self.coordinates = coordinates

        self.surf = Surface((2*size, 2*size), SRCALPHA, 32)
        circle(self.surf, color, (size, size), 2*size)
        self.rect = Rect(0, 0, size, size)

        self.rect.centerx = coordinates[0]
        self.rect.centery = coordinates[1]

        self.status = "placed"
    
    def update(self, screen):
        """
        called at each step, essentially for display
        """
        if self.env.render:
            scaled_rect = Rect(self.rect.x * self.env.scaling_factor, self.rect.y * self.env.scaling_factor, self.rect.width * self.env.scaling_factor, self.rect.height * self.env.scaling_factor)
            screen.blit(scale(self.surf, scaled_rect.size), scaled_rect)

    def check_importance(self, agent): #kinda abstract
        """
        The importance relative to an artifact type can be dynamic, in that case this method has to be redefined. If the robot detects one artifact, it will call this method at each step to check the importance it gives to the task associated with the artifact.
        """
        pass

    def destroy(self):
        """destroy the artifact, better be called by the interact method"""
        self.status = "destroyed"
        
    def interact(self, competence):
        """
        The agents will call this method in order to have interaction with artifact.

        :param competence: Competence of the agent relative to the task, it will have an impact on the interaction depending of the artifact type.
        """
        pass