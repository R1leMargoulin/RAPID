from pygame.sprite import Sprite, spritecollide, collide_circle
from pygame.transform import scale
from pygame.draw import *
from pygame import Surface, SRCALPHA, Rect


class Artifact:
    def __init__(self, env, id, name, type, coordinates, size = 1, color=(255,0,0)):

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
        essentially for display
        """
        if self.env.render:
            scaled_rect = Rect(self.rect.x * self.env.scaling_factor, self.rect.y * self.env.scaling_factor, self.rect.width * self.env.scaling_factor, self.rect.height * self.env.scaling_factor)
            screen.blit(scale(self.surf, scaled_rect.size), scaled_rect)

    def destroy(self):
        self.status = "destroyed"
        
    def interact(self, competence):
        pass