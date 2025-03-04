from typing import NamedTuple
from pygame.sprite import Sprite



class Transform2d():
        """
        2D transform regrouping :
        - x : position of the entity on the x axis
        - y : position of the entity on the y axis
        - w : yaw -> rotation of the entity on the z axis
        """
        def __init__(self, x:float = 0.0, y:float = 0.0, w:float = 0.0):
                self.x = x
                self.y = y
                self.w = w
            


