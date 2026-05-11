from ..utils import *

import numpy as np

def random_wiggle(selfrobot): #could be called wiggle
    #set srobot speed at it's max speed
    selfrobot.speed.x = selfrobot.max_speed.x
    selfrobot.speed.y = selfrobot.max_speed.y

    #random rotation
    selfrobot.speed.w = random.uniform(-selfrobot.max_speed.w ,selfrobot.max_speed.w)
    selfrobot.transform.w += selfrobot.speed.w

    #2pi modulo
    selfrobot.transform.w = selfrobot.transform.w%(2*np.pi)


    #calculation of the x and y movement depending of the x direction speed and the w orientation.
    xmove = selfrobot.speed.x * np.cos(selfrobot.transform.w)
    ymove = selfrobot.speed.x * np.sin(selfrobot.transform.w)

    selfrobot.translate(xmove, ymove)