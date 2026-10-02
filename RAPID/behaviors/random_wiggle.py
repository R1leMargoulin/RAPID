import random
import numpy as np

def random_wiggle(selfrobot): #could be called wiggle
    selfrobot.speed.x = selfrobot.max_speed.x
    selfrobot.speed.y = selfrobot.max_speed.y

    selfrobot.speed.w = random.uniform(-selfrobot.max_speed.w ,selfrobot.max_speed.w)
    selfrobot.transform.w += selfrobot.speed.w

    selfrobot.transform.w = selfrobot.transform.w%(2*np.pi)

    xmove = selfrobot.speed.x * np.cos(selfrobot.transform.w)
    ymove = selfrobot.speed.x * np.sin(selfrobot.transform.w)

    selfrobot.translate(xmove, ymove)
