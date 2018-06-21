from server import Server, Movement
from enum import Enum
import traci
import traci.constants as tc
import curses
import json
import math
import numpy as np


class MsgType(Enum):
    MOVEMENT = 0
    SPEED_ROTATION = 1
    CALIBRATION = 2
    CONFIG = 3
    INTEGRATION_REQUEST = 4
    UNKNOWN = 5

'''
Handle Messages Received
'''

def handleMovementMessage(info, controller):

    move = info["movement"]

    if move == Movement.RIGHT.value:
        controller.setIncrement(0,1)

    elif move == Movement.LEFT.value:
        controller.setIncrement(0,-1)
    
    elif move == Movement.FORWARD.value:
        controller.setIncrement(1,0)

    elif move == Movement.BACKWARD.value:
        controller.setIncrement(-1,0)

    elif move == Movement.FORWARD_LEFT.value:
        controller.setIncrement(1,-1)

    elif move == Movement.FORWARD_RIGHT.value:
        controller.setIncrement(1,1)

    elif move == Movement.BACKWARD_LEFT.value:
        controller.setIncrement(-1,1)

    elif move == Movement.BACKWARD_RIGHT.value:
        controller.setIncrement(-1,-1)  

    return "ok"

def handleSpeedRotationMessage(info,controller):

    speed = info["speed"]
    rotation = info["rotation"]

    controller.speed = speed
    controller.angular = rotation

    return "ok"
