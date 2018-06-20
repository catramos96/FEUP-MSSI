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
    SPEED = 1
    ROTATION = 2
    CALIBRATION = 3
    CONFIG = 4
    INTEGRATION_REQUEST = 5
    UNKNOWN = 6

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

    '''
    elif move == Movement.FORWARD_LEFT.value:
        traci.vehicle.moveToXY(
            car_id, edge_id, lane, pos[0] + x, pos[1] + y, old_angle + angular, keep_route)

    elif move == Movement.FORWARD_RIGHT.value:
        traci.vehicle.moveToXY(
            car_id, edge_id, lane, pos[0] + x, pos[1] + y, old_angle + angle, keep_route)

    elif move == Movement.BACKWARD_LEFT.value:
        traci.vehicle.moveToXY(
            car_id, edge_id, lane, pos[0] + x, pos[1] + y, old_angle + angle, keep_route)
    elif move == Movement.BACKWARD_RIGHT.value:
        traci.vehicle.moveToXY(
            car_id, edge_id, lane, pos[0] + x, pos[1] + y, old_angle + angle, keep_route)
    
    else:
        return
    '''

    # TODO: return approved or not
