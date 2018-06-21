from server import Server, Movement
from enum import Enum
import traci
import traci.constants as tc
import curses
import json
import math
import numpy as np
import vehicle_controller


class MsgType(Enum):
    MOVEMENT = 0
    SPEED_ROTATION = 1
    CALIBRATION = 2
    CONFIG = 3
    INTEGRATION_REQUEST = 4
    UNKNOWN = 5
    REPLY_ACCEPTED = 6
    REPLY_REJECTED = 7

def getAcceptedMessage(id):
    content = {
        "type" : MsgType.REPLY_ACCEPTED.value,
        "id" : id
    }
    msg = json.dumps(content)
    return msg

def getRejectedMessage(id):
    content = {
        "type" : MsgType.REPLY_REJECTED.value,
        "id" : id
    }
    msg = json.dumps(content)
    return msg

'''
Handle Messages Received
'''

def handleMovementMessage(info, controller):

    move = info["movement"]

    if move == Movement.RIGHT.value:
        controller.setIncrement(0, 1)

    elif move == Movement.LEFT.value:
        controller.setIncrement(0, -1)

    elif move == Movement.FORWARD.value:
        controller.setIncrement(1, 0)

    elif move == Movement.BACKWARD.value:
        controller.setIncrement(-1, 0)

    elif move == Movement.FORWARD_LEFT.value:
        controller.setIncrement(1, -1)

    elif move == Movement.FORWARD_RIGHT.value:
        controller.setIncrement(1, 1)

    elif move == Movement.BACKWARD_LEFT.value:
        controller.setIncrement(-1, 1)

    elif move == Movement.BACKWARD_RIGHT.value:
        controller.setIncrement(-1, -1)

    return getAcceptedMessage(controller.car_id)


def handleSpeedRotationMessage(info, controller):

    speed = info["speed"]
    rotation = info["rotation"]

    controller.speed = speed
    controller.angular = rotation

    return getAcceptedMessage(controller.car_id)


def handleIntegrationRequestMessage(info, controllers,trip,integration_requests):

    speed = info["speed"]
    rotation = info["rotation"]

    id = "car_" + `integration_requests`

    controller = vehicle_controller.VehicleController(id,trip)
    controller.speed = speed
    controller.angular = rotation

    controllers.append(controller)


    print("NEW VEHICLE ADDED: " + id)

    return getAcceptedMessage(id)
