from enum import Enum
from datetime import datetime
import traci
import traci.constants as tc
import curses
import json
import math
import numpy as np
import vehicle_controller
import msg_resources



def getAcceptedMessage(id):
    content = {
        "type" : msg_resources.MsgType.REPLY_ACCEPTED.value,
        "id" : id
    }
    msg = json.dumps(content)
    return msg

def getRejectedMessage(id):
    content = {
        "type" : msg_resources.MsgType.REPLY_REJECTED.value,
        "id" : id
    }
    msg = json.dumps(content)
    return msg

def getCalibrationMessage(id,date_sent):
    content = {
        "type" : msg_resources.MsgType.CALIBRATION.value,
        "id" : id,
        "date_sent" : date_sent.strftime('%Y-%m-%d %H:%M:%S.%f'),
        "date_received" : ""
    }
    msg = json.dumps(content)
    return msg

'''
Handle Messages Received
'''

def handleMovementMessage(info, controller):

    move = info["movement"]

    if move == msg_resources.Movement.RIGHT.value:
        controller.setIncrement(0, 1,info)

    elif move == msg_resources.Movement.LEFT.value:
        controller.setIncrement(0, -1,info)

    elif move == msg_resources.Movement.FORWARD.value:
        controller.setIncrement(1, 0,info)

    elif move == msg_resources.Movement.BACKWARD.value:
        controller.setIncrement(-1, 0,info)

    elif move == msg_resources.Movement.FORWARD_LEFT.value:
        controller.setIncrement(1, -1,info)

    elif move == msg_resources.Movement.FORWARD_RIGHT.value:
        controller.setIncrement(1, 1,info)

    elif move == msg_resources.Movement.BACKWARD_LEFT.value:
        controller.setIncrement(-1, 1,info)

    elif move == msg_resources.Movement.BACKWARD_RIGHT.value:
        controller.setIncrement(-1, -1,info)


def handleSpeedRotationMessage(info, controller):

    speed = info["speed"]
    rotation = info["rotation"]

    controller.speed = speed
    controller.angular = rotation


def handleIntegrationRequestMessage(info, controllers,trip,integration_requests):

    speed = info["speed"]
    rotation = info["rotation"]

    ip = info["ip"]
    port = info["port"]

    id = "car_" + `integration_requests`

    controller = vehicle_controller.VehicleController(id,trip)
    controller.speed = speed
    controller.angular = rotation
    controller.ip = ip
    controller.port = port

    controllers.append(controller)


    print("NEW VEHICLE ADDED: " + id)

    return getAcceptedMessage(id)


def handleCalibrationMessage(info, controller):
    date_received = datetime.strptime(info["date_received"],'%Y-%m-%d %H:%M:%S.%f')
    date_sent= datetime.strptime(info["date_sent"],'%Y-%m-%d %H:%M:%S.%f')

    delay = (date_received - date_sent).total_seconds()
    print("delay %f" % (delay))
    controller.delay = delay