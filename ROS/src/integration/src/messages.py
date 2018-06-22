#!/usr/bin/env python

import json
import turtlebot_teleop
from enum import Enum


class MsgType(Enum):
    MOVEMENT = 0
    SPEED_ROTATION = 1
    CALIBRATION = 2
    CONFIG = 3
    INTEGRATION_REQUEST = 4
    UNKNOWN = 5
    REPLY_ACCEPTED = 6
    REPLY_REJECTED = 7

'''
CREATING METHODS
'''


def getIntegrationRequestMsg(speed,rotation, ip, port):
    content = {
        "type": MsgType.INTEGRATION_REQUEST.value,
        "speed" : speed,
        "rotation":rotation,
        "ip": ip,
        "port": port
    }
    msg = json.dumps(content)
    return msg


def getMovementMsg(id,movement):
    content = {
        "type": MsgType.MOVEMENT.value,
        "id" : id,
        "movement": movement
    }
    msg = json.dumps(content)
    return msg


def getSpeedRotationMsg(id,speed,rotation):
    content = {
        "type": MsgType.SPEED_ROTATION.value,
        "id" : id,
        "speed": speed,
        "rotation": rotation
    }
    msg = json.dumps(content)
    return msg


def getCalibrationMsg(id,delay):
    content = {
        "type": MsgType.CALIBRATION.value,
        "id" : id,
        "delay": delay}  # add other parameters if needed
    msg = json.dumps(content)
    return msg
