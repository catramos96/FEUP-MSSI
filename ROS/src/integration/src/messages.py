#!/usr/bin/env python

import json
import turtlebot_teleop
from enum import Enum


class MsgType(Enum):
    MOVEMENT = 0
    SPEED = 1
    ROTATION = 2
    CALIBRATION = 3
    CONFIG = 4
    INTEGRATION_REQUEST = 5
    UNKNOWN = 6


'''
CREATING METHODS
'''


def getIntegrationRequestMsg(id,speed,rotation):
    content = {
        "type": MsgType.INTEGRATION_REQUEST.value,
        "id" : id,
        "speed" : speed,
        "rotation":rotation
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


def getSpeedMsg(id,speed):
    content = {
        "type": MsgType.SPEED.value,
        "id" : id,
        "value": speed}
    msg = json.dumps(content)
    return msg


def getRotationMsg(id,rotation):
    content = {
        "type": MsgType.ROTATION.value,
        "id" : id,
        "value": rotation}
    msg = json.dumps(content)
    return msg


def getCalibrationMsg(id,delay):
    content = {
        "type": MsgType.CALIBRATION.value,
        "id" : id,
        "delay": delay}  # add other parameters if needed
    msg = json.dumps(content)
    return msg