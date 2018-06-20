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


def getIntegrationRequestMsg(speed,rotation):
    content = {
        "type": MsgType.INTEGRATION_REQUEST.value,
        "speed" : speed,
        "rotation":rotation
    }
    msg = json.dumps(content)
    return msg


def getMovementMsg(movement):
    content = {
        "type": MsgType.MOVEMENT.value,
        "movement": movement
    }
    msg = json.dumps(content)
    return msg


def getSpeedMsg(speed):
    content = {
        "type": MsgType.SPEED.value,
        "value": speed}
    msg = json.dumps(content)
    return msg


def getRotationMsg(rotation):
    content = {
        "type": MsgType.ROTATION.value,
        "value": rotation}
    msg = json.dumps(content)
    return msg


def getCalibrationMsg(delay):
    content = {
        "type": MsgType.CALIBRATION.value,
        "delay": delay}  # add other parameters if needed
    msg = json.dumps(content)
    return msg