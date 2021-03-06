#!/usr/bin/env python
import json
import turtlebot_teleop
import resources
import datetime

'''
Build and Handle incommed messages
'''


def handleMovementMessage(info, controller):

    move = info["movement"]

    # valid movement
    if move >= 0 and move <= 7:

        # get key related to movement
        key = resources.movementsCodeInverse[move]

        # update current position with direction instruction
        controller.move_update(resources.moveBindings[key][0], resources.moveBindings[key]
                               [1], resources.moveBindings[key][2], resources.moveBindings[key][3], False)
    return


def handleCalibrationMessage(info):

    # set received time
    info["date_received"] = datetime.datetime.now().strftime(
        '%Y-%m-%d %H:%M:%S.%f')
    msg = json.dumps(info)
    
    return msg


def getIntegrationRequestMsg(speed, rotation, ip, port):
    content = {
        "type": resources.MsgType.INTEGRATION_REQUEST.value,
        "speed": speed,
        "rotation": rotation,
        "ip": ip,
        "port": port
    }
    msg = json.dumps(content)
    return msg


def getMovementMsg(id, movement):
    content = {
        "type": resources.MsgType.MOVEMENT.value,
        "id": id,
        "movement": movement
    }
    msg = json.dumps(content)
    return msg


def getSpeedRotationMsg(id, speed, rotation):
    content = {
        "type": resources.MsgType.SPEED_ROTATION.value,
        "id": id,
        "speed": speed,
        "rotation": rotation
    }
    msg = json.dumps(content)
    return msg


def getCalibrationMsg(id, delay):
    content = {
        "type": resources.MsgType.CALIBRATION.value,
        "id": id,
        "delay": delay}  
    msg = json.dumps(content)
    return msg
