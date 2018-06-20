from server import Server, Movement
from enum import Enum
import traci
import traci.constants as tc
import curses
import json

class MsgType(Enum):
    MOVEMENT = 0
    SPEED = 1
    ROTATION = 2
    CALIBRATION = 3
    CONFIG = 4
    INTEGRATION_REQUEST = 5
    UNKNOWN = 6

def moveCar(car_id, x, y, angle):
    keep_route = 2
    edge_id = ''
    step_info = traci.vehicle.getSubscriptionResults(car_id)
    lane = -1
    pos = traci.vehicle.getPosition(car_id)
    old_angle = traci.vehicle.getAngle(car_id)

    traci.vehicle.moveToXY(
        car_id, edge_id, lane, pos[0] + x, pos[1] + y, old_angle + angle, keep_route)
    return

def handleMovementMessage(info):
    move = info["movement"]
    if move == ord('q'):
        return
    elif move == Movement.RIGHT.value:
        moveCar("newVeh", 1, 0, 0)
    elif move == Movement.BACKWARD.value:
        moveCar("newVeh", 0, -1, 0)
    elif move == Movement.LEFT.value:
        moveCar("newVeh", -1, 0, 0)
    elif move == Movement.FORWARD.value:
        moveCar("newVeh", 0, 1, 0)
    elif move == Movement.FORWARD_LEFT.value:
        moveCar("newVeh", -1, 1, 0)
    elif move == Movement.FORWARD_RIGHT.value:
        moveCar("newVeh", 1, 1, 0)
    elif move == Movement.BACKWARD_LEFT.value:
        moveCar("newVeh", -1, -1, 0)
    elif move == Movement.BACKWARD_RIGHT.value:
        moveCar("newVeh", 1, -1, 0)
    else:
        return

    # TODO: return approved or not