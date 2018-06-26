#!/usr/bin/env python
from enum import Enum

intro_msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


class Movement(Enum):
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3
    FORWARD_LEFT = 4
    FORWARD_RIGHT = 5
    BACKWARD_LEFT = 6
    BACKWARD_RIGHT = 7


class MsgType(Enum):
    MOVEMENT = 0
    SPEED_ROTATION = 1
    CALIBRATION = 2
    CONFIG = 3
    INTEGRATION_REQUEST = 4
    UNKNOWN = 5
    REPLY_ACCEPTED = 6
    REPLY_REJECTED = 7


movementsCode = {
    'i': Movement.FORWARD,
    'o': Movement.FORWARD_RIGHT,
    'u': Movement.FORWARD_LEFT,
    ',': Movement.BACKWARD,
    '.': Movement.BACKWARD_RIGHT,
    'm': Movement.BACKWARD_LEFT,
    'j': Movement.LEFT,
    'l': Movement.RIGHT,
}

movementsCodeInverse = {
    Movement.FORWARD.value: 'i',
    Movement.FORWARD_RIGHT.value: 'o',
    Movement.FORWARD_LEFT.value: 'u',
    Movement.BACKWARD.value: ',',
    Movement.BACKWARD_RIGHT.value: '.',
    Movement.BACKWARD_LEFT.value: 'm',
    Movement.LEFT.value: 'j',
    Movement.RIGHT.value: 'l',
}
