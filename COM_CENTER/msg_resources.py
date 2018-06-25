#!/usr/bin/env python
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

class Movement(Enum):
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3
    FORWARD_LEFT = 4
    FORWARD_RIGHT = 5
    BACKWARD_LEFT = 6
    BACKWARD_RIGHT = 7
    IDLE = 8