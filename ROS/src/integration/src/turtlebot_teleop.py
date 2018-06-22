#!/usr/bin/env python

from __future__ import print_function
from client import Connection
from enum import Enum
import roslib
roslib.load_manifest('integration')
import rospy
from thread import start_new_thread

from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty
import json
import messages
import receiver
intro_msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

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

class MOVEMENT(Enum):
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3
    FORWARD_LEFT = 4
    FORWARD_RIGHT = 5
    BACKWARD_LEFT = 6
    BACKWARD_RIGHT = 7

movementsCode = {
    'i': MOVEMENT.FORWARD,
    'o': MOVEMENT.FORWARD_RIGHT,
    'u': MOVEMENT.FORWARD_LEFT,
    ',': MOVEMENT.BACKWARD,
    '.': MOVEMENT.BACKWARD_RIGHT,
    'm': MOVEMENT.BACKWARD_LEFT,
    'j': MOVEMENT.LEFT,
    'l': MOVEMENT.RIGHT,
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    start_new_thread(receiver.start, ())

    #turtle1/cmd -> turtlesim subscribed
    # alterar para cmd_vel para controlar o verdadeiro
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)


    rospy.init_node('turtlebot_teleop')

    id = ''
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    
    conn = Connection()

    # Establish connection
    msg = messages.getIntegrationRequestMsg(speed,turn, receiver.Status.ip, receiver.Status.port)
    while 1:
        reply = conn.sendRequest(msg)
        print(reply)
        info = json.loads(reply)
        if(info["type"] == messages.MsgType.REPLY_ACCEPTED.value):
            id = info["id"]
            break
        else:
            print("Trying to connect...")

    print("Connection established!")

    try:
        print(intro_msg)
        print(vels(speed, turn))
        while(1):
            key = getKey()

            # move
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

                print(key + " - [%d,%d,%d,%d]" % (x,y,z,th))
                msg = messages.getMovementMsg(id,movementsCode[key].value)
                print(msg)
                reply = conn.sendRequest(msg)

            # speed and angular
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(intro_msg)
                status = (status + 1) % 15

                msg = messages.getSpeedRotationMsg(id,speed,turn)
                print(msg)
                reply = conn.sendRequest(msg)

            else:
                x = 0
                y = 0
                z = 0
                th = 0
                reply = None
                if (key == '\x03'):
                    break
            
            if(reply is not None):
                info = json.loads(reply)
                if(info["type"] == messages.MsgType.REPLY_ACCEPTED.value):
                    twist = Twist()
                    twist.linear.x = x*speed
                    twist.linear.y = y*speed
                    twist.linear.z = z*speed
                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = th*turn
                    pub.publish(twist)

            

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
