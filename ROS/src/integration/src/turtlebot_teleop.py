#!/usr/bin/env python

from __future__ import print_function
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
import resources
import datetime
import client
import time

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Movement:
    id = ''
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    last_updated = -1

    def __init__(self):
        start_new_thread(self.check_movement, ())

    def move_update(self,x, y, z, th):
        self.x = x
        self.y = y
        self.z = z
        self.th = th

    def vels(self):
        return "currently:\tspeed %s\tturn %s " % (self.speed, self.turn)

    def print(self):
        return " - [%d,%d,%d,%d]" % (self.x, self.y, self.z, self.th)

    def move(self):
        twist = Twist()
        twist.linear.x = self.x*self.speed
        twist.linear.y = self.y*self.speed
        twist.linear.z = self.z*self.speed
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.th*self.turn
        pub.publish(twist)
        self.last_updated = datetime.datetime.now()

    def check_movement(self):
        movement_dur = 100000

        while(1):

            if self.last_updated != -1 and [self.x,self.y,self.z,self.th] != [0,0,0,0]:
                elapsed_time = datetime.datetime.now() - self.last_updated

                if(elapsed_time.seconds > 0 or elapsed_time.microseconds >= movement_dur):
                    self.move_update(0,0,0,0)
                    self.move()
                    self.last_updated = -1
                    time.sleep(0.09)
                else:
                    time.sleep((movement_dur-elapsed_time.total_seconds()*1000000)/1000000 * 9/10)

    

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # turtle1/cmd -> turtlesim subscribed
    # alterar para cmd_vel para controlar o verdadeiro
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)

    rospy.init_node('turtlebot_teleop')

    move = Movement()
    start_new_thread(receiver.start, (move,))


    # Establish connection
    conn = client.Connection()

    msg = messages.getIntegrationRequestMsg(
        move.speed, move.turn, receiver.Status.ip, receiver.Status.port)
    while 1:
        reply = conn.sendRequest(msg)
        print(reply)
        info = json.loads(reply)
        if(info["type"] == resources.MsgType.REPLY_ACCEPTED.value):
            move.id = info["id"]
            break
        else:
            print("Trying to connect...")

    print("Connection established!")

    # Reading keys and send them to sumo
    try:
        print(resources.intro_msg)
        print(move.vels())
        while(1):
            key = getKey()

            # move
            if key in resources.moveBindings.keys():
                print(key + move.print())
                msg = messages.getMovementMsg(
                    move.id, resources.movementsCode[key].value)
                conn.sendMessage(msg)

            # speed and angular
            elif key in resources.speedBindings.keys():

                # update speed and turn
                move.speed = move.speed * resources.speedBindings[key][0]
                move.turn = move.turn * resources.speedBindings[key][1]

                # print menu
                print(move.vels())
                if (move.status == 14):
                    print(resources.intro_msg)
                move.status = (move.status + 1) % 15

                msg = messages.getSpeedRotationMsg(
                    move.id, move.speed, move.turn)
                conn.sendMessage(msg)

            else:
                move.move_update(0, 0, 0, 0)
                move.last_updated = -1
                reply = None
                if (key == '\x03'):
                    break

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
