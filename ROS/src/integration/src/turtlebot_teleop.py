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
import math
import threading


''' Blocks until a key is pressed '''


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


'''
Class that registers the received direction instructions
'''


class Movement:
    conn = -1   # current connection with virtual representation
    id = ''     # virtual representation id
    simulation_step_dur = 0.1   # in seconds

    # linear velocity in meters per second
    speed = rospy.get_param("~speed", 0.5)
    # angular velocity in rad per second
    turn = rospy.get_param("~turn", 1.0)

    # instruction directions
    x = 0   # forward if 1, backward if -1
    y = 0
    z = 0
    th = 0  # clockwise if 1, counterclockwise if -1

    status = 0  # show status if 1

    # last time the instruction direction was updated
    last_updated = datetime.datetime.now()

    sem = threading.Semaphore()  # for synchronization

    def __init__(self, conn):
        self.conn = conn
        start_new_thread(self.check_movement, ())

    '''
    Publishes the new direction instructions
    '''

    def move_update(self, x, y, z, th, from_thread):

        if(not from_thread):
            self.sem.acquire()

        self.x = x
        self.y = y
        self.z = z
        self.th = th

        twist = Twist()
        twist.linear.x = self.x*self.speed
        twist.linear.y = self.y*self.speed
        twist.linear.z = self.z*self.speed
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.th*self.turn
        pub.publish(twist)

        self.last_updated = datetime.datetime.now()

        if(not from_thread):
            self.sem.release()

    def vels(self):
        return "VELOCITIES :\tspeed %s\tturn %s " % (self.speed, self.turn)

    def print(self):
        return " - [%d,%d,%d,%d]" % (self.x, self.y, self.z, self.th)

    '''
    Thread: Check if the movement is withing the simulation step duration
    '''

    def check_movement(self):

        while(1):

            sleep = 0
            self.sem.acquire()

            # movement was incresed
            if [self.x, self.y, self.z, self.th] != [0, 0, 0, 0]:
                elapsed_time = datetime.datetime.now() - self.last_updated

                # movement duration is above 100 ms
                if(elapsed_time.total_seconds() >= self.simulation_step_dur):

                    # stop movement
                    self.move_update(0, 0, 0, 0, True)
                    sleep = 0.1
                # wait until reach the 100 ms (aprox.)
                else:
                    sleep = (self.simulation_step_dur -
                             elapsed_time.total_seconds())

            self.sem.release()
            time.sleep(sleep)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # turtle1/cmd_vel -> turtlesim subscribed
    # change to cmd_vel to control a real turtlebot3
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    rospy.init_node('turtlebot_teleop')

    # create movement register
    move = Movement(client.Connection())

    # start thread to listen to incoming messages
    start_new_thread(receiver.start, (move,))

    # establish connection with sumo
    msg = messages.getIntegrationRequestMsg(
        move.speed, move.turn, receiver.Status.ip, receiver.Status.port)

    # waits for a positive reply to the integration request
    while 1:
        reply = move.conn.sendRequest(msg)
        print("RECEIVED: %s" % (reply))
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
        while(1):
            key = getKey()

            # move key
            if key in resources.moveBindings.keys():
                print("KEY PRESSED: %s" % (key + move.print()))
                msg = messages.getMovementMsg(
                    move.id, resources.movementsCode[key].value)
                move.conn.sendMessage(msg)

            # speed and angular key
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
                move.conn.sendMessage(msg)

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
