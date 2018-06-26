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
import resources
import datetime
import client
import time
import math
import threading
import receiver

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

    def __init__(self, conn,speed,turn,step_dur):
        self.conn = conn
        self.speed = speed
        self.turn = turn
        self.simulation_step_dur = step_dur
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

    if(len(sys.argv) != 9):
        print("Wrong number of arguments.\n 'python3 turtlebot_teleop.py [sumo_ip] [sumo_port] [robot_ip] [robot_port] [speed] [turn] [use_simulator] [step_dur_seconds]")
        exit

    sumo_ip = sys.argv[1]
    sumo_port = int(sys.argv[2])
    robot_ip = sys.argv[3]
    robot_port = int(sys.argv[4])
    speed = float(sys.argv[5])
    turn = float(sys.argv[6])
    use_simulator = int(sys.argv[7])
    step_dur = float(sys.argv[8])

    print("SUMO_IP: %s" % (sumo_ip))
    print("SUMO_PORT: %d" % (sumo_port))
    print("ROBOT_IP: %s" % (robot_ip))
    print("ROBOT_PORT: %d" % (robot_port))
    print("SPEED: %f" % (speed))
    print("TURN: %f" % (turn))
    print("USE_SIMULATOR: %d" % (use_simulator))
    print("STEP_DUR (s): %f\n" % (step_dur))


    settings = termios.tcgetattr(sys.stdin)

    # publish to simulator
    if(use_simulator == 1):
        pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    # publish to robot
    else:
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.init_node('turtlebot_teleop')

    receiver = receiver.Status(robot_ip,robot_port)

     # create movement register
    move = Movement(client.Connection(sumo_ip,sumo_port,receiver),speed,turn,step_dur)

    # start thread to listen to incoming messages
    start_new_thread(receiver.start, (move,))

    # establish connection with sumo
    msg = messages.getIntegrationRequestMsg(
        move.speed, move.turn, receiver.ip, receiver.port)

    # waits for a positive reply to the integration request
    while 1:
        reply = move.conn.sendRequest(msg)
        print("RECEIVED: %s" % (msg))
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
