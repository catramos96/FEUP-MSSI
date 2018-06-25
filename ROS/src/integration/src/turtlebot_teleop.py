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
    last_updated = datetime.datetime.now()

    current_angle = 0

    sem = threading.Semaphore()

    def __init__(self):
        start_new_thread(self.check_movement, ())

    def move_update(self,x, y, z, th, from_thread):

        if(not from_thread):
            self.sem.acquire()

        old_angle_instruction = self.th
        old_update_instruction = self.last_updated

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


        #print("old_angle_instr.: %f" % (old_angle_instruction))
 
        '''
        if(old_angle_instruction != 0):
            elapsed_time = (self.last_updated - old_update_instruction).total_seconds()
       
            print("elapsed_time: %f" % (elapsed_time))
            print("old_angle: %f" %(self.current_angle))

            self.current_angle = self.current_angle + elapsed_time * math.degrees(self.turn) * old_angle_instruction

            print("Turn degrees per second: %f" % (math.degrees(self.turn)))
            print("Angle increment: %f" % (elapsed_time * math.degrees(self.turn) * old_angle_instruction))

            if(self.current_angle >= 360):
                self.current_angle = self.current_angle - 360
            elif(self.current_angle < 0):
                self.current_angle = self.current_angle + 360

            print("current_angle: %f" % (self.current_angle))
        '''


    def vels(self):
        return "currently:\tspeed %s\tturn %s " % (self.speed, self.turn)

    def print(self):
        return " - [%d,%d,%d,%d]" % (self.x, self.y, self.z, self.th)       

    def check_movement(self):
        movement_dur = 0.1 #seconds

        while(1):

            sleep = 0
            self.sem.acquire()

            # movement was incresed
            if [self.x,self.y,self.z,self.th] != [0,0,0,0]:
                elapsed_time = datetime.datetime.now() - self.last_updated

                # movement duration is above 100 ms
                if(elapsed_time.total_seconds() >= movement_dur):
                    
                    # stop movement
                    self.move_update(0,0,0,0,True)
                    sleep = 0.1
                # wait until reach the 100 ms (aprox.)
                else:
                    sleep = (movement_dur-elapsed_time.total_seconds())
            
            self.sem.release()
            time.sleep(sleep)

    

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
