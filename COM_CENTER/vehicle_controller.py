import os
import sys
from server import Server, Movement
import traci
import traci.constants as tc
import math
import numpy as np
import resources
import socket

speed_scalar = 10

class VehicleController:
    car_id = ""
    speed = 0.5
    angular = 1.0
    timer = 0
    increment = [0,0]           # [direction,angle]
    step_duration = 100000     #100 ms per step
    move_duration = 1000000    #1sec

    inTraci = False
    route = ""

    def __init__(self,id,route):
        self.car_id = id
        self.route = route

    '''
    direction=1 if forward or =-1 if backward
    '''
    def setIncrement(self,direction,ang):

        #calculate percentage of movement for the 100 ms
        inc_dir = float(self.speed*speed_scalar*direction*self.step_duration/self.move_duration)
        inc_ang = float(ang*math.degrees(self.angular)*self.step_duration/self.move_duration)

        new_increment = [inc_dir,inc_ang]

        if(self.increment != new_increment):
            self.increment = [inc_dir,inc_ang]
            self.timer = 0

    def step(self):

        if(self.inTraci == False):
            resources.addCar(self.car_id, self.route, "reroutingType")
            self.inTraci = True

        else:
            #get traci info
            keep_route = 2
            edge_id = ''
            step_info = traci.vehicle.getSubscriptionResults(self.car_id)
            lane = -1
            pos = traci.vehicle.getPosition(self.car_id)
            old_angle = traci.vehicle.getAngle(self.car_id)

            x = pos[0] + self.increment[0]*math.cos(math.radians(450-old_angle))
            y = pos[1] + self.increment[0]*math.sin(math.radians(450-old_angle))

            stop_at_trafficlight = resources.stopAtTrafficLights(self.car_id, step_info[tc.VAR_LANE_ID])
            collision = resources.collision(self.car_id, [x,y])
            print("lane", step_info[tc.VAR_LANE_ID])
            print("para no semaforo? ", self.car_id, stop_at_trafficlight)
            print("para  por colisao? ", self.car_id, collision)
            if self.timer < self.move_duration and self.increment != [0,0] and not stop_at_trafficlight and not collision:

                #increment timer
                self.timer = self.timer + self.step_duration

                #set position
                traci.vehicle.moveToXY(
                self.car_id, edge_id, lane,
                x,
                y,
                old_angle + self.increment[1], keep_route)      #angle

                if(self.timer >= self.move_duration):
                    self.increment = [0,0]
                    self.timer = 0
            else:
                #same position
                traci.vehicle.moveToXY(
                self.car_id, edge_id, lane, pos[0], pos[1], old_angle, keep_route)

    #not tested
    def send_message(self, msg):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.ip, self.port))
        s.send(msg.encode("utf-8")
        s.close()
