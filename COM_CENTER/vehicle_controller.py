import os
import sys
from server import Server, Movement
import traci
import math
import numpy as np

class VehicleController:
    car_id = ""
    speed = 0.5
    angular = 1.0
    timer = 0
    increment = [0,0]           # [direction,angle]
    step_duration = 100000     #100 ms per step
    move_duration = 1000000    #1sec

    def __init__(self,id):
        self.car_id = id

    def setSpeed(self,speed):
        self.speed = speed

    def setAngular(self,angular):
        self.angular = angular

    '''
    direction=1 if forward or =0 if backward
    '''
    def setIncrement(self,direction,ang):
        
        #calculate percentage of movement for the 100 ms
        inc_dir = direction*self.step_duration/self.move_duration
        inc_ang = ang*math.degrees(self.angular)*self.step_duration/self.move_duration

        new_increment = [inc_dir,inc_ang]

        if(self.increment != new_increment):
            self.increment = [inc_dir,inc_ang]
            self.timer = 0

    def step(self):
        #get traci info
        keep_route = 2
        edge_id = ''
        step_info = traci.vehicle.getSubscriptionResults(self.car_id)
        lane = -1
        pos = traci.vehicle.getPosition(self.car_id)
        old_angle = traci.vehicle.getAngle(self.car_id)

        if self.timer < self.move_duration and self.increment != [0,0]:

            #increment timer
            self.timer = self.timer + self.step_duration

            print self.increment

            #set position
            traci.vehicle.moveToXY(
            self.car_id, edge_id, lane, 
            pos[0] + self.increment[0]*math.cos(old_angle), #x
            pos[1] + self.increment[0]*math.sin(old_angle), #y
            old_angle + self.increment[1], keep_route)      #angle
            
            print traci.vehicle.getAngle(self.car_id)
            print traci.vehicle.getPosition(self.car_id)

            if(self.timer >= self.move_duration):
                self.increment = [0,0]
                self.timer = 0
        else:
            #same position
            traci.vehicle.moveToXY(
            self.car_id, edge_id, lane, pos[0], pos[1], old_angle, keep_route)

        

            
