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
    increment = [0,0,0]
    step_duration = 1000000
    move_duration = 1000000

    def __init__(self,id,step_dur):
        self.car_id = id
        self.step_duration = step_dur

    def setSpeed(self,speed):
        self.speed = speed

    def setAngular(self,angular):
        self.angular = angular

    def setIncrement(self,x,y,ang):
        inc_x = x*self.step_duration/self.move_duration
        inc_y = y*self.step_duration/self.move_duration
        inc_ang = ang*math.degrees(self.angular)*self.step_duration/self.move_duration

        new_increment = [inc_x,inc_y,inc_ang]

        if(self.increment != new_increment):
            self.increment = [inc_x,inc_y,inc_ang]
            self.timer = 0

    def step(self):
        if self.timer < self.move_duration and self.increment != [0,0,0]:

            #get traci info
            keep_route = 2
            edge_id = ''
            step_info = traci.vehicle.getSubscriptionResults(self.car_id)
            lane = -1
            pos = traci.vehicle.getPosition(self.car_id)
            old_angle = traci.vehicle.getAngle(self.car_id)

            #increment timer
            self.timer = self.timer + self.step_duration

            #set position
            traci.vehicle.moveToXY(
            self.car_id, edge_id, lane, pos[0] + self.increment[0], pos[1] + self.increment[1], old_angle + self.increment[2], keep_route)

            
            print traci.vehicle.getAngle(self.car_id)

            print traci.vehicle.getPosition(self.car_id)

            if(self.timer >= self.move_duration):
                self.increment = [0,0,0]
                self.timer = 0

            
