import os
import sys
from server import Server
import traci
import traci.constants as tc
import math
import numpy as np
import sumo_resources
import socket
import messages
import msg_resources
import json

speed_scalar = 10
FAR_AWAY = 1000


class VehicleController:
    car_id = ""
    speed = 0.5
    angular = 1.0
    increment = [0, 0]           # [direction,angle]
    step_duration = 100000  # 100 ms per step
    move_duration = 1000000  # time of velocity = 1 sec

    inTraci = False
    route = ""

    distance = sumo_resources.FAR_AWAY     # to other cars

    last_message_instruction = -1

    def __init__(self, id, route):
        self.car_id = id
        self.route = route

    '''
    direction=1 if forward or =-1 if backward
    '''

    def setIncrement(self, direction, ang, message):

        # calculate percentage of movement for the 100 ms
        inc_dir = float(self.speed*speed_scalar*direction *
                        self.step_duration/self.move_duration)
        inc_ang = float(ang*math.degrees(self.angular) *
                        self.step_duration/self.move_duration)
        self.last_message_instruction = json.dumps(message)

        new_increment = [inc_dir, inc_ang]

        if(self.increment != new_increment):
            self.increment = [inc_dir, inc_ang]

    def step(self):

        if(self.inTraci == False):
            sumo_resources.addCar(self.car_id, self.route, "reroutingType")
            self.inTraci = True

        else:
            # get traci info
            keep_route = 2
            edge_id = ''
            step_info = traci.vehicle.getSubscriptionResults(self.car_id)
            lane = -1
            pos = traci.vehicle.getPosition(self.car_id)
            old_angle = traci.vehicle.getAngle(self.car_id)

            # calculate next position
            x = pos[0] + self.increment[0] * \
                math.cos(math.radians(450-old_angle))
            y = pos[1] + self.increment[0] * \
                math.sin(math.radians(450-old_angle))

            # check colisions
            old_distance_value = self.distance
            stop_at_trafficlight = sumo_resources.stopAtTrafficLights(
                self.car_id, step_info[tc.VAR_LANE_ID])
            collision = sumo_resources.collision(self, [x, y])

            # move if:
            # - increment was request
            # - it's not stopped at a red light of a semaphore
            # - it's not colliding with another vehicle or the colision area is decreasing
            if self.increment != [0, 0] and not stop_at_trafficlight and ((collision and old_distance_value < self.distance) or not collision):

                # set position
                traci.vehicle.moveToXY(
                    self.car_id, edge_id, lane,
                    x,
                    y,
                    old_angle + self.increment[1], keep_route)  # angle

                self.increment = [0, 0]

                if(self.last_message_instruction != -1):
                    self.send_message(self.last_message_instruction)
            else:
                # same position
                traci.vehicle.moveToXY(
                    self.car_id, edge_id, lane, pos[0], pos[1], old_angle, keep_route)

    # not tested
    def send_message(self, msg):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.ip, self.port))
        s.send(msg.encode("utf-8"))
        s.close()
