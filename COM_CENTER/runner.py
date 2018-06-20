import os
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


sumoBinary = "/usr/bin/sumo-gui"
sumoCmd = []

import traci
import traci.constants as tc
import curses
import json
from server import Server
import messages
import vehicle_controller as vc
import messages_listener
from thread import start_new_thread


traci.start(["sumo-gui", "-c", "../SUMO/data/hello.sumocfg"])
traci.route.add("trip", ["E12", "E23"])


def trackCarsInJunction():
    distance = 10
    junctions_list = traci.junction.getIDList()
    for junction in junctions_list:
        traci.junction.subscribeContext(
            junction, tc.CMD_GET_VEHICLE_VARIABLE, distance, [tc.VAR_LANE_ID])
    return


def addCar(car_id, route, car_type):
    traci.vehicle.addFull(car_id, route, car_type)
    traci.vehicle.subscribe(
        car_id, (tc.VAR_ROAD_ID, tc.VAR_LANE_ID, tc.VAR_POSITION, tc.VAR_ANGLE))
    traci.vehicle.setColor(car_id, (100, 254, 100, 254))
    return


def stopAtTrafficLights(car_id, car_lane):

    lane_id = traci.vehicle.getLaneID(car_id)

    junction_list = traci.junction.getIDList()

    # get the cars near junction
    cars_in_junction = {}
    for id in junction_list:
        junction_info = traci.junction.getContextSubscriptionResults(id)
        if(junction_info is not None):
            cars_in_junction.update(junction_info)

    traffic_lights = traci.trafficlight.getIDList()

    for light_id in traffic_lights:
        controlled_lanes = traci.trafficlight.getControlledLanes(light_id)
        # car in a lane controlled by trafficlight &  currente distance <= "distance"
        pp.pprint(controlled_lanes)
        if car_lane in controlled_lanes and car_id in cars_in_junction:
            pp.pprint("yup, ele ta aqui")
    return False

step_duration = 10000

trackCarsInJunction()
addCar("newVeh", "trip", "reroutingType")
config = vc.VehicleController("newVeh",step_duration)

controllers = [config]  #list of controllers

start_new_thread(messages_listener.listener,(controllers,))

print(traci.vehicle.getSubscriptionResults("newVeh"))

for step in range(step_duration): 

    for i  in range(0,len(controllers)):
        controllers[i].step()

    traci.simulationStep()

    
    

traci.close()
