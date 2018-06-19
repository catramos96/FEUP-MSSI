import os, sys

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
from server import Server, Movement


traci.start(["sumo-gui", "-c", "../SUMO/data/hello.sumocfg"])
traci.route.add("trip", ["E12", "E23"])

def trackCarsInJunction():
    distance = 10
    junctions_list = traci.junction.getIDList()
    for junction in junctions_list:
        traci.junction.subscribeContext(junction, tc.CMD_GET_VEHICLE_VARIABLE, distance, [tc.VAR_LANE_ID])
    return;


def addCar(car_id, route, car_type):
    traci.vehicle.addFull(car_id,route, car_type)
    traci.vehicle.subscribe(car_id, (tc.VAR_ROAD_ID, tc.VAR_LANE_ID, tc.VAR_POSITION, tc.VAR_ANGLE))
    traci.vehicle.setColor(car_id, (100,254,100,254))
    return;


def stopAtTrafficLights(car_id, car_lane):

    lane_id = traci.vehicle.getLaneID(car_id)

    junction_list = traci.junction.getIDList()

    #get the cars near junction
    cars_in_junction = {}
    for id in junction_list:
        junction_info = traci.junction.getContextSubscriptionResults(id)
        if( junction_info is not None):
            cars_in_junction.update(junction_info)

    traffic_lights = traci.trafficlight.getIDList();

    for light_id in traffic_lights:
        controlled_lanes = traci.trafficlight.getControlledLanes(light_id)
        #car in a lane controlled by trafficlight &  currente distance <= "distance"
        pp.pprint(controlled_lanes)
        if car_lane in controlled_lanes and car_id in cars_in_junction:
            pp.pprint("yup, ele ta aqui")
    return False;

def moveCar(car_id, x, y, angle):
    keep_route = 2
    edge_id = ''
    step_info =  traci.vehicle.getSubscriptionResults(car_id)
    lane = -1
    pos = traci.vehicle.getPosition(car_id)
    old_angle = traci.vehicle.getAngle(car_id)


    traci.vehicle.moveToXY(car_id, edge_id , lane , pos[0] + x, pos[1]  + y, old_angle + angle, keep_route)
    return;


trackCarsInJunction()
addCar("newVeh", "trip", "reroutingType")


server = Server()
server.acceptConnection()




print(traci.vehicle.getSubscriptionResults("newVeh"))
for step in range(1000000):
   print("step", step)
   traci.simulationStep()
   msg = server.receiveMessage()
   if(msg == "start"):
       server.sendMessage("ok")
   else:
       info = json.loads(msg)
       if("move" in info):
           move = info["move"]
           if move == ord('q'):
               break
           elif move == Movement.RIGHT.value:
               moveCar("newVeh",1,0, 40)
           elif move == Movement.BACKWARD.value:
               moveCar("newVeh",0,-1, 40)
           elif move == Movement.LEFT.value:
               moveCar("newVeh",-1, 0, 40)
           elif move == Movement.FORWARD.value:
               moveCar("newVeh",0,1, 40)
           server.sendMessage("ok")

traci.close()
