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
import math
import pprint
pp = pprint.PrettyPrinter(indent=4)


screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

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
    traci.vehicle.subscribeContext(car_id, tc.CMD_GET_VEHICLE_VARIABLE, 15, [tc.VAR_POSITION, tc.VAR_WIDTH, tc.VAR_HEIGHT])
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
        if car_lane in controlled_lanes and car_id in cars_in_junction:
            index = controlled_lanes.index(car_lane)
            pp.pprint(index)
            traffic_colors = traci.trafficlight.getRedYellowGreenState(light_id)
            if (len(traffic_colors) > index ):
                color = traffic_colors[index]
                if(color == "r" or color == "R"):
                    return True
    return False;

def get_rear_bumper_point(x,y, ang, length):
    return [x + length*math.cos(ang), y+length*math.sin(ang)]

def overlap(pos,r, pos2, r2):
    print(math.pow(pos2[0]-pos[0],2) + math.pow(pos[1]-pos2[1],2), "<=", math.pow(r + r2, 2))
    if (math.pow(pos2[0]-pos[0],2) + math.pow(pos[1]-pos2[1],2) <= math.pow(r + r2, 2)):
        return True
    return False

def collision(car_id, pos):
    nearby_cars = traci.vehicle.getContextSubscriptionResults(car_id)

    if(nearby_cars is None):
        return

    angle = traci.vehicle.getAngle(car_id)
    length = traci.vehicle.getLength(car_id)
    widht = traci.vehicle.getWidth(car_id)
    rear_bumper_center = get_rear_bumper_point(pos[0],pos[1],angle,length)
    front_bumper_center = pos


    for car_near in nearby_cars:

        if(car_near == car_id):
            continue

        near_angle = traci.vehicle.getAngle(car_near)
        near_length = traci.vehicle.getLength(car_near)
        near_front_bumper = traci.vehicle.getPosition(car_near)
        near_rear_bumper = get_rear_bumper_point(near_front_bumper[0],
                                                     near_front_bumper[1],
                                                     near_angle,
                                                     near_length
                                                     )
        print(front_bumper_center, rear_bumper_center, ":",near_front_bumper, near_rear_bumper)

        near_width = traci.vehicle.getWidth(car_near)

        safe_dist = 0
        #front front bumpers
        if(overlap(front_bumper_center, length + safe_dist, near_front_bumper, near_width)):
           return True
        #front rear
        if(overlap(front_bumper_center, length + safe_dist, near_rear_bumper, near_width)):
           return True

        #rear front
        if(overlap(rear_bumper_center, length + safe_dist, near_front_bumper, near_width)):
           return True
        #rear rear
        if(overlap(rear_bumper_center, length + safe_dist, near_rear_bumper, near_width)):
           return True


    return False

def moveCar(car_id, x, y, angle):
    keep_route = 2
    edge_id = ''
    step_info =  traci.vehicle.getSubscriptionResults(car_id)
    lane = -1
    pos = step_info[tc.VAR_POSITION]
    old_angle = step_info[tc.VAR_ANGLE]

    if(not stopAtTrafficLights(car_id, pos) ):
        traci.vehicle.moveToXY(car_id, edge_id , lane , pos[0] + x, pos[1]  + y, old_angle + angle, keep_route)
    return;


trackCarsInJunction()
addCar("newVeh", "trip", "reroutingType")

for step in range(1000000):
   print("step", step, "\n")
   traci.simulationStep()

   if(step == 20 ):
       addCar("meme", "trip", "reroutingType")

   subs = traci.vehicle.getSubscriptionResults("newVeh")
   pos = traci.vehicle.getPosition("newVeh")

   print("parar no semaforo? ")
   print(stopAtTrafficLights("newVeh", subs[tc.VAR_LANE_ID]))


   print("newVeh: ", collision("newVeh", pos), "\n")
   if(step > 20 ):
       pos_meme = traci.vehicle.getPosition("meme")
       print("meme: ", collision("meme", pos_meme), "\n")

   char = screen.getch()

   if char == ord('q'):
       break
   elif char == curses.KEY_RIGHT:
       moveCar('newVeh',1,0, 40)
   elif char == curses.KEY_DOWN:
       moveCar('newVeh',0,-1, 40)
   elif char == curses.KEY_LEFT:
       moveCar('newVeh',-1, 0, 40)
   elif char == curses.KEY_UP:
       moveCar('newVeh',0,1, 40)

traci.close()
screen.close()
