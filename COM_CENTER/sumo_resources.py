import traci
import traci.constants as tc
import math

FAR_AWAY = 1000

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
    traci.vehicle.subscribeContext(car_id, tc.CMD_GET_VEHICLE_VARIABLE, 15, [
                                   tc.VAR_POSITION, tc.VAR_WIDTH, tc.VAR_HEIGHT])
    traci.vehicle.setColor(car_id, (100, 254, 100, 254))
    return;


def stopAtTrafficLights(car_id, car_lane):

    lane_id = traci.vehicle.getLaneID(car_id)

    junction_list = traci.junction.getIDList()

    # get the cars near junction
    cars_in_junction = {}
    for id in junction_list:
        junction_info = traci.junction.getContextSubscriptionResults(id)
        if(junction_info is not None):
            cars_in_junction.update(junction_info)

    traffic_lights = traci.trafficlight.getIDList();

    for light_id in traffic_lights:
        controlled_lanes = traci.trafficlight.getControlledLanes(light_id)
        # car in a lane controlled by trafficlight &  currente distance <= "distance"
        if car_lane in controlled_lanes and car_id in cars_in_junction:
            index = controlled_lanes.index(car_lane)
            traffic_colors = traci.trafficlight.getRedYellowGreenState(
                light_id)
            if (len(traffic_colors) > index):
                color = traffic_colors[index]
                if(color == "r" or color == "R"):
                    return True
    return False;


def get_rear_bumper_point(x, y, ang, length):
    return [x + length*math.cos(ang), y+length*math.sin(ang)]


def overlap(pos, r, pos2, r2):
    if (math.pow(pos2[0]-pos[0], 2) + math.pow(pos[1]-pos2[1], 2) <= math.pow(r + r2, 2)):
        return math.pow(pos2[0]-pos[0], 2) + math.pow(pos[1]-pos2[1], 2)
    return FAR_AWAY


def collision(controller, pos):
    nearby_cars = traci.vehicle.getContextSubscriptionResults(
        controller.car_id)

    if(nearby_cars is None):
        controller.distance = FAR_AWAY
        return False

    angle = traci.vehicle.getAngle(controller.car_id)
    length = traci.vehicle.getLength(controller.car_id)
    widht = traci.vehicle.getWidth(controller.car_id)
    rear_bumper_center = get_rear_bumper_point(pos[0], pos[1], angle, length)
    front_bumper_center = pos

    min_dist = FAR_AWAY

    current_center_pos = [pos[0] - length/2 * math.cos(math.radians(450-angle)), pos[1] - length/2 * math.sin(math.radians(450-angle))]

    for car_near in nearby_cars:

        if(car_near == controller.car_id):
            continue

        near_angle = traci.vehicle.getAngle(car_near)
        near_length = traci.vehicle.getLength(car_near)
        near_front_bumper = traci.vehicle.getPosition(car_near)
        near_rear_bumper = get_rear_bumper_point(near_front_bumper[0],
                                                     near_front_bumper[1],
                                                     near_angle,
                                                     near_length
                                                     )

        near_width = traci.vehicle.getWidth(car_near)

        safe_dist = 0

        pos2 = near_front_bumper = traci.vehicle.getPosition(car_near)
        length2 = traci.vehicle.getLength(car_near)
        angle2 = traci.vehicle.getAngle(car_near)

        center_pos = [pos2[0] - length2/2 * math.cos(math.radians(450-angle2)), pos2[1] - length2/2 * math.sin(math.radians(450-angle2))]

        tmp = overlap(center_pos,length2/2.2, current_center_pos,length/2.2)

        if(tmp<min_dist):
            min_dist = tmp

        '''

        # front front bumpers
        min_dist = overlap(front_bumper_center, length + safe_dist, near_front_bumper, near_width)

        # front rear
        tmp = overlap(front_bumper_center, length + safe_dist, near_rear_bumper, near_width)

        if(tmp < min_dist):
            min_dist = tmp

        # rear front
        tmp = overlap(rear_bumper_center, length + safe_dist, near_front_bumper, near_width)

        if(tmp < min_dist):
            min_dist = tmp

        # rear rear
        tmp = overlap(rear_bumper_center, length + safe_dist, near_rear_bumper, near_width)
        if(tmp < min_dist):
            min_dist = tmp
        '''
        
    controller.distance = min_dist
    if(controller.distance != FAR_AWAY):
        return True


    return False
