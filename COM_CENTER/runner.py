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
import json
import messages
import vehicle_controller as vc
import messages_listener
from thread import start_new_thread
import sumo_resources
import calibration

if __name__ == "__main__":

    if(len(sys.argv) != 6):
        print(
            "Wrong number of arguments.\n 'python3 runner.py [sumo_ip] [sumo_port] [step_dur_seconds] [calibration_seconds] [n_vehicles]")
        exit

    sumo_ip = sys.argv[1]
    sumo_port = int(sys.argv[2])
    step_dur = float(sys.argv[3])
    calibration_dur = float(sys.argv[4])
    n_vehicles = int(sys.argv[5])

    print("SUMO_IP: %s" % (sumo_ip))
    print("SUMO_PORT: %d" % (sumo_port))
    print("STEP_DUR: %f" % (step_dur))
    print("CALIBRATION (s): %f" % (calibration_dur))
    print("N_VEHICLES: %d" % (n_vehicles))

    # start simulation
    traci.start(["sumo-gui", "--start", "-c", "../SUMO/data/hello.sumocfg"])
    route = "trip"
    traci.route.add(route, ["E12", "E23"])

    sumo_resources.trackCarsInJunction()

    controllers = []

    # messages listener thread
    start_new_thread(messages_listener.listener, (controllers, route,sumo_ip,sumo_port,))

    # calibration thread
    start_new_thread(calibration.calibration, (controllers, calibration_dur,))

    # independent vehicles
    for i in range(0, n_vehicles):
        sumo_resources.addCar('independent' + str(i), route, "reroutingType")


    while True:
        traci.simulationStep()

        for i in range(0, len(controllers)):
            controllers[i].step()

    traci.close()
