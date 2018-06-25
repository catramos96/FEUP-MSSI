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
from server import Server
import messages
import vehicle_controller as vc
import messages_listener
from thread import start_new_thread
import sumo_resources


traci.start(["sumo-gui", "--start", "-c", "../SUMO/data/hello.sumocfg"])
route = "trip"
traci.route.add(route, ["E12", "E23"])

sumo_resources.trackCarsInJunction()

controllers = []

# messages listener
start_new_thread(messages_listener.listener,(controllers,route,))

# independent vehicles
'''
sumo_resources.addCar('independent1', route, "reroutingType")
sumo_resources.addCar('independent2', route, "reroutingType")
'''

print(traci.simulation.getDeltaT() / 10 )


counter = 0
n_vehicles_added = 2
while True:
    traci.simulationStep()

    for i  in range(0,len(controllers)):
        controllers[i].step()

    '''
    if(counter == 50):   #5sec
        counter = 0
        sumo_resources.addCar('independent'+n_vehicles_added+1, route, "reroutingType")
        sumo_resources.addCar('independent'+n_vehicles_added+2, route, "reroutingType")
        n_vehicles_added = n_vehicles_added + 3
    '''



traci.close()
