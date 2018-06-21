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
import resources


traci.start(["sumo-gui", "-c", "../SUMO/data/hello.sumocfg"])
route = "trip"
traci.route.add(route, ["E12", "E23"])

resources.trackCarsInJunction()

controllers = []  

# messages listener
start_new_thread(messages_listener.listener,(controllers,route,))


while True:
    traci.simulationStep()
    
    for i  in range(0,len(controllers)):
        controllers[i].step()
    
    

traci.close()
