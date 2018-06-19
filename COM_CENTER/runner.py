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

screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)


traci.start(["sumo-gui", "-c", "../SUMO/data/hello.sumocfg"])
traci.route.add("trip", ["E12", "E23"])
traci.vehicle.add("newVeh","trip",typeID="reroutingType")

traci.vehicle.subscribe("newVeh", (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))

traci.vehicle.setColor("newVeh", (100,254,100,254))
server = Server()
server.acceptConnection()

def moveCar(carID, x, y, angle):
    keepRoute = 2
    edgeID = ''
    keepRoute = 1
    lane = -1
    pos = traci.vehicle.getPosition("newVeh")
    old_angle = traci.vehicle.getAngle("newVeh")
    traci.vehicle.moveToXY("newVeh", edgeID , lane , pos[0] + x, pos[1]  + y, old_angle + angle, keepRoute)
    return;



print(traci.vehicle.getSubscriptionResults("newVeh"))
for step in range(1000000):
   print("step", step)
   traci.simulationStep()
   #char = screen.getch()
   msg = server.receiveMessage()
   if(msg == "start"):
       server.sendMessage("ok")
   else:
       info = json.loads(msg)
       if("move" in info):
           move = info["move"]
           if move == ord('q'):
               break
           elif move == Movement.RIGHT:
               moveCar('newVeh',1,0, 40)
           elif move == Movement.BACKWARD:
               moveCar('newVeh',0,-1, 40)
           elif move == Movement.LEFT:
               moveCar('newVeh',-1, 0, 40)
           elif move == Movement.FORWARD:
               moveCar('newVeh',0,1, 40)
           server.sendMessage("ok")
   print(traci.vehicle.getSubscriptionResults("newVeh"))
   print(traci.vehicle.getPosition("newVeh"))

traci.close()
screen.close()
