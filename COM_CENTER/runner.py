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





print(traci.vehicle.getSubscriptionResults("newVeh"))
for step in range(1000000):
   print("step", step)
   traci.simulationStep()
   char = screen.getch()

   if char == ord('q'):
       break
   elif char == curses.KEY_RIGHT:
     # print doesn't work with curses, use addstr instead

     edgeID = traci.vehicle.getRoadID("newVeh")
     pos = traci.vehicle.getPosition("newVeh")
     laneIndex = traci.vehicle.getLaneIndex("newVeh")

     traci.vehicle.moveToXY("newVeh", '' , -1 , pos[0], pos[1] + 0.6 , 40 , 2)
     screen.addstr(0, 0, 'right')

   print(traci.vehicle.getSubscriptionResults("newVeh"))
   print(traci.vehicle.getPosition("newVeh"))

traci.close()
screen.close()
