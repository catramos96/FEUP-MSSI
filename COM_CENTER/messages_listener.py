from thread import start_new_thread
import time
import vehicle_controller as vc
from server import Server
import json
import messages

def getController(id,controllers):
    for i in range(0, len(controllers)):
        if(controllers[i].car_id == id):
            return controllers[i]
    
    return -1

def listener(controllers):

    server = Server()
    server.acceptConnection()

    while True:
        msg = server.receiveMessage()
    
        if(msg == "start"):
            server.sendMessage("ok")
        else:
            info = json.loads(msg)
            id = info["id"]

            controller = getController(id,controllers)

            if(controller != -1):

                print(msg)

                if(info["type"] == messages.MsgType.MOVEMENT.value):
                    reply = messages.handleMovementMessage(info,controller)
                    server.sendMessage(reply)            
                elif(info["type"] == messages.MsgType.SPEED_ROTATION.value):
                    reply = messages.handleSpeedRotationMessage(info,controller)
                    server.sendMessage(reply) 