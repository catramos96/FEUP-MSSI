from thread import start_new_thread
import time
import vehicle_controller as vc
from server import Server
import json
import messages
import msg_resources 

def getController(id, controllers):
    for i in range(0, len(controllers)):
        if(controllers[i].car_id == id):
            return controllers[i]

    return -1


def listener(controllers, route,ip,port):

    server = Server(ip,port)
    server.acceptConnection()

    integration_requests = 0

    print("Started listening to incoming messages ...")

    while True:

        # blocks until a message is received 
        msg = server.receiveMessage()
        info = json.loads(msg)

        id = ''

        if 'id' in info:
            id = info["id"]

        controller = getController(id, controllers)

        print("RECEIVED: %s" % (msg))

        if(controller != -1):

            # handle received messages

            if(info["type"] == msg_resources.MsgType.MOVEMENT.value):
                messages.handleMovementMessage(info, controller)

            elif(info["type"] == msg_resources.MsgType.SPEED_ROTATION.value):
                messages.handleSpeedRotationMessage(info, controller)

            elif(info["type"] == msg_resources.MsgType.INTEGRATION_REQUEST.value):
                reply = messages.getRejectedMessage(id)
                controller.send_message(reply)
            elif(info["type"] == msg_resources.MsgType.CALIBRATION.value):
                messages.handleCalibrationMessage(info,controller)

        else:

            if(info["type"] == msg_resources.MsgType.INTEGRATION_REQUEST.value):
                integration_requests = integration_requests + 1
                reply = messages.handleIntegrationRequestMessage(info, controllers,route,integration_requests)   