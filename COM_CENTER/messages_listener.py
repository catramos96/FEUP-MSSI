from thread import start_new_thread
import time
import vehicle_controller as vc
from server import Server
import json
import messages


def getController(id, controllers):
    for i in range(0, len(controllers)):
        if(controllers[i].car_id == id):
            return controllers[i]

    return -1


def listener(controllers, route):

    server = Server()
    server.acceptConnection()

    integration_requests = 0

    print("Started listening to incoming messages ...")

    while True:
        msg = server.receiveMessage()
        info = json.loads(msg)

        id = ''

        if 'id' in info:
            id = info["id"]

        controller = getController(id, controllers)

        print(msg)
        reply_sent = False

        if(controller != -1):

            if(info["type"] == messages.MsgType.MOVEMENT.value):
                reply = messages.handleMovementMessage(info, controller)
                server.sendMessage(reply)
                reply_sent = True

            elif(info["type"] == messages.MsgType.SPEED_ROTATION.value):
                reply = messages.handleSpeedRotationMessage(info, controller)
                server.sendMessage(reply)
                reply_sent = True

        else:

            if(info["type"] == messages.MsgType.INTEGRATION_REQUEST.value):
                integration_requests = integration_requests + 1
                reply = messages.handleIntegrationRequestMessage(info, controllers,route,integration_requests)
                server.sendMessage(reply)
                reply_sent = True

        if(reply_sent == False):
            reply = messages.getRejectedMessage(id)
            server.sendMessage(reply)
