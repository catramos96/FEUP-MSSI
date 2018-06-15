from server import Server

import json
import signal
import sys

server = Server()
server.acceptConnection()

def ctrl_z_handler(signal, frame):
    global server
    server.close()
    sys.exit(0)

def handleMessage(data):
    global server
    if(data == "start"):
        server.sendMessage("0")
    else:
        info = json.loads(data)
        if("move" in info):
            print(info["move"])
            server.sendMessage("ok")
    

signal.signal(signal.SIGINT, ctrl_z_handler)

i = True
while(i == True):
    msg = server.receiveMessage()
    handleMessage(msg)

