import socket
import sys
import time
import analytics
import json
import resources
import messages

# do not create instances of this class!!!
# use the class directly


class Status:
    ip = "127.0.0.1"
    port = 3001
    awaiting_response = False
    response = None
    live = True


def start_waiting():
    analytics.start()
    Status.awaiting_response = True


def get_response():
    if(not Status.awaiting_response):
        data = Status.response
        Status.response = None
        return data
    else:
        return None


def start(move):
    # creating socket
    BUFFER_SIZE = 100
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((Status.ip, Status.port))
    s.listen(1)

    # setting loop
    while Status.live:
        conn, addr = s.accept()
        if(conn is not None):

            # receiving message
            msg = conn.recv(BUFFER_SIZE).decode("utf-8")

            if(msg is not None or len(msg) != 0):
                # set response so main thread can analyze it
                analytics.end()
                Status.response = msg
                Status.awaiting_response = False

                info = json.loads(msg)

                print("RECEIVED: %s" % (info))

                # handle incomming messages
                
                if(info["type"] == resources.MsgType.MOVEMENT.value):
                    messages.handleMovementMessage(info, move)
                elif(info["type"] == resources.MsgType.CALIBRATION.value):
                    reply = messages.handleCalibrationMessage(info)
                    move.conn.sendMessage(reply)
