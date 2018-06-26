import socket
import sys
import time
import analytics
import json
import resources
import messages

class Status:
    ip = "127.0.0.1"
    port = 3001
    awaiting_response = False
    response = None
    live = True

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port


    def start_waiting(self):
        analytics.start()
        self.awaiting_response = True


    def get_response(self):
        if(not self.awaiting_response):
            data = self.response
            self.response = None
            return data
        else:
            return None

    def start(self,move):

        # creating socket
        BUFFER_SIZE = 100
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((self.ip, self.port))
        s.listen(1)

        # setting loop
        while self.live:
            conn, addr = s.accept()
            if(conn is not None):

                # receiving message
                msg = conn.recv(BUFFER_SIZE).decode("utf-8")

                if(msg is not None or len(msg) != 0):
                    # set response so main thread can analyze it
                    analytics.end()
                    self.response = msg
                    self.awaiting_response = False

                    info = json.loads(msg)

                    print("RECEIVED: %s" % (msg))

                    # handle incomming messages
                    
                    if(info["type"] == resources.MsgType.MOVEMENT.value):
                        messages.handleMovementMessage(info, move)
                    elif(info["type"] == resources.MsgType.CALIBRATION.value):
                        reply = messages.handleCalibrationMessage(info)
                        move.conn.sendMessage(reply)
