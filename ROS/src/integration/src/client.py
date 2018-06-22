#!/usr/bin/env python

import socket
from enum import Enum
from analytics import Timer
import receiver

class Connection:
    
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3
    
    TCP_IP = '127.0.0.1'
    TCP_PORT = 5005
    BUFFER_SIZE = 1024

    timer = Timer()

    def __init__(self):
        
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.TCP_IP, self.TCP_PORT))

    def sendRequest(self, msg):
        self.sendMessage(msg)
        ###
        #take this in the future
        self.timer.start()
        #trade with this
        '''
        receiver.start_waiting()
        while True:
            #maybe put here signals to reduce load
            data = receiver.get_response()
            if(data is not None):
                break
                '''
        #this too to take out
        data = self.receiveResponse()
        self.timer.end()
        ###
        return data

    def sendMessage(self, message):
        self.s.send(message.encode('utf-8'))

    def receiveResponse(self):
        return self.s.recv(self.BUFFER_SIZE)

    def close(self):
        self.s.close()

#conn = Connection()
#data = conn.sendMovement(0)
#sendMessage(s, MESSAGE)
#data = receiveResponse(s, BUFFER_SIZE)

#print("received data: %s", data)
#conn.close()
