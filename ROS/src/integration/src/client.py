#!/usr/bin/env python

import socket
from enum import Enum
from analytics import Timer

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
    
    def start(self):
        self.sendMessage("start")
        return self.receiveResponse()

    def sendMovement(self, msg):
        self.sendMessage(msg)
        timer.start()
        data = self.receiveResponse()
        timer.end()
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
