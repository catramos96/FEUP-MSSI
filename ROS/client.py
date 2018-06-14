#!/usr/bin/env python

import socket
from enum import Enum

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024

class Movement(Enum):
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3

def connect(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    return s

def sendMovement(socket, movement):
    msg = "move:" + str(movement.value)
    sendMessage(socket, msg)
    data = receiveResponse(socket, BUFFER_SIZE)
    return data

def sendMessage(socket, message):
    socket.send(message.encode('utf-8'))

def receiveResponse(socket, buffer):
    return socket.recv(buffer)

def close(socket):
    socket.close()

s = connect(TCP_IP, TCP_PORT)
data = sendMovement(s, Movement.FORWARD)
#sendMessage(s, MESSAGE)
#data = receiveResponse(s, BUFFER_SIZE)

print("received data: %s", data)
close(s)
