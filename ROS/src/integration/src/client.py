#!/usr/bin/env python

import socket
from enum import Enum
from analytics import Timer
import receiver

'''
Class that establishes a connection with a specific ip and port
'''


class Connection:
    ip = '127.0.0.1'
    port = 5005
    BUFFER_SIZE = 1024

    timer = Timer()

    def __init__(self,ip,port,receiver):
        self.ip = ip
        self.port = port
        self.receiver = receiver
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.ip, self.port))
    '''
    Sends a request and blocks until getting a reply
    '''

    def sendRequest(self, msg):
        self.sendMessage(msg)

        self.receiver.start_waiting()
        while True:
            data = self.receiver.get_response()
            if(data is not None):
                break

        return data

    """ Sends a message to the connection ip and port """

    def sendMessage(self, message):
        self.s.send(message.encode('utf-8'))

    """ Blocks until a response is received """

    def receiveResponse(self):
        return self.s.recv(self.BUFFER_SIZE)

    """ Closes the connection """

    def close(self):
        self.s.close()
