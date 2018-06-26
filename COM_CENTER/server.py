#!/usr/bin/env python
import socket
import signal
import sys


class Server:

    ip = '127.0.0.1'
    port= 5005
    BUFFER_SIZE = 200  # Normally 1024, but we want fast response

    def __init__(self,ip,port):
        global socket
        self.ip = ip
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((self.ip, self.port))
        self.s.listen(10)

    def sendMessage(self, message):
        self.conn.send(message.encode('utf-8'))

    def acceptConnection(self):
        conn, addr = self.s.accept()
        print('Connection address: ', addr)
        self.conn = conn

    def receiveMessage(self):
        data = self.conn.recv(self.BUFFER_SIZE).decode('utf-8')
        return data

    def close(self):
        global socket
        if(self.conn != None):
            self.conn.close()
        self.s.shutdown(socket.SHUT_WR)
        self.s.close()
        print("Server closed")