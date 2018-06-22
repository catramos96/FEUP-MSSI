import socket
import sys
import time
import analytics

#do not create instances of this class!!!
#use the class directly
class Status:
    ip = "127.0.0.1"
    port =3001
    awaiting_response = False
    response = None
    live = True

def start_waiting():
    analytics.start()
    Status.awaiting_response = True

def get_response():
    if(not Status.awaiting_response):
        return Status.response
    else:
        return None

def start():
    #creating socket
    BUFFER_SIZE = 100
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((Status.ip, Status.port))
    s.listen(1)

    #setting loop
    while Status.live:
        conn, addr = s.accept()
        if(conn is not None):
            data = conn.recv(BUFFER_SIZE).decode("utf-8")
            if(data is not None):
                #set response so main thread can analyze it
                analytics.end()
                Status.response = data
                Status.awaiting_response = False

