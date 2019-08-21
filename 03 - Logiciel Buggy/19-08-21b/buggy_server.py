#-*- coding: utf-8 -*-

import asyncore
import socket
import time

# Global
global param_1
global param_2
global param_3
global param_4

# Commands & misc
CMD_GET_API  = 'GET_API'
CMD_GET      = 'GET'
CMD_SET      = 'SET'
CMD_MODE     = 'MODE'
CMD_SPLIT    = ';'

'''
    Main read/write function
'''
class buggyHandler(asyncore.dispatcher_with_send):
    
    def handle_read(self):
        global param_1
        global param_2
        global param_3
        global param_4

        data = self.recv(8192)
        if data:
            print('Receive command: ' + data.decode("utf-8"))
            res = data.decode("utf-8").rstrip(CMD_SPLIT).split(CMD_SPLIT)
            print(res)

            if (res[0] == "GET_API"):
                resp = "GET_API;param_1;param_2;param_3;param_4"
                print(resp)
                self.send(resp.encode("utf-8"))
                print('Response sent.')
                return

            if (res[0] == "GET"):
                resp = "GET;" + str(param_1) + ";" + str(param_2) + ";"  + str(param_3) + ";" + str(param_4)
                print(resp)
                self.send(resp.encode("utf-8"))
                print('Response sent.')
                return

            if (res[0] == "SET"):
                param_1 = float(res[1])
                param_2 = float(res[2])
                param_3 = float(res[3])
                param_4 = float(res[4])
                return
'''
    Server creation
'''
class buggyServer(asyncore.dispatcher):
    
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)

    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Incoming connection from ', repr(addr))
            handler = buggyHandler(sock)

server = buggyServer('localhost', 1976)
counter = 0

# Init parameters
param_1 = 0.0
param_2 = 0.0
param_3 = 0.0
param_4 = 0.0

# Main loop
while True:
    # Non blocking call
    asyncore.loop(timeout=0, count=1)

    # Wait
    time.sleep(0.1)
    counter += 1

    if (counter % 10) == 0:
        print("param_1: ", param_1)
        print("param_2: ", param_2)
        print("param_3: ", param_3)
        print("param_4: ", param_4)

# EOF
