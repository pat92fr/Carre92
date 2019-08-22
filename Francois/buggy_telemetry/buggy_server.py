#-*- coding: utf-8 -*-

import asyncore
import asynchat
import socket
import time

from types import FunctionType

# Global
global param_1
global param_2
global param_3
global param_4

global telemetryClient

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
    
    def handle_close(self):
        print('Closed by peer.')
        self.close()
    
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
    Server
'''
class buggyServer(asyncore.dispatcher):
    
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)

    def handle_accept(self):
        print('handle_accept')
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Incoming connection from ', repr(addr))
            handler = buggyHandler(sock)



'''''''''''''''''''''''''''''
    Main telemetry handler
'''''''''''''''''''''''''''''
class buggyTelemetryHandler(asyncore.dispatcher_with_send):
  
    def handle_read(self):
        pass

    def handle_close(self):
        global telemetryClient
        print('Telemetry> connection closed.')
        telemetryClient = False
        self.close()

'''''''''''''''''''''''
    Server telemetry
'''''''''''''''''''''''
class buggyTelemetryServer(asyncore.dispatcher):
    
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
    
    def handle_accept(self):
        global telemetryClient
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Telemetry> incoming connection from ', repr(addr))
            handler = buggyTelemetryHandler(sock)
            self.handler = handler
            telemetryClient = True

    def sendTelemetry(self, data):
        global telemetryClient
        if telemetryClient:
            self.handler.send(data.encode("utf-8"))

# MAIN
server          = buggyServer('localhost', 1976)
serverTelemetry = buggyTelemetryServer('localhost', 1977)


# Init parameters
counter = 0
param_1 = 0.0
param_2 = 100.0
param_3 = 0.0

telemetryClient = False

# Main loop
while True:
    
    # Non blocking call
    asyncore.loop(timeout=0, count=1)

    # Wait
    time.sleep(0.1)
    counter += 1

    if 0: #(counter % 10) == 0:
        print("")
        print(counter)
        print("param_1: ", param_1)
        print("param_2: ", param_2)
        print("param_3: ", param_3)

    msg = str(counter) + ';' + str(param_1) + ';' + str(param_2) + ';' + str(param_3)
    serverTelemetry.sendTelemetry(msg)

    # In order to display graph
    param_1 += 1.0
    param_2 -= 2.0
    param_3 += 5.0

    if param_1 >100.0:
        param_1 = -100.0

    if param_2 <0.0:
        param_2 = 100.0

    if param_3 >100.0:
        param_3 = 0.0


# EOF
