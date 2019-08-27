import time
import socket
import asyncore
from random import random
from random import choice


'''''''''''''''''''''''''''''
    Main telemetry handler
'''''''''''''''''''''''''''
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
'''''''''''''''''''''
class buggyTelemetryServer(asyncore.dispatcher):
    
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
        print('Telemetry> waiting for connection')

    
    def handle_accept(self):
        global telemetryClient
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Telemetry> incoming connection from: ', repr(addr))
            self.handler = buggyTelemetryHandler(sock)
            telemetryClient = True

    def sendTelemetry(self, data):
        global telemetryClient
        if telemetryClient:
            self.handler.send(data)

def main():

    global telemetryClient
    telemetryClient = False
    
    direction = 0
    throttle = 1
    actual_error = 2

    # Server start
    serverTelemetry = buggyTelemetryServer("localhost", 7001)

    # Main loop
    counter = 0
    while True:

        # Non blocking call
        asyncore.loop(timeout=0, count=1)

        # Telemetry
        msg = str(counter)+';'
        nbParam = 10
        for i in range(0,nbParam):
            msg += str(float(   random() * choice([-1,1]) * choice([10,100,1000]) ))
            if i != (nbParam-1):
                msg += ";"
        length = str(len(msg)).ljust(4)
        serverTelemetry.sendTelemetry(length.encode("utf-8"))
        serverTelemetry.sendTelemetry(msg.encode("utf-8"))

        counter += 1

# Le C ca rassure !!
if __name__ == "__main__":
    main()
    
