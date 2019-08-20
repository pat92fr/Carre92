import sys,os
import random
import socket
import time
import subprocess
import msvcrt


class miniClient:

    def __init__(self):
        self.fps = 0
        self.lid = 0
        self.lig = 0
        self.ia_started = False
        self.s = 0
        
        #self.ip = '10.0.10.49'  # The server's hostname or IP address : Test jetson
        #self.ip = '10.42.0.1'  # The server's hostname or IP address : REAL BUGGY ! 
        #self.ip = '192.168.1.18'  # The server's hostname or IP address : Test jetson
        #self.ip = '192.168.1.5'  # The server's hostname or IP address : Test jetson
        self.ip = '10.42.0.1'  # The server's hostname or IP address : REAL BUGGY ! 
        
    def run_win_cmd(self, cmd):
        result = []
        process = subprocess.Popen(cmd,
                                   shell=True,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        for line in process.stdout:
            result.append(line)
        errcode = process.returncode
        for line in result:
            print(line)
        if errcode is not None:
            raise Exception('cmd %s failed, see above for details', cmd)


    def main_loop(self):
        



        #self.initialize_display()
        self.socket_client()
            
        done = False
        keys = 0x20
        while not done:

            #self.update_display(keys)


            # recieve socket frame and send response                
            data = self.s.recv(1024)
            self.fps = int.from_bytes(data[:2],  byteorder='big')
            self.lig = int.from_bytes(data[2:4], byteorder='big')
            self.lid = int.from_bytes(data[4:],  byteorder='big')

            print("FPS : " + str(self.fps) + " lig : " + str(self.lig) + " lid : " + str(self.lid))

            if msvcrt.kbhit():

                keys = msvcrt.getch()

                print("got key !" + str(keys))

                if keys == b"c":
                    print("got c !")

                    print(" DOES NOT WORK MUST DEBUG THIS COMMAND")
                    self.run_win_cmd("D:\\gstreamer\\1.0\\x86_64\\bin\\gst-launch-1.0 -v udpsrc port=5500 caps = 'application/x-rtp' ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videoscale ! autovideosink sync=false")
                    print("camera started")
                    #.\gst-launch-1.0 -v udpsrc port=5500 caps = 'application/x-rtp' ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videoscale ! autovideosink sync=false
                
                if keys == b'i':
                    print("starting IA")
                    self.s.send(b'i')

                if keys == b'w':
                    print("starting wall following")
                    self.s.send(b'w')

                if keys == b's':
                    print("screenshot")
                    self.s.send(b's')

                if keys == b'q':
                    self.s.send(b'q')
                    print ("i will quit")
                    self.s.close()
                    return

                else:
                    self.s.send(b'n')

            else:
                self.s.send(b'n')

            



    def socket_client(self):


        HOST = self.ip  # The server's hostname or IP address : Test jetson
        PORT = 6000            # The port used by the server

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.s.connect((HOST, PORT))



def main():

    serv = miniClient()
    serv.main_loop()


if __name__ == "__main__":
    main()
