#-*- coding: utf-8 -*-

########################
# pip install numpy
# pip install matplotlib
########################

# Import
########
import sys
import os
import math
import time
import socket
import datetime
import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser
from matplotlib.widgets import Button
from PIL import Image

# Main procedure
################
def main():

    server_address = '192.168.1.34'
    server_port = 7001
    param_count = 16

    # Try the connection to the server
    ##################################
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((server_address, server_port))
    except:
        print('Connection #KO# to ' + server_address + ':' + str(server_port))
        sys.exit(0)

    print('Connection OK to ' + server_address + ':' + str(server_port))

    x = 0.0 #m
    y = 0.0 #m

    carto = np.zeros((4096,4096,3), dtype=np.uint8)
    carto_offset_x = 2048
    carto_offset_y = 2048
    carto_scale = 50

    #fig, ax = plt.subplots()

    # Main while loop
    loop_counter = 0
    while True:
    
        # Wait for length
        data = client.recv(4)
        if len(data) != 0:
            length = int(data.decode("utf-8").replace(' ',''))
            # Wait for the payload
            data = client.recv(length)
            if len(data) != 0:
                res = data.decode("utf-8").rstrip('\n').rstrip(' ').split(';')
                if len(res) != param_count:
                    print('#Error')
                    continue

                # read data
                right_lidar = float(res[2])
                left_lidar = float(res[1])
                speed = float(res[8])
                heading = float(res[15])
                #print("right_lidar:" + str(round(right_lidar,2)) + " left_lidar:" + str(round(left_lidar,2)) + " speed:" + str(round(speed,2)) + " heading:" + str(int(heading)) )

                # estimate position
                x += speed * 1.0/60.0 * math.cos(math.radians(heading))
                y += speed * 1.0/60.0 * math.sin(math.radians(heading))
                print("position:(" + str(round(x,2)) + "," + str(round(y,2)) + ")" )

                x1 = x + left_lidar * math.cos(math.radians(heading+60))
                y1 = y + left_lidar * math.sin(math.radians(heading+60))

                x2 = x + right_lidar * math.cos(math.radians(heading-60))
                y2 = y + right_lidar * math.sin(math.radians(heading-60))

                # update map
                carto[carto_offset_x + int(x1*carto_scale), carto_offset_y + int(y1*carto_scale) ] = [255,0,0]
                carto[carto_offset_x + int(x2*carto_scale), carto_offset_y + int(y2*carto_scale) ] = [0,255,0]


                loop_counter += 1
                if loop_counter % (60*60*4) == 0:
                    img = Image.fromarray(carto, 'RGB')
                    img.save('my.png')
                    #img.show()
                    #plt.imshow(carto)
                    #plt.pause(0.001)





# Main
if __name__ == '__main__':    
    main()

# EOF

