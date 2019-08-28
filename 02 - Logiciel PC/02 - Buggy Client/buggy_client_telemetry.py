#-*- coding: utf-8 -*-

########################
# pip install numpy
# pip install matplotlib
########################

# Import
########
import socket
import sys
import os
import datetime
import numpy as np
import matplotlib.pyplot as plt
import time
from argparse import ArgumentParser

# Main procedure
################
def main():

    # Check parameters
    ##################
    parser = ArgumentParser()
    parser.add_argument("-p", "--port", dest="port",
                        help="port number", type=int, default=7001)
    parser.add_argument("-i", "--ip", dest="host",
                        help="IP address or host", type=str, default='10.42.0.1')
    args = parser.parse_args()

    # Try the connection to the server
    ##################################
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((args.host, args.port))
    except:
        print('Connection #KO# to ' + args.host + ':' + str(args.port))
        sys.exit(0)

    print('Connection OK to ' + args.host + ':' + str(args.port))

    # Telemetry graph definition        
    sampleCtx     = {}
    sampleCtx[0]  = { "title": "time",         "unit": "",   "color": "w",    "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[1]  = { "title": "LIG",          "unit": "cm", "color": "-g",   "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[2]  = { "title": "LID",          "unit": "cm", "color": "-r",   "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[3]  = { "title": "LIH",          "unit": "cm", "color": "-b",   "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[4]  = { "title": "LIDAR error",  "unit": "cm", "color": "--k",  "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[5]  = { "title": "LIDAR error2", "unit": "cm", "color": "--k",  "factor": 1.0, "plot": 1, "obj": None }
    sampleCtx[6]  = { "title": "PID Lidar",    "unit": "",   "color": "--r",  "factor": 1.0, "plot": 2, "obj": None }
    sampleCtx[7]  = { "title": "PID AI",       "unit": "",   "color": "--b",  "factor": 1.0, "plot": 2, "obj": None }
    sampleCtx[8]  = { "title": "Direction",    "unit": "",   "color": "-r",   "factor": 1.0, "plot": 3, "obj": None }
    sampleCtx[9]  = { "title": "Gaz",          "unit": "",   "color": "-b",   "factor": 1.0, "plot": 3, "obj": None }
    sampleCtx[10] = { "title": "Mode",         "unit": "",   "color": "--k",  "factor": 1.0, "plot": 4, "obj": None }
    
    paramNumber = len(sampleCtx)
    print('Number of parameters:' + str(paramNumber))

    nbPlot = -1
    for i in range(len(sampleCtx)):
        print (f"#  - sample id {i}: factor: {sampleCtx[i]['factor']} => {sampleCtx[i]['title']} {sampleCtx[i]['unit']}")
        if nbPlot < sampleCtx[i]['plot']:
           nbPlot = sampleCtx[i]['plot']
    nbPlot += 1
    print ("")

    # Variable
    nbSample    = 1
    windowSize  = 5 * 60

    # Create the sample container
    sample_list = []
    for i in range(paramNumber):
        sample_list.append([])
        sample_list[i] = [0] * windowSize
    
    # Fill the timestamp array
    for i in range(windowSize):
        sample_list[0][i] = i

    # Display plot
    f, ax = plt.subplots(nbPlot, 1, sharex=True)
    for i in range(0, paramNumber):
        sampleCtx[i]['obj'] = ax[sampleCtx[i]['plot']]

    # Main while loop
    while True:
    
        # Wait for length
        data = client.recv(4)
        if len(data) != 0:
            length = int(data.decode("utf-8").replace(' ',''))
            # Wait for the payload
            data = client.recv(length)
            if len(data) != 0:
                res = data.decode("utf-8").rstrip('\n').rstrip(' ').split(';')
                
                if len(res) != paramNumber:
                    print('#Error')
                    continue
                
                for i in range(1, paramNumber):
                    sample_list[i][nbSample] = float(res[i]) * sampleCtx[i]["factor"]
                
                nbSample +=1
                
                if nbSample >= windowSize:
                    nbSample = 0
                    stopDisplay = True
                    for c in range(1, len(sampleCtx)):
                        ctx = sampleCtx[c]["obj"]
                        ctx.clear()
                        ctx.plot(np.array(sample_list[0]), np.array(sample_list[c]), sampleCtx[c]["color"], linewidth=1, label=sampleCtx[c]["title"])
                        handles, labels = ctx.get_legend_handles_labels()
                        ctx.legend(handles, labels)
                        ctx.grid(True)
                
                    print("# Press CTR-C to start a new aquisition.")
                    
                    while stopDisplay:
                        try:
                            plt.pause(0.5)
                        except KeyboardInterrupt:
                            print("# CTRL-C: new telemetry acquisition")
                            stopDisplay = False
                            for c in range(1, len(sampleCtx)):
                                ctx = sampleCtx[c]["obj"]
                                ctx.clear()
                            continue

    print('Deconnexion.')
    client.close()

# Main
if __name__ == '__main__':    
    main()


# EOF
