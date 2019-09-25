#-*- coding: utf-8 -*-

########################
# pip install numpy
# pip install matplotlib
########################

# Import
########
import sys
import os
import time
import socket
import datetime
import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser
from matplotlib.widgets import Button

# Main procedure
################
def main():
    
    # Check parameters
    ##################
    parser = ArgumentParser()
    parser.add_argument("-p", "--port", dest="port",
                        help="port number", type=int, default=7001)
    parser.add_argument("-d", "--duration", dest="duration",
                        help="max sampling duration in seconds", type=int, default=5)
    parser.add_argument("-i", "--ip", dest="host",
                        help="IP address or host", type=str, default='192.168.1.34')
    parser.add_argument("--log",
                        help="Log telemetry in file", action="store_true")
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
    sampleCtx[0]  = { "title": "time",          "unit": "",   "color": "w",    "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[1]  = { "title": "LiG",           "unit": "cm", "color": "-g",   "factor": 1.0, "plot": 2, "obj": None }
    sampleCtx[2]  = { "title": "LiD",           "unit": "cm", "color": "-r",   "factor": 1.0, "plot": 2, "obj": None }
    sampleCtx[3]  = { "title": "LiH",           "unit": "cm", "color": "-b",   "factor": 1.0, "plot": 2, "obj": None }
    
    sampleCtx[4]  = { "title": "LiError",       "unit": "", "color": "g",  "factor": 1.0, "plot": 3, "obj": None }
    sampleCtx[5]  = { "title": "WallPID",       "unit": "",   "color": "k",  "factor": 1.0, "plot": 3, "obj": None }

    sampleCtx[6]  = { "title": "TrgtSpeed",     "unit": "ms",   "color": "--k",  "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[7]  = { "title": "CurrSpeed",     "unit": "ms",   "color": "-b",  "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[8]  = { "title": "ActlSpeed",     "unit": "ms",   "color": "-r",  "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[9]  = { "title": "SpeedError",   "unit": "ms",   "color": "g",  "factor": 1.0, "plot": 0, "obj": None }
    sampleCtx[10]  = { "title": "Throttle",     "unit": "",   "color": "k",  "factor": 1.0, "plot": 1, "obj": None }

    sampleCtx[11]  = { "title": "LinePos",      "unit": "",   "color": "r",  "factor": 1.0, "plot": 4, "obj": None }
    sampleCtx[12]  = { "title": "LinePID",      "unit": "",   "color": "k",  "factor": 1.0, "plot": 4, "obj": None }
    sampleCtx[13]  = { "title": "Steering",     "unit": "x10",   "color": "--r",   "factor": 1.0, "plot": 5, "obj": None }
    sampleCtx[14]  = { "title": "Steering",     "unit": "deg",   "color": "k",   "factor": 1.0, "plot": 5, "obj": None }
    #sampleCtx[15]  = { "title": "Autopilot",    "unit": "",   "color": "-b",   "factor": 1.0, "plot": 6, "obj": None }
    
    paramNumber = len(sampleCtx)
    print('Number of parameters:' + str(paramNumber))

    nbPlot = -1
    csvTitle = ''
    for i in range(len(sampleCtx)):
        csvTitle += sampleCtx[i]['title'] + ";"
        print (f"#  - sample id {i}: factor: {sampleCtx[i]['factor']} => {sampleCtx[i]['title']} {sampleCtx[i]['unit']}")
        if nbPlot < sampleCtx[i]['plot']:
           nbPlot = sampleCtx[i]['plot']
    nbPlot += 1
    print ("")
    
    # Create the file result
    if args.log :
        fileName = "./csv_buggy_telemetry_" + time.strftime("%Y%m%d-%H%M%S") + ".csv"
        csvFile = open(fileName, "w+")
        csvFile.write(csvTitle[:-1]+"\r\n")
        print('# Log telemetry to file ', fileName)
    
    # Variable
    nbSample    = 1
    windowSize  = args.duration * 60

    # Create the sample container
    sample_list = []
    for i in range(paramNumber):
        sample_list.append([])
        sample_list[i] = [0] * windowSize
    
    # Fill the timestamp array
    for i in range(windowSize):
        sample_list[0][i] = i

    # Create plot context
    f, ax = plt.subplots(nbPlot, 1, sharex=True)
    for i in range(0, paramNumber):
        sampleCtx[i]['obj'] = ax[sampleCtx[i]['plot']]
    
    # Create exit button
    theEnd     = plt.axes([0.8, 0.025, 0.1, 0.05])
    exitButton = Button(theEnd, 'Exit')

    # Exit button
    def exitNow(event):
        print('Action requested by user: exit.')
        client.close()
        if args.log :
            csvFile.close()
        sys.exit(0)
    exitButton.on_clicked(exitNow)

    # Create stop/go button
    global stop
    stop = False

    stopGo       = plt.axes([0.1, 0.025, 0.1, 0.05])
    stopGoButton = Button(stopGo, 'stop/go')

    # Sample button
    def stopGoAction(event):
        global stop
        if stop == True:
            stop = False
            print('Action requested by user: do sampling mode.')
        else:
            stop = True
            print('Action requested by user: stop sampling mode.')
    stopGoButton.on_clicked(stopGoAction)

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

                # Log to file ?
                if args.log :
                    csvFile.write(data.decode("utf-8")+"\r\n")

                # Log to ram
                for i in range(1, paramNumber):
                    sample_list[i][nbSample] = float(res[i]) * sampleCtx[i]["factor"]

                # One more sample
                nbSample +=1

                # Window is full ?
                if (nbSample >= windowSize):

                    nbSample = 0
                    
                    for c in range(1, len(sampleCtx)):
                        ctx = sampleCtx[c]["obj"]
                        ctx.plot(np.array(sample_list[0]), np.array(sample_list[c]), sampleCtx[c]["color"], linewidth=1, label=sampleCtx[c]["title"])
                        handles, labels = ctx.get_legend_handles_labels()
                        ctx.legend(handles, labels)
                        ctx.grid(True)

                    plt.pause(0.5)

                    if stop == False:
                        for c in range(1, len(sampleCtx)):
                            ctx = sampleCtx[c]["obj"]
                            ctx.clear()
                    else:
                        for c in range(1, len(sampleCtx)):
                            ctx = sampleCtx[c]["obj"]
                            ctx.plot(np.array(sample_list[0]), np.array(sample_list[c]), sampleCtx[c]["color"], linewidth=1, label=sampleCtx[c]["title"])
                            handles, labels = ctx.get_legend_handles_labels()
                            ctx.legend(handles, labels)
                            ctx.grid(True)
                
                        while stop:
                            plt.pause(0.5)

                            # Log to file ?
                            if args.log :
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
                                            
                                        # Log to file
                                        csvFile.write(data.decode("utf-8")+"\r\n")

# Main
if __name__ == '__main__':    
    main()

# EOF
