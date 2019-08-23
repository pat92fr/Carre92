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

# START CPATURE button
########################
def START_CAPTURE():
    print('START_CAPTURE')

# Main procedure
################
def main():

    # Check parameters
    ##################
    parser = ArgumentParser()
    parser.add_argument("-p", "--port", dest="port",
                        help="port number", type=int, default=1977)
    parser.add_argument("-i", "--ip", dest="host",
                        help="IP address or host", type=str, default='localhost')
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
    sampleCtx[0]  = { "title": "time",         "unit": "[1/60s]",  "color": "w",    "enable": 1, "factor": 1.0}  
    sampleCtx[1]  = { "title": "test param_1", "unit": "",         "color": "--k",  "enable": 1, "factor": 5.0}
    sampleCtx[2]  = { "title": "test param_2", "unit": "[mm]",     "color": "m",    "enable": 1, "factor": 1.0}
    sampleCtx[3]  = { "title": "test param_3", "unit": "[mm/ms]",  "color": "-b",   "enable": 1, "factor": 1.0}
    paramNumber = len(sampleCtx)
    print('Number of parameters:' + str(paramNumber))

    for i in range(len(sampleCtx)):
        print (f"#  - sample id {i}: enable: {sampleCtx[i]['enable']} factor: {sampleCtx[i]['factor']} => {sampleCtx[i]['title']} {sampleCtx[i]['unit']}")
    print ("")

    # Specify plot range
    range_plot = range(1, 3)

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
    f1, ax1 = plt.subplots()

    # Main while loop
    while True:
        # Wait for response
        data = client.recv(1024)
        if len(data)==0:
            try:	  
                ax1.grid(True)
                plt.pause(1)
            except:
                sys.exit(0)
                print("# Exit by user")
        else:
            res = data.decode("utf-8").rstrip('\n').rstrip(' ').split(';')	
            
        if res == None:
            continue	
            
        for i in range(1, paramNumber):
            if sampleCtx[i]["enable"] == 1:
                sample_list[i][nbSample] = float(res[i]) * sampleCtx[i]["factor"]
        
        # Speed up the display 
        if (nbSample % 5) == 0:
		  # Plot 1 
          try: 
              ax1.clear()
              for i in range_plot:
                if sampleCtx[i]["enable"] == 1:
                    txt_color = "%s" % (sampleCtx[i]["color"])
                    ax1.plot(np.array(sample_list[0]), np.array(sample_list[i]), txt_color, linewidth=1)
              plt.pause(0.01)
          except Exception as e:
              print(f"# Error: {e}")
              traceback.print_exc(file=sys.stdout)
              sys.exit(0)
			
        # Sample rollover
        nbSample += 1
        if nbSample >= windowSize:
            ax1.clear()
            # Plot 
            for i in range_plot:
                if sampleCtx[i]["enable"] == 1:
                    ax1.plot(np.array(sample_list[0]), np.array(sample_list[i]), sampleCtx[i]["color"], linewidth=1, label=sampleCtx[i]["title"])
                    handles, labels = ax1.get_legend_handles_labels()
                    ax1.legend(handles, labels)
                    ax1.grid(True)
                    txt_label = "Buggy telemetry [%s]" % (datetime.datetime.now())
                    ax1.set_title(txt_label)

            nbSample = 0
        
    print('Deconnexion.')
    client.close()

# Main
if __name__ == '__main__':    
    main()


# EOF
