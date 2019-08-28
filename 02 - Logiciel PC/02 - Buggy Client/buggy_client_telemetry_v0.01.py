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
    # G1
    sampleCtx[0]  = { "title": "time",         "unit": "[1/60s]",  "color": "w",    "enable": 1, "factor": 1.0}  
    sampleCtx[1]  = { "title": "LIG",          "unit": "cm",       "color": "--k",  "enable": 1, "factor": 1.0}
    sampleCtx[2]  = { "title": "LID",          "unit": "cm",       "color": ":g",   "enable": 1, "factor": 1.0}
    sampleCtx[3]  = { "title": "LIH",          "unit": "cm",       "color": "-b",  "enable": 1, "factor": 1.0}
    sampleCtx[4]  = { "title": "LIDAR error",  "unit": "cm",       "color": "--r",   "enable": 1, "factor": 1.0}
    range_plot1 = range(1, 4)

    # G2
    sampleCtx[5]  = { "title": "LIDAR error2", "unit": "cm",       "color": "m",    "enable": 1, "factor": 1.0}
    range_plot2 = range(5, 5)

    # G3
    sampleCtx[6]  = { "title": "PID Lidar", "unit": "",            "color": "m",    "enable": 1, "factor": 1.0}
    sampleCtx[7]  = { "title": "PID AI", "unit": "",               "color": "m",    "enable": 1, "factor": 1.0}
    range_plot3 = range(6, 7)

    # G4          
    sampleCtx[8]  = { "title": "Direction", "unit": "",            "color": "-b",   "enable": 1, "factor": 1.0}
    sampleCtx[9]  = { "title": "Gaz", "unit": "",                  "color": "-b",   "enable": 1, "factor": 1.0}
    range_plot4 = range(8, 9)

    # G5
    sampleCtx[10]  = { "title": "Mode", "unit": "",                 "color": "-b",   "enable": 1, "factor": 1.0}
    range_plot5 = range(10, 10)
    
    paramNumber = len(sampleCtx)
    print('Number of parameters:' + str(paramNumber))

    for i in range(len(sampleCtx)):
        print (f"#  - sample id {i}: enable: {sampleCtx[i]['enable']} factor: {sampleCtx[i]['factor']} => {sampleCtx[i]['title']} {sampleCtx[i]['unit']}")
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
    f1, ax1 = plt.subplots()
    f2, ax2 = plt.subplots()
    f3, ax3 = plt.subplots()
    f4, ax4 = plt.subplots()
    f5, ax5 = plt.subplots()

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
            res = data.decode("utf-8").rstrip('\n').rstrip(' ').split('\n')

        print(res)
        print(len(res))
        
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
              for i in range_plot1:
                if sampleCtx[i]["enable"] == 1:
                    txt_color = "%s" % (sampleCtx[i]["color"])
                    ax1.plot(np.array(sample_list[0]), np.array(sample_list[i]), txt_color, linewidth=1)
              plt.pause(0.01)
          except Exception as e:
              print(f"# Error: {e}")
              traceback.print_exc(file=sys.stdout)
              sys.exit(0)
	# Plot 2 
          try: 
              ax2.clear()
              for i in range_plot2:
                if sampleCtx[i]["enable"] == 1:
                    txt_color = "%s" % (sampleCtx[i]["color"])
                    ax2.plot(np.array(sample_list[0]), np.array(sample_list[i]), txt_color, linewidth=1)
              plt.pause(0.01)
          except Exception as e:
              print(f"# Error: {e}")
              traceback.print_exc(file=sys.stdout)
              sys.exit(0)
	# Plot 3 
          try: 
              ax3.clear()
              for i in range_plot3:
                if sampleCtx[i]["enable"] == 1:
                    txt_color = "%s" % (sampleCtx[i]["color"])
                    ax3.plot(np.array(sample_list[0]), np.array(sample_list[i]), txt_color, linewidth=1)
              plt.pause(0.01)
          except Exception as e:
              print(f"# Error: {e}")
              traceback.print_exc(file=sys.stdout)
              sys.exit(0)
	# Plot 4 
          try: 
              ax4.clear()
              for i in range_plot4:
                if sampleCtx[i]["enable"] == 1:
                    txt_color = "%s" % (sampleCtx[i]["color"])
                    ax4.plot(np.array(sample_list[0]), np.array(sample_list[i]), txt_color, linewidth=1)
              plt.pause(0.01)
          except Exception as e:
              print(f"# Error: {e}")
              traceback.print_exc(file=sys.stdout)
              sys.exit(0)
	# Plot 5
          try: 
              ax5.clear()
              for i in range_plot5:
                if sampleCtx[i]["enable"] == 1:
                    txt_color = "%s" % (sampleCtx[i]["color"])
                    ax5.plot(np.array(sample_list[0]), np.array(sample_list[i]), txt_color, linewidth=1)
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
            for i in range_plot1:
                if sampleCtx[i]["enable"] == 1:
                    ax1.plot(np.array(sample_list[0]), np.array(sample_list[i]), sampleCtx[i]["color"], linewidth=1, label=sampleCtx[i]["title"])
                    handles, labels = ax1.get_legend_handles_labels()
                    ax1.legend(handles, labels)
                    ax1.grid(True)
                    txt_label = "Buggy telemetry [%s]" % (datetime.datetime.now())
                    ax1.set_title(txt_label)

            ax2.clear()
            # Plot 
            for i in range_plot2:
                if sampleCtx[i]["enable"] == 1:
                    ax2.plot(np.array(sample_list[0]), np.array(sample_list[i]), sampleCtx[i]["color"], linewidth=1, label=sampleCtx[i]["title"])
                    handles, labels = ax2.get_legend_handles_labels()
                    ax2.legend(handles, labels)
                    ax2.grid(True)
                    txt_label = "Buggy telemetry [%s]" % (datetime.datetime.now())
                    ax2.set_title(txt_label)

            ax3.clear()
            # Plot 
            for i in range_plot1:
                if sampleCtx[i]["enable"] == 1:
                    ax3.plot(np.array(sample_list[0]), np.array(sample_list[i]), sampleCtx[i]["color"], linewidth=1, label=sampleCtx[i]["title"])
                    handles, labels = ax3.get_legend_handles_labels()
                    ax3.legend(handles, labels)
                    ax3.grid(True)
                    txt_label = "Buggy telemetry [%s]" % (datetime.datetime.now())
                    ax3.set_title(txt_label)

            ax4.clear()
            # Plot 
            for i in range_plot1:
                if sampleCtx[i]["enable"] == 1:
                    ax4.plot(np.array(sample_list[0]), np.array(sample_list[i]), sampleCtx[i]["color"], linewidth=1, label=sampleCtx[i]["title"])
                    handles, labels = ax4.get_legend_handles_labels()
                    ax4.legend(handles, labels)
                    ax4.grid(True)
                    txt_label = "Buggy telemetry [%s]" % (datetime.datetime.now())
                    ax4.set_title(txt_label)

            ax5.clear()
            # Plot 
            for i in range_plot1:
                if sampleCtx[i]["enable"] == 1:
                    ax5.plot(np.array(sample_list[0]), np.array(sample_list[i]), sampleCtx[i]["color"], linewidth=1, label=sampleCtx[i]["title"])
                    handles, labels = ax5.get_legend_handles_labels()
                    ax5.legend(handles, labels)
                    ax5.grid(True)
                    txt_label = "Buggy telemetry [%s]" % (datetime.datetime.now())
                    ax5.set_title(txt_label)

            while 1:
                nbSample = 0
        
    print('Deconnexion.')
    client.close()

# Main
if __name__ == '__main__':    
    main()


# EOF
