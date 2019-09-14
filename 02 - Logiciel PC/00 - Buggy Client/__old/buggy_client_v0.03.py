#-*- coding: utf-8 -*-

import socket
import sys
import os
from tkinter  import *
from argparse import ArgumentParser

# Globals
global client
global paramCtx

# Commands & misc
CMD_GET_API  = 'GET_API'
CMD_GET      = 'GET'
CMD_SET      = 'SET'
CMD_KILL     = 'KILL'
CMD_IA       = 'IA'
CMD_REC_START= 'REC_START'
CMD_REC_STOP = 'REC_STOP'
CMD_RUN_START= 'RUN_START'
CMD_RUN_STOP = 'RUN_STOP'
CMD_SPLIT    = ';'
FILE_NAME    = './buggy_file_results.txt'

# Command handlers
def GET():
    print('GET')
    msg = CMD_GET
    print('# Sending:' + msg)
    n = client.send(msg.encode('utf-8'))
    if (n != len(msg)):
        print('Unable to send command.')
        return
    data = client.recv(1024)
    print('# Receive:' + data.decode("utf-8") )
    res = data.decode("utf-8").rstrip(CMD_SPLIT).split(CMD_SPLIT)
    if (res[0]!=CMD_GET):
        print('ERROR: bad response to get_api command: {res}')
        return
    for p in range(nbParameters):
        ctx = paramCtx[p]['object']
        ctx.delete(0,END)
        ctx.insert(0,res[p + 1])

def SET():
    print('SET')
    msg = CMD_SET
    for p in range(nbParameters):
        ctx = paramCtx[p]['object']
        msg += ";" + ctx.get()
    print('# Sending:' + msg)
    n = client.send(msg.encode('utf-8'))
    if (n != len(msg)):
        print('Unable to send command.')
        return

def SAVE():
    print('SAVE')
    # Clean the place
    fileResult = open(FILE_NAME, "w").close()
    fileResult = open(FILE_NAME, "w")
    for p in range(0, len(paramCtx) - 1):
        ctx = paramCtx[p]['object']
        fileResult.write("%s=%s\r\n" % (paramCtx[p]['label'], ctx.get()))
    fileResult.close()

def READ():
    print('READ')
    if os.path.exists(FILE_NAME) == False:
        print('# File is not present, unable to read parameters: ' + FILE_NAME)
        return
    else:
        fileResult = open(FILE_NAME, "r")
        line = fileResult.readline()
        while line:
            res = line.replace(' ', '').strip().split('=')
            if (len(res)==2):
                for p in range(0, len(paramCtx) - 1):
                    if paramCtx[p]['label'] == res[0]:
                        ctx = paramCtx[p]['object']
                        ctx.delete(0,END)
                        ctx.insert(0,res[1])
            line = fileResult.readline()
        fileResult.close()

def KILL():
    print('KILL')
    msg = CMD_KILL
    print('# Sending:' + msg)
    n = client.send(msg.encode('utf-8'))
    if (n != len(msg)):
        print('Unable to send command.')
        return

def IA():
    print('IA')
    msg = CMD_IA
    print('# Sending:' + msg)
    n = client.send(msg.encode('utf-8'))
    if (n != len(msg)):
        print('Unable to send command.')
        return

def REC_START():
    print('REC_START')
    msg = CMD_REC_START
    print('# Sending:' + msg)
    n = client.send(msg.encode('utf-8'))
    if (n != len(msg)):
        print('Unable to send command.')
        return
    
def REC_STOP():
    print('REC_STOP')
    msg = CMD_REC_STOP
    print('# Sending:' + msg)
    n = client.send(msg.encode('utf-8'))
    if (n != len(msg)):
        print('Unable to send command.')
        return
    
# Check parameters
parser = ArgumentParser()
parser.add_argument("-p", "--port", dest="port",
                    help="port number", type=int, default=6000)
parser.add_argument("-i", "--ip", dest="host",
                    help="IP address or host", type=str, default='10.42.0.1')
args = parser.parse_args()

# Try the connection to the server
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    client.connect((args.host, args.port))
except:
    print('Connection #KO# to ' + args.host + ':' + str(args.port))
    sys.exit(0)

print('Connection OK to ' + args.host + ':' + str(args.port))

# Check if file result is present
paramFile = {}
nbParamtersFromFile = 0

if os.path.exists(FILE_NAME) == False:
    print('# File is not present, unable to read parameters: ' + FILE_NAME)
else:
    print('# File is present, parameters from: ' + FILE_NAME)
    # Open the file
    fileResult = open(FILE_NAME, "r")
    line = fileResult.readline()
    while line:
        res = line.replace(' ', '').strip().split('=')
        if (len(res)==2):
            print(res)
            paramFile[nbParamtersFromFile] = {'label': res[0], 'value': res[1]}
            nbParamtersFromFile += 1
        line = fileResult.readline()
    fileResult.close()
    print('Number of parameters present in file: ' + str(nbParamtersFromFile))

# First command : GET_API in order to get the list of parameters
msg = CMD_GET_API
print('# Sending:' + msg)
n = client.send(msg.encode('utf-8'))
if (n != len(msg)):
    print('Unable to send command.')
    sys.exit(0)
# Wait for response
data = client.recv(1024)
print('# Receive:' + data.decode("utf-8") )
# Check response and get the number of parameters
res = data.decode("utf-8").rstrip(CMD_SPLIT).split(CMD_SPLIT)
if (res[0]!=CMD_GET_API):
    print('ERROR: bad response to get_api command: {res}')
    sys.exit(0)

nbParameters = len(res) - 1
print('Number of parameters from buggy server: ' + str(nbParameters))

if (nbParamtersFromFile!=0) and (nbParamtersFromFile != nbParameters):
    print('ERROR: number of parameters differs. Compare file/bugger server or suppress file')
    sys.exit(0)

paramCtx = {}
for p in range(0, nbParameters):
    paramCtx[p] = {'label': res[1+p], 'value': 0.0, 'object': None}

# Check param name between file and buggy server
if nbParamtersFromFile!=0:
    for p in range(0, nbParameters):
        found = False
        for pp in range(0, nbParameters):
            if paramCtx[p]['label'] == paramFile[pp]['label']:
                found = True
        if found == False:
            print('ERROR: parameter name differ between file and buggy server')
            sys.exit(0)

# Display host and port
paramCtx[nbParameters] = {'label': 'host/port', 'value': args.host + " " + str(args.port), 'object': None}

# Main frame for get/set and save action
frame = Tk()
frame.title('GET/SET parameters & commands')

# All parameters
for p in range(0, nbParameters):
    Label(frame, text=paramCtx[p]['label']).grid(row=p, column=0)
    paramCtx[p]['object'] = Entry(frame)
    paramCtx[p]['object'].grid(row=p, column=1)

# Host & IP
Label(frame, text=paramCtx[nbParameters]['label']).grid(row=nbParameters, column=0)
paramCtx[nbParameters]['object'] = Entry(frame)
paramCtx[nbParameters]['object'].grid(row=nbParameters, column=1)
paramCtx[nbParameters]['object'].delete(0,END)
paramCtx[nbParameters]['object'].insert(0, paramCtx[nbParameters]['value'])

Button(frame, text="Read buggy", command=GET ).grid(row=(nbParameters),      column=2)
Button(frame, text="Read file ", command=READ).grid(row=(nbParameters),      column=3)		
Button(frame, text="Save buggy", command=SET ).grid(row=(nbParameters),      column=4)
Button(frame, text="Save file ", command=SAVE).grid(row=(nbParameters),      column=5)
Button(frame, text="IA        ", command=IA).grid(row=(nbParameters),        column=6)
Button(frame, text="REC start ", command=REC_START).grid(row=(nbParameters), column=7)
Button(frame, text="REC stop  ", command=REC_STOP).grid(row=(nbParameters),  column=8)
Button(frame, text="KILL      ", command=KILL).grid(row=(nbParameters),      column=9)

# Request the values
frame.update_idletasks()

# Main loop entry
frame.mainloop()

print('Deconnexion.')
client.close()

# EOF
