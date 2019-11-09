# encoding: utf-8

# Import
import zmq
import zlib
import pickle
import sys
import time
import numpy
from datetime import datetime
from random   import randrange

def cleanup():
    """Cleanup function"""
    socket.close()
    socket_control.close()
    context.term()
    sys.exit(0)

def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(dtype = str(A.dtype), shape = A.shape,)
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)

# ZMQ context
context = zmq.Context()

####################
### Publish part ###
####################

# Socket in PUBlisher mode for lidar telemetry transmission
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")

# Debug
nbMsgSent = 0

######################
### Subscribe part ###
######################
socket_control = context.socket(zmq.SUB)
socket_control.connect('tcp://localhost:5555')
# SUBscribe to all topics i.e. the subscriber want to process all the message from the publisher
socket_control.setsockopt(zmq.SUBSCRIBE, b'')

# Create poller context
poller = zmq.Poller()
poller.register(socket_control, zmq.POLLIN)

# Main loop
while True:
    
    # Build and publish string
    nbMsgSent += 1
    lidarArray = numpy.random.rand(3,2)
    print('%s# PUBLISH lidar telemetry (%d):' % (datetime.now().strftime('%M:%S.%f'), \
                                                 nbMsgSent))
    print(lidarArray)
    
    # Try/catch : zmq send error or user CTRL-C
    try:
        send_array(socket, lidarArray)
    except zmq.ZMQError as e:
        print('ZMQ error (%s), stopping...' % e.strerror)
        print(e)
        cleanup()
    except KeyboardInterrupt:
        print('Server stopped by user, stopping...')
        cleanup()

    # Send message every 10ms: 100Hz
    time.sleep(0.01)

    # Check for KILL message
    socks = dict(poller.poll(0))
    if socks.get(socket_control) == zmq.POLLIN:
        print('%s# Receive KILL message, stopping...' % (datetime.now().strftime('%M:%S.%f')))
        cleanup()

# Eof
