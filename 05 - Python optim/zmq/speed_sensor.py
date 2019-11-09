# Import
import zmq
import zlib
import pickle
import sys
import time
from datetime import datetime
from random   import randrange

def cleanup():
    """Cleanup function"""
    socket.close()
    context.term()
    sys.exit(0)

def send_zipped_pickle(socket, obj, flags=0, protocol=pickle.HIGHEST_PROTOCOL):
    """pickle an object, and zip the pickle before sending it"""
    p = pickle.dumps(obj, protocol)
    z = zlib.compress(p)
    return socket.send(z, flags=flags)

# ZMQ context
context = zmq.Context()
# Socket in PUBlisher mode for speed telemetry transmission
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5557")

# Debug
nbMsgSent = 0

# Main loop
while True:
    
    # Build and publish string
    nbMsgSent += 1
    speedValue = randrange(0, 100)
    print('%s# PUBLISH speed telemetry (%d): %i' % (datetime.now().strftime('%M:%S.%f'), \
                                                    nbMsgSent, \
                                                    speedValue))
    
    # Try/catch : zmq send error or user CTRL-C
    try:
        send_zipped_pickle(socket, speedValue)
    except zmq.ZMQError as e:
        print('ZMQ error (%s), stopping...' % e.strerror)
        print(e)
        cleanup()
    except KeyboardInterrupt:
        print('Server stopped by user, stopping...')
        cleanup()

    # Send message every 1 seconds: 1Hz
    time.sleep(1)

# Eof