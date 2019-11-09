# Import
import sys
import zmq
import zlib
import pickle
import numpy
from datetime import datetime

def cleanup():
    """Cleanup function"""
    socket_lidar.close()
    socket_speed.close()
    socket_serial.close()
    context.term()
    sys.exit(0)

def recv_zipped_pickle(socket, flags=0, protocol=pickle.HIGHEST_PROTOCOL):
    """inverse of send_zipped_pickle"""
    z = socket.recv(flags)
    p = zlib.decompress(z)
    return pickle.loads(p)

def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    buf = memoryview(msg)
    A = numpy.frombuffer(buf, dtype=md['dtype'])
    return A.reshape(md['shape'])

# Debug
print('Current libzmq version is %s' % zmq.zmq_version())
print('Current  pyzmq version is %s' % zmq.__version__)

#  ZMQ context
context = zmq.Context()

# Socket in SUBscribe mode for lidar telemetry reception
socket_lidar = context.socket(zmq.SUB)
socket_lidar.connect('tcp://localhost:5556')
# SUBscribe to all topics i.e. the subscriber want to process all the message from the publisher
socket_lidar.setsockopt(zmq.SUBSCRIBE, b'')
# Debug
nbMsgLidar = 0

# Socket in SUBscribe mode for speed telemetry reception
socket_speed = context.socket(zmq.SUB)
socket_speed.connect('tcp://localhost:5557')
# SUBscribe to all topics i.e. the subscriber want to process all the message from the publisher
socket_speed.setsockopt(zmq.SUBSCRIBE, b'')
# Debug
nbMsgSpeed = 0

# Socket in SUBscribe mode for serial telemetry reception
socket_serial = context.socket(zmq.SUB)
socket_serial.connect('tcp://localhost:5558')
# SUBscribe to all topics i.e. the subscriber want to process all the message from the publisher
socket_serial.setsockopt(zmq.SUBSCRIBE, b'')
# Debug
nbMsgSerial = 0

# Create poller context
poll = zmq.Poller()
poll.register(socket_lidar,  zmq.POLLIN)
poll.register(socket_speed,  zmq.POLLIN)
poll.register(socket_serial, zmq.POLLIN)

# Debug
print('Main sink started at: %s' % datetime.now().strftime('%M:%S.%f'))

# Main loop
while True:

    # Try/catch : zmq polling error or user CTRL-C
    try:
        # Non-blocking wait for message from client
        sockets = dict(poll.poll(0))
    except zmq.ZMQError as e:
        print('ZMQ error (%s), stopping...' % e.strerror)
        print(e)
        cleanup()
    except KeyboardInterrupt:
        print('Server stopped by user, stopping...')
        cleanup()

    # Process serial telemetry
    if socket_serial in sockets:
        serialString = recv_zipped_pickle(socket_serial)
        nbMsgSerial += 1
        print('%s# Receive SERIAL telemetry (%d): %s' % (datetime.now().strftime('%M:%S.%f'), \
                                                         nbMsgSerial, \
                                                         serialString))
    # Process speed telemetry
    if socket_speed in sockets:
        speedValue = recv_zipped_pickle(socket_speed)
        nbMsgSpeed += 1
        print('%s# Receive SPEED telemetry (%d): %s' % (datetime.now().strftime('%M:%S.%f'), \
                                                        nbMsgSpeed, \
                                                        speedValue))
    # Process lidar telemetry
    if socket_lidar in sockets:
        lidarArray = recv_array(socket_lidar)
        nbMsgLidar += 1
        print('%s# Receive LIDAR telemetry (%d):' % (datetime.now().strftime('%M:%S.%f'), \
                                                     nbMsgLidar))
        print(lidarArray)

# Eof
