import serial
import io
from time import sleep

port = serial.Serial("/dev/ttyUSB0",baudrate=115200,timeout=0.1,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
print(port.isOpen())
#sio = io.TextIOWrapper(io.BufferedRWPair(port,port))
while True:
    dir_byte = int(128)
    thr_byte = int(128)
    mode_byte = int(1)
    port.write("{:d};".format(dir_byte).encode('ascii')
               +"{:d};".format(thr_byte).encode('ascii')
               +"{:d}\r\n".format(mode_byte).encode('ascii'))
    #port.write( b'0;0;0' + b'13' + b'10')
    #sio.write("0;0;0\r\n")
    #sio.flush()
    line = port.readline()
    #print(line.__len__())
    #print(str(line))
    fields = line.decode('ascii').split(';')
    #fields = line.split(';')
    if(len(fields)>=8):
      lidar_distance_gauche = int(fields[0])
      lidar_distance_droit = int(fields[1])
      telemetry_speed = int(fields[2])
      telemetry_manual_dir = int(fields[3])
      telemetry_manual_thr = int(fields[4])
      telemetry_auto_dir = int(fields[5])
      telemetry_auto_thr = int(fields[6])
      telemetry_mode = int(fields[7])   
      print("LiG:" + str(lidar_distance_gauche)+" LiD:" + str(lidar_distance_droit))
    sleep(1.0/55.0)

