import serial
from time import sleep

port = serial.Serial("COM5",baudrate=115200,timeout=3.0)

while True:
    dir_byte = int(128)
    thr_byte = int(128)
    mode_byte = int(1)
    port.write("{:d};".format(dir_byte).encode('ascii')
               +"{:d};".format(thr_byte).encode('ascii')
               +"{:d}\r\n".format(mode_byte).encode('ascii'))
    line = port.readline()
    #print(str(line))
    fields = line.decode('ascii').split(';')
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

