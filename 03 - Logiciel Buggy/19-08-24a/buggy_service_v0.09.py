###from threading import Thread
import time
import cv2
import serial
import socket
import numpy as np

def gstreamer_pipeline (capture_width=1280, capture_height=720, display_width=160, display_height=90, framerate=60, flip_method=2) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))

# math tool
def constraint(variable, min, max):
    if variable>max:
        return max
    elif variable<min:
        return min
    else:
        return variable

# pid
class pid:
    def __init__(self,kp=0.0, ki=0.0, kd=0.0, integral_max=10000, output_max=1.0, alpha=0.0):
        # settings
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max  = integral_max
        self.output_max  = output_max
        self.alpha = alpha
        # state
        self.error = 0.0
        self.filtered_error = 0.0
        self.last_filtered_error = 0.0
        self.derivative = 0.0
        self.integral_error = 0.0
        self.output = 0.0
   
    def compute(self,error):
        # update
        self.error = error
        self.last_filtered_error = self.filtered_error
        self.filtered_error = self.filtered_error*(1.0-self.alpha) + self.alpha*self.error # EWMA
        self.derivative =  self.filtered_error -  self.last_filtered_error
        self.integral_error += self.error
        # windup (staturate)
        self.integral_error = constraint(self.integral_error, -self.integral_max, self.integral_max )
        # pid
        p = self.error * self.kp
        i = self.integral_error * self.ki
        d = self.derivative * self.kd
        # saturate and output
        self.output = constraint(p+i+d, -self.output_max, self.output_max)
        return self.output

class buggyServer:

    def __init__(self):

        # control 
        self.pid_camera_params = {'kp':1.5, 'kd':3.0, 'kd2':2.0, 'error':0.0, 'last_error':0.0, 'next_error':0.0 }
        self.pid_wall_following = pid(kp=0.01, ki=0.0, kd=0.0, integral_max=10000, output_max=1.0, alpha=0.3) 
        self.wall_threshold_min = 40
        self.wall_threshold_max = 160
        self.wall_distance_middle = 86
        self.direction = 128
        self.throttle = 128
        self.mode = 0

        # On met -1 pour qu'au premier calcul des pids, les Lidars ne soient pas pris en compte, car nous n'avons pas encore d'acquisition
        self.lidar_distance_gauche = 0
        self.lidar_distance_droit = 0
        self.telemetry_speed = 0
        self.telemetry_manual_dir = 0
        self.telemetry_manual_thr = 0
        self.telemetry_auto_dir = 0
        self.telemetry_auto_thr = 0
        self.telemetry_mode = 0  

        # global state
        self.run_ai = True

        self.s = 0
        self.model = 0
        self.cap_send = 0
        self.out_send = 0

        self.ia_started = False
        self.wall_following_started = True
        self.screenshot = False

        self.rec_width = 0
        self.rec_height = 0

        #self.pc_IP ='10.42.0.188' #PC Section Robotique !
        self.pc_IP ='10.42.0.112' #PC HP portable Patrick !
        #self.pc_IP ='192.168.1.34' #PC Home Patrick !
        #self.pc_IP ='192.168.1.10' #PC Home Patrick !
        #self.pc_IP ='10.0.10.50' #test card


        #serial port for STM32 communication 
        self.port = serial.Serial("/dev/ttyUSB0",baudrate=115200,timeout=3.0)

        self.y_1 = 0
        self.y_2 = 0

        self.image = 0
        self.video_mode = "hires"

        if self.video_mode == "hires":

            self.rec_width = 1280
            self.rec_height = 720   

            self.new_width = 160
            self.new_height = 90

        if self.video_mode == "lores":

            self.rec_width = 1280
            self.rec_height = 720

            self.new_width = 160
            self.new_height = 90


        self.init_socket()
        self.init_cam()
        self.load_configuration_file()

    def load_configuration_file(self):

        file = open('config.txt', 'r')
        for line in file:
            print(line)
            fields = line.split(' ')
#            if fields[0] == "WALL_AVOIDING":
#                self.pid_left_wall_avoiding_params['kp'] = float(fields[1])
#                self.pid_left_wall_avoiding_params['kd'] = float(fields[2])
#                self.pid_right_wall_avoiding_params['kp'] = float(fields[1])
#                self.pid_right_wall_avoiding_params['kd'] = float(fields[2])
            if fields[0] == "WALL_FOLLOWING":
                self.pid_wall_following.kp = float(fields[1])
                self.pid_wall_following.ki = float(fields[2])
                self.pid_wall_following.kd = float(fields[3])
        file.close()
        
    def init_socket(self):

        print("Init Socket...")

        HOST = ''  # Standard loopback interface address (localhost)
        PORT = 6000         # Port to listen on (non-privileged ports are > 1023)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as self.s:
            self.s.bind((HOST, PORT))
            print('listening...')
            self.s.listen()
            self.conn, addr = self.s.accept()
            print('self.connected by', addr)


    def init_wall_following(self):

        self.wall_following_started = True


    def init_IA(self):

        from keras.models import load_model
        #import h5py

        # load model
        self.model = load_model('bitume_model.h5')
        self.model.summary()

        self.ia_started = True

    def init_cam(self):

        print("Init Camera...")

	# create gstreamer capture pipeline
        self.cap_send = cv2.VideoCapture(gstreamer_pipeline(capture_width=self.rec_width, capture_height=self.rec_height, display_width=self.new_width, display_height=self.new_height, framerate=60, flip_method=2), cv2.CAP_GSTREAMER)
        return_value, self.image = self.cap_send.read()
        assert(self.image.shape == (self.new_height,self.new_width,3))

        # init gstreamer broadcast pipeline
        self.out_send = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=200 ! queue ! rtph264pay config-inerval=1 ! udpsink host='+self.pc_IP+' port=5500 sync=false',cv2.CAP_GSTREAMER,0, 20, (self.new_width,self.new_height), True)

    def image_processing(self):
        # parameters
        crop_height = 32
        height_start_1 = 0 
        height_end_1 = height_start_1+crop_height 
        height_start_2 = height_end_1
        height_end_2 = height_start_2+crop_height 

        # process image before IA

        # clean-up and grayscale
        image_smoothed= cv2.blur(self.image,(3,3))
        #########image_gray = cv2.cvtColor(image_smoothed, cv2.COLOR_RGB2GRAY)

        # crop lower and middle frames
        im_croped_1 = image_smoothed[self.new_height-height_end_1:self.new_height-height_start_1,:self.new_width]
        assert(im_croped_1.shape == (crop_height,self.new_width,3))
        im_croped_2 = image_smoothed[self.new_height-height_end_2:self.new_height-height_start_2,:self.new_width]
        assert(im_croped_2.shape == (crop_height,self.new_width,3))
        
        if self.screenshot:
            self.screenshot = False
            #### take screenshot here im_croped_2

        # CNN processing
        x_1 = im_croped_1.reshape(crop_height,self.new_width,3)/255.0
        x_2 = im_croped_2.reshape(crop_height,self.new_width,3)/255.0
        x = np.array([x_1, x_2])
        y = self.model.predict(x)
        self.y_1, self.y_2 = y.item(0), y.item(1)
        ###print(str(self.y_1))
        ###print(str(self.y_2))
        
        # draw line on image
        pt1_x = int((self.y_1+1.0)/2.0*self.new_width)
        pt1_y = int((self.new_height-height_start_1-(crop_height/2)))
        pt2_x = int((self.y_2+1.0)/2.0*self.new_width)
        pt2_y = int((self.new_height-height_end_2+(crop_height/2)))
        ###print ('coordonées : pt1({}, {}), pt2({},{})'.format(pt1_x,pt1_y,pt2_x,pt2_y))
        cv2.line(self.image,(pt1_x,pt1_y),(pt2_x,pt2_y),(255,0,0),5)

    def buggyservice(self):

        # game loop
        last_time = time.time()
        frame_counter = 0
        fps = 0
        
        while(True):
            #value = 2
            self.conn.send(fps.to_bytes(2, byteorder='big') + self.lidar_distance_gauche.to_bytes(2, byteorder='big', signed=True) + self.lidar_distance_droit.to_bytes(2, byteorder='big', signed=True) )
            command = self.conn.recv(1024)

            if (command == b's'):
                print("Screenshot!")
                self.screenshot = True

            if (command == b'i'):
                if self.ia_started:
                    print("IA already stared")
                else:
                    print("Initializing IA")
                    self.init_IA()

            if (command == b'w'):
                if self.wall_following_started:
                    print("Wall following already stared")
                else:

                    print("Initializing Wall following")
                    self.init_wall_following()

            if (command == b'a'):
                    print("Reload configuration file")
                    self.load_configuration_file()

            if (command == b'q'):
                print("All done, quitting !")
                self.s.close()
                self.cap_send.release()
                self.out_send.release()
                return

            return_value, self.image = self.cap_send.read()
            assert(self.image.shape == (self.new_height,self.new_width,3))

            self.direction = 128
            self.throttle = 128
            self.mode = 0
            if self.ia_started:
                self.image_processing()
                
                #pid management
                self.pid_camera_params['last_error'] = self.pid_camera_params['error']
                self.pid_camera_params['error'] = self.pid_camera_params['error']*0.5 + 0.5*self.y_1
                self.pid_camera_params['next_error'] = self.pid_camera_params['next_error']*0.5 + 0.5*self.y_2
                pid_ai = self.pid_camera_params['error'] * self.pid_camera_params['kp'] + (self.pid_camera_params['error']-self.pid_camera_params['last_error']) * self.pid_camera_params['kd'] + (self.pid_camera_params['next_error']-self.pid_camera_params['error']) * self.pid_camera_params['kd2'] 
                if pid_ai>1.0:
                    pid_ai=1.0
                elif pid_ai<-1.0:
                    pid_ai=-1.0

                # direction & throttle control
                self.direction = int(((pid_ai+1.0)/2.0*255.0))
                self.throttle = 128
                self.mode = 1
                
            if self.wall_following_started:
                # direction & throttle control

                # update PID according LIDAR inputs
                error = constraint(self.lidar_distance_droit - self.lidar_distance_gauche, -150, 150)
                pid_lidar = self.pid_wall_following.compute( error )
                print("error: " + str(error) + " pid_lidar: " + str(pid_lidar))
                
                self.direction = int(((pid_lidar+1.0)/2.0*255.0))   
                self.throttle = 128
                self.mode = 1

            #print("fps:" + str(fps) + "   DIR:" + str(self.direction) + "   THR:" + str(self.throttle) + "   MODE:" + str(self.mode))
            self.port.write("{:d};".format(self.direction).encode('ascii') + "{:d};".format(self.throttle).encode('ascii') + "{:d}\r\n".format(self.mode).encode('ascii'))
            serial_line = self.port.readline()
            ###print(str(serial_line))
            fields = serial_line.decode('ascii').split(';')
            if(len(fields)>=8):
               self.lidar_distance_gauche = int(fields[0])
               self.lidar_distance_droit = int(fields[1])
               self.telemetry_speed = int(fields[2])
               self.telemetry_manual_dir = int(fields[3])
               self.telemetry_manual_thr = int(fields[4])
               self.telemetry_auto_dir = int(fields[5])
               self.telemetry_auto_thr = int(fields[6])
               self.telemetry_mode = int(fields[7])   
               ###print("LiG:" + str(self.lidar_distance_gauche)+" LiD:" + str(self.lidar_distance_droit))

            self.out_send.write(self.image)

            #fps counter
            frame_counter += 1
            if time.time()>=last_time+1.0:
                last_time=time.time()
                fps = frame_counter
                frame_counter = 0

def main():

    serv = buggyServer()
    serv.buggyservice()

if __name__ == "__main__":
    main()
