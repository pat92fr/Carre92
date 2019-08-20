from threading import Thread
import time
import sys
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

def compute_wall_pid(pid_param, right, left):
    pid_param['last_error'] = pid_param['error']
    pid_param['error'] = right - left
    print("  error:" + str(pid_param['error'])) 
    pid = pid_param['error'] * pid_param['kp'] + (pid_param['error']-pid_param['last_error']) * pid_param['kd'] 
    if pid>1.0:
        pid = 1.0
    elif pid<-1.0:
        pid = -1.0
    return pid

class buggyServer:

    def __init__(self):

        # control 
        self.pid_camera_params = {'kp':1.5, 'kd':3.0, 'kd2':2.0, 'error':0.0, 'last_error':0.0, 'next_error':0.0 }
        self.pid_left_wall_avoiding_params = {'kp':0.08, 'kd':0.0, 'error':0.0, 'last_error':0.0 }
        self.pid_right_wall_avoiding_params = {'kp':0.08, 'kd':0.0, 'error':0.0, 'last_error':0.0 }
        self.pid_both_wall_following_params = {'kp':0.03, 'kd':0.0, 'error':0.0, 'last_error':0.0 }
        self.pid_left_wall_following_params = {'kp':0.03, 'kd':0.0, 'error':0.0, 'last_error':0.0 }
        self.pid_right_wall_following_params = {'kp':0.02, 'kd':0.0, 'error':0.0, 'last_error':0.0 }
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
        self.wall_following_started = False
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

        self.pid_left_wall_avoiding_params['kp'] = 0.08
        self.pid_left_wall_avoiding_params['kd'] = 0.00
        
        self.pid_right_wall_avoiding_params['kp'] = 0.08
        self.pid_right_wall_avoiding_params['kd'] = 0.00
        
        self.pid_both_wall_following_params['kp'] = 0.03
        self.pid_both_wall_following_params['kd'] = 0.00
        
        self.pid_left_wall_following_params['kp'] = 0.03
        self.pid_left_wall_following_params['kd'] = 0.00

        self.pid_right_wall_following_params['kp'] = 0.03
        self.pid_right_wall_following_params['kd'] = 0.00
        
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
        import h5py

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

        while True:

            while(True):
                value = 2
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
                    pid_left_wall_avoiding = compute_wall_pid(self.pid_left_wall_avoiding_params, self.wall_threshold_min, self.lidar_distance_gauche)
                    pid_right_wall_avoiding = compute_wall_pid(self.pid_right_wall_avoiding_params, self.lidar_distance_droit, self.wall_threshold_min)
                    pid_left_wall_following = compute_wall_pid(self.pid_left_wall_following_params, self.wall_distance_middle,self.lidar_distance_gauche)
                    pid_right_wall_following = compute_wall_pid(self.pid_right_wall_following_params, self.lidar_distance_droit, self.wall_distance_middle)
                    pid_both_wall_following = compute_wall_pid(self.pid_both_wall_following_params, self.lidar_distance_droit, self.lidar_distance_gauche)
                    
                    # select best PID output according position
                    if (self.lidar_distance_droit != -1) and (self.lidar_distance_droit < self.wall_threshold_min):
                        # avoiding right wall
                        print("avoiding right wall: " + str(pid_right_wall_avoiding))
                        self.direction = int(((pid_right_wall_avoiding+1.0)/2.0*255.0))   
                        self.throttle = 128
                        self.mode = 1
                    elif (self.lidar_distance_gauche != -1) and (self.lidar_distance_gauche < self.wall_threshold_min):
                        # avoiding left wall
                        print("avoiding left wall: " + str(pid_left_wall_avoiding))
                        self.direction = int(((pid_left_wall_avoiding+1.0)/2.0*255.0))   
                        self.throttle = 128
                        self.mode = 1
                    elif (self.lidar_distance_droit >= self.wall_threshold_min) and (self.lidar_distance_droit < self.wall_threshold_max) \
                    and (self.lidar_distance_gauche >= self.wall_threshold_min) and (self.lidar_distance_gauche < self.wall_threshold_max):
                        # wall following with both walls
                        print("wall following with both walls: " + str(pid_both_wall_following))
                        self.direction = int(((pid_both_wall_following+1.0)/2.0*255.0)) 
                        self.throttle = 128
                        self.mode = 1
                    elif (self.lidar_distance_gauche >= self.wall_threshold_min) and (self.lidar_distance_gauche < self.wall_threshold_max):
                        # wall following with left wall only
                        print("wall following with left wall only: " + str(pid_left_wall_following))
                        self.direction = int(((pid_left_wall_following+1.0)/2.0*255.0))   
                        self.throttle = 128
                        self.mode = 1
                    elif (self.lidar_distance_droit >= self.wall_threshold_min) and (self.lidar_distance_droit < self.wall_threshold_max):
                        # wall following with right wall only
                        print("wall following with right wall only: " + str(pid_right_wall_following))
                        self.direction = int(((pid_right_wall_following+1.0)/2.0*255.0))     
                        self.throttle = 128
                        self.mode = 1
                    else:	
                        # go straight without wall following
                        print("go straight without wall following: " + str(128))
                        self.direction = 128
                        self.throttle = 128
                        self.mode = 0

                print("fps:" + str(fps) + "   DIR:" + str(self.direction) + "   THR:" + str(self.throttle) + "   MODE:" + str(self.mode))
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
