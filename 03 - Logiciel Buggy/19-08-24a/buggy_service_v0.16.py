import time
import cv2
import serial
import socket
import asyncore
import numpy as np
# my modules
import my_controller
from my_math import  *

# global (for client-server communication)
global telemetryClient
global param_kp
global param_ki
global param_kd
global param_max_speed
global param_min_speed
global param_k_speed
global param_positional_error_threshold
global param_dual_rate
global param_direction_trim
global command_id # received command from client 
    # CID =  0 : default/idle
    # CID =  1 : kill buggy server
    # CID =  2 : start Line Following with CNN
    # CID =  3 : start recording video and data
    # CID =  4 : stop recording video and data 
    # CID =  5 : start run
    # CID =  6 : stop run
    # CID = 10 : reload settings (globals)

def load_configuration_file():
    global param_kp
    global param_ki
    global param_kd        
    global param_max_speed       
    global param_min_speed       
    global param_k_speed      
    global param_positional_error_threshold 
    global param_dual_rate
    global param_direction_trim
    file = open('config.txt', 'r')
    for line in file:
        print(line)
        fields = line.split(' ')
        if fields[0] == "WALL_FOLLOWING":
            param_kp = float(fields[1])
            param_ki = float(fields[2])
            param_kd = float(fields[3])
        if fields[0] == "SPEED":
            param_min_speed =  int(fields[1])
            param_max_speed =  int(fields[2])
            param_k_speed =  float(fields[3])
            param_positional_error_threshold =  int(fields[4])
        if fields[0] == "DIRECTION":
            param_direction_trim =  int(fields[1])
            param_dual_rate =  float(fields[2])
    file.close()
        
# constants (for client-server communication)
CMD_GET_API  = 'GET_API'
CMD_GET         = 'GET'
CMD_SET         = 'SET'
CMD_MODE     = 'MODE'
CMD_SPLIT      = ';'

def gstreamer_pipeline (capture_width=1280, capture_height=720, display_width=160, display_height=90, framerate=60, flip_method=2) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))

class buggyHandler(asyncore.dispatcher_with_send):
    
    def handle_read(self):
        global param_kp
        global param_ki
        global param_kd
        global param_min_speed
        global param_max_speed
        global param_k_speed
        global param_positional_error_threshold
        global param_dual_rate
        global param_direction_trim        
        global command_id

        data = self.recv(8192)
        if data:
            print('Receive command: ' + data.decode("utf-8"))
            res = data.decode("utf-8").rstrip(CMD_SPLIT).split(CMD_SPLIT)
            print(res)

            if (res[0] == "GET_API"):
                resp = "GET_API;kp;ki;kd;min_speed;max_speed;k_speed;positional_error_threshold;dual_rate;direction_trim"
                print(resp)
                self.send(resp.encode("utf-8"))
                print('Response sent.')
                return

            if (res[0] == "GET"):
                resp = "GET;" + str(param_kp) + ";" + str(param_ki) + ";"  + str(param_kd) + ";"  + str(param_min_speed) + ";"  + str(param_max_speed) + ";"  + str(param_k_speed) + ";"  + str(param_positional_error_threshold) + ";"  + str(param_dual_rate) + ";"  + str(param_direction_trim)
                print(resp)
                self.send(resp.encode("utf-8"))
                print('Response sent.')
                return

            if (res[0] == "SET"):
                param_kp = float(res[1])
                param_ki = float(res[2])
                param_kd = float(res[3])
                param_min_speed = float(res[4])
                param_max_speed = float(res[5])
                param_k_speed = float(res[6])
                param_positional_error_threshold = int(res[7])
                param_dual_rate = float(res[8])
                param_direction_trim = int(res[9])
                command_id = 10
                return        

            if (res[0] == "KILL"):
                command_id = 1
                return

            if (res[0] == "IA"):
                command_id = 2
                return

            if (res[0] == "REC_START"):
                command_id = 3
                return

            if (res[0] == "REC_STOP"):
                command_id = 4
                return

            if (res[0] == "RUN_START"):
                command_id = 5
                return

            if (res[0] == "RUN_STOP"):
                command_id = 6
                return
                
                
                
'''''''''''''''''''''''''''''
    Main telemetry handler
'''''''''''''''''''''''''''
class buggyTelemetryHandler(asyncore.dispatcher_with_send):
  
    def handle_read(self):
        pass

    def handle_close(self):
        global telemetryClient
        print('Telemetry> connection closed.')
        telemetryClient = False
        self.close()

'''''''''''''''''''''''
    Server telemetry
'''''''''''''''''''''
class buggyTelemetryServer(asyncore.dispatcher):
    
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
    
    def handle_accept(self):
        global telemetryClient
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Telemetry> incoming connection from ', repr(addr))
            self.handler = buggyTelemetryHandler(sock)
            telemetryClient = True

    def sendTelemetry(self, data):
        global telemetryClient
        if telemetryClient:
            self.handler.send(data.encode("utf-8"))

                
class buggyServer(asyncore.dispatcher):

    def __init__(self, host, port):

        # init (for client-server communication)
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
        
        # control 
        self.pid_camera_params = {'kp':1.5, 'kd':3.0, 'kd2':2.0, 'error':0.0, 'last_error':0.0, 'next_error':0.0 }
        self.pid_wall_following = my_controller.pid(kp=0.0033, ki=0.0003, kd=0.1, integral_max=1000, output_max=1.0, alpha=0.1) 
        self.wall_threshold_min = 40
        self.wall_threshold_max = 160
        self.wall_distance_middle = 86
        self.direction = 128
        self.throttle = 128
        self.mode = 0
        self.min_speed = 146
        self.max_speed = 163
        self.k_speed = 0.01
        self.positional_error_threshold = 350
        self.dual_rate = 0.5
        self.direction_trim = 0
        
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
        self.ia = False # control CNN and image processing
        self.running = False # control THROTTLE !!!
        self.recording = False # control recording video and data locally

        self.s = 0
        self.ia_version = 2
        self.model = 0
        self.cap_send = 0
        self.out_send = 0

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

        self.init_cam()
        
    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Incoming connection from ', repr(addr))
            buggyHandler(sock)
        
    def init_IA(self):

        from keras.models import load_model

        if not self.ia:
            print("Initializing IA")
            # load model
            if self.ia_version == 1:
                self.model = load_model('bitume_model.h5')
            elif self.ia_version == 2:
                self.model = load_model('synthetic_model.h5')
            self.model.summary()
            self.ia = True

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
        if self.ia_version == 1:
            # V1 (color / CNN)
            # clean-up and grayscale
            image_smoothed= cv2.blur(self.image,(3,3))

            # crop lower and middle frames
            im_croped_1 = image_smoothed[self.new_height-height_end_1:self.new_height-height_start_1,:self.new_width]
            assert(im_croped_1.shape == (crop_height,self.new_width,3))
            im_croped_2 = image_smoothed[self.new_height-height_end_2:self.new_height-height_start_2,:self.new_width]
            assert(im_croped_2.shape == (crop_height,self.new_width,3))
            
            # CNN processing
            x_1 = im_croped_1.reshape(crop_height,self.new_width,3)/255.0
            x_2 = im_croped_2.reshape(crop_height,self.new_width,3)/255.0
            x = np.array([x_1, x_2])
            y = self.model.predict(x)
            self.y_1, self.y_2 = y.item(0), y.item(1)
            ###print(str(self.y_1))
            ###print(str(self.y_2))
        elif self.ia_version == 2:
            # V2 (grayscale / CNN)

            # equialize
            image_yuv = cv2.cvtColor(self.image, cv2.COLOR_RGB2YUV)
            image_yuv[:, :, 0] = cv2.equalizeHist(image_yuv[:, :, 0])
            self.image = cv2.cvtColor(image_yuv, cv2.COLOR_YUV2RGB)
            
            # clean-up and grayscale
            image_smoothed= cv2.blur(self.image,(3,3))
            image_gray = cv2.cvtColor(image_smoothed, cv2.COLOR_RGB2GRAY)

            # crop lower and middle frames
            im_croped_1 = image_gray[self.new_height-height_end_1:self.new_height-height_start_1,:self.new_width]
            assert(im_croped_1.shape == (crop_height,self.new_width))
            im_croped_2 = image_gray[self.new_height-height_end_2:self.new_height-height_start_2,:self.new_width]
            assert(im_croped_2.shape == (crop_height,self.new_width))
            
            # CNN processing
            x_1 = im_croped_1.reshape(crop_height,self.new_width,1)/255.0
            x_2 = im_croped_2.reshape(crop_height,self.new_width,1)/255.0
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

    def process(self):
        
        # telemetry
        #####self.conn.send(fps.to_bytes(2, byteorder='big') + self.lidar_distance_gauche.to_bytes(2, byteorder='big', signed=True) + self.lidar_distance_droit.to_bytes(2, byteorder='big', signed=True) )


        # camera
        return_value, self.image = self.cap_send.read()
        assert(self.image.shape == (self.new_height,self.new_width,3))

        # reset control algorithm
        self.direction = 128
        self.throttle = 128
        self.mode = 0
        
        # based on AI
        if self.ia:
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

            # direction control
            self.direction = int(((pid_ai+1.0)/2.0*255.0))
        # based on LIDARs    
        else:
            # PID update according positional error given by left and right front LIDARs
            actual_error = constraint(self.lidar_distance_droit - self.lidar_distance_gauche, -150, 150)
            pid = self.pid_wall_following.compute(actual_error)
            positional_error = abs(self.pid_wall_following.integral_error)
            ###print("actual_error: " + str(actual_error) + " positional_error: " + str(positional_error) )

            # direction and throttle, control with threshold
            if positional_error > self.positional_error_threshold:
                
                # high positional error             
                ###print("high positional error")
                
                # direction normal rate
                self.direction = int(((pid+1.0)/2.0*255.0))

                # reduce throttle
                self.throttle = int( float(self.max_speed) - self.k_speed*(float)(positional_error- self.positional_error_threshold) )
                ###print("max_speed:" + str(self.max_speed) + "   delta:" + str(self.k_speed*(float)(positional_error- self.positional_error_threshold)) + "  THR:" + str(self.throttle))
                self.throttle = constraint(self.throttle,  self.min_speed,  self.max_speed)

            else:
            
                # low positional error             
                ###print("low positional error")

                # direction low rate
                self.direction = int(((pid*self.dual_rate+1.0)/2.0*255.0))

                # max throttle
                self.throttle = self.max_speed

        # throttle control
        if self.running:
            self.mode = 1
        else:
            self.throttle = 128
            self.mode = 0
        
        #trim & limits
        self.direction += self.direction_trim
        self.direction = constraint(self.direction,  20,  235) # hard-coded limits
        self.throttle = constraint(self.throttle,  128,  200) # hard-coded limits
        
        # STM32 control link
        self.port.write("{:d};".format(self.direction).encode('ascii') + "{:d};".format(int(self.throttle)).encode('ascii') + "{:d}\r\n".format(self.mode).encode('ascii'))
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

        # camera broadcast
        #self.out_send.write(self.image)
        self.out_send.write(self.image)
    
    def reload_settings(self):
        global param_kp
        global param_ki
        global param_kd
        global param_min_speed
        global param_max_speed
        global param_k_speed
        global param_positional_error_threshold
        global param_dual_rate
        global param_direction_trim        
        self.pid_wall_following.kp = param_kp
        self.pid_wall_following.ki = param_ki
        self.pid_wall_following.kd = param_kd
        self.min_speed = param_min_speed
        self.max_speed = param_max_speed
        self.k_speed = param_k_speed
        self.positional_error_threshold = param_positional_error_threshold
        self.dual_rate = param_dual_rate
        self.direction_trim = param_direction_trim

    def start_running(self):
        self.running = True
        
    def stop_running(self):
        self.running = False
        
    def quit(self):
        print("All done, quitting !")
        self.cap_send.release()
        self.out_send.release()

def main():
    global command_id
    global telemetryClient
    # FPS debug
    last_time = time.time()
    frame_counter = 0
    fps = 0
    fps_counter = 0
    # Settings
    command_id = 0 # idle
    load_configuration_file()
    # Server start
    serv = buggyServer("10.42.0.1", 6000)
    serv.reload_settings()
    serverTelemetry = buggyTelemetryServer("10.42.0.1", 7001)
    telemetryClient = False
    counter = 0
    # Main loop
    while True:
        # Non blocking call
        asyncore.loop(timeout=0, count=1)
        #  Process command ID
        if command_id == 1:
            serv.quit()
            break
        elif command_id == 2:
            serv.init_IA()
        elif command_id == 3:
            print("Start recording...")
            #serv.xxxx()
        elif command_id == 4:
            print("Stop recording!")
            #serv.xxxx()
        elif command_id == 5:
            print("Start running...")
            serv.start_running()
        elif command_id == 6:
            print("Stop running!")
            serv.stop_running()
        elif command_id == 10:
            serv.reload_settings()
        command_id = 0
        # Process
        serv.process()
        # Telemetry
        fps_counter += 1
        msg = str(fps_counter) + ';' + str(float(serv.direction)) + ';' + str(float(serv.throttle)) + ';' + str(float(serv.pid_wall_following.error))
        serverTelemetry.sendTelemetry(msg)
        # FPS debug
        frame_counter += 1
        if time.time()>=last_time+1.0:
            last_time=time.time()
            fps = frame_counter
            frame_counter = 0
        # Local Debug
        if counter % 60 == 0:
            print("fps:" + str(fps) + "   DIR:" + str(serv.direction) + "   THR:" + str(serv.throttle) + "   MODE:" + str(serv.mode) + "   LiG:" + str(serv.lidar_distance_gauche)+"   LiD:" + str(serv.lidar_distance_droit))
        counter += 1

# Le C ca rassure !!
if __name__ == "__main__":
    main()
    
