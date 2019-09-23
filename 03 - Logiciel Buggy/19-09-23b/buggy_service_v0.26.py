import re
import time
import cv2
import serial
import socket
import asyncore
#import numpy as np

# my modules
import my_controller
from my_math import  *
import my_datalogger

# global (for client-server communication)
global telemetry_client_connected
global remote_configuration_command_id # received command from client 
    # CID =  0 : default/idle
    # CID =  1 : kill buggy server
    # CID =  2 : start Line Following with CNN
    # CID =  3 : start recording video and data
    # CID =  4 : stop recording video and data 
    # CID =  5 : start run
    # CID =  6 : stop run
    # CID =  7 : start recording dataset
    # CID =  8 : stop recording dataset
    # CID = 10 : reload settings (globals)

# global (for settings)
global param_min_speed
global param_max_speed
global param_acceleration
global param_deceleration
global param_speed_kp
global param_speed_ki
global param_speed_kd
global param_speed_kff

global param_lidar_direction_kp # from position error (cm) to steering (-1..+1)
global param_lidar_direction_ki
global param_lidar_direction_kd
global param_lidar_k_speed # proportional coef . Input is sum of position error (cm). Output is delta speed (m/s) summed to max speed (m/s)
global param_lidar_positional_error_threshold # sum of position error (cm over 30 frames) threshold to activate proportional speed  limiting

global param_ai_direction_alpha
global param_ai_direction_kp # from direction error (-1..+1) to steering (-1..+1)
global param_ai_direction_ki
global param_ai_direction_kd
global param_ai_direction_k_speed # proportional coef . Input is direction error (-1..+1). Output is delta speed (m/s) summed to max speed (m/s)
global param_ai_steering_k_speed # proportional coef . Input is steering (-1..+1). Output is delta speed (m/s) summed to max speed (m/s)

global param_dual_rate
global param_steering_trim

def load_configuration_file():
    global param_min_speed      
    global param_max_speed       
    global param_acceleration
    global param_deceleration
    global param_speed_kp
    global param_speed_ki
    global param_speed_kd        
    global param_speed_kff        

    global param_lidar_direction_kp
    global param_lidar_direction_ki
    global param_lidar_direction_kd        
    global param_lidar_k_speed      
    global param_lidar_positional_error_threshold 

    global param_ai_direction_alpha
    global param_ai_direction_kp
    global param_ai_direction_ki
    global param_ai_direction_kd
    global param_ai_direction_k_speed
    global param_ai_steering_k_speed
 
    global param_steering_trim
    global param_dual_rate

    file = open('config.txt', 'r')
    for line in file:
        ###print(line)
        fields = re.split(' +', line)
        ###print(str(fields))

        if fields[0] == "SPEED":
            param_min_speed =  float(fields[1])
            param_max_speed =  float(fields[2])
            param_acceleration =  float(fields[3])
            param_deceleration =  float(fields[4])
            param_speed_kp = float(fields[5])
            param_speed_ki = float(fields[6])
            param_speed_kd = float(fields[7])
            param_speed_kff = float(fields[8])

        if fields[0] == "WALL_FOLLOWING":
            param_lidar_direction_kp = float(fields[1])
            param_lidar_direction_ki = float(fields[2])
            param_lidar_direction_kd = float(fields[3])
            param_lidar_k_speed =  float(fields[4])
            param_lidar_positional_error_threshold =  int(fields[5])

        if fields[0] == "LINE_FOLLOWING":
            param_ai_direction_alpha = float(fields[1])
            param_ai_direction_kp = float(fields[2])
            param_ai_direction_ki = float(fields[3])
            param_ai_direction_kd = float(fields[4])
            param_ai_direction_k_speed = float(fields[5])
            param_ai_steering_k_speed = float(fields[6])
            
        if fields[0] == "STEERING":
            param_steering_trim =  int(fields[1])
            param_dual_rate =  float(fields[2])
    file.close()

'''''''''''''''''''''''''''''
    Telemetry handler
'''''''''''''''''''''''''''
class telemetry_handler(asyncore.dispatcher_with_send):
  
    def handle_read(self):
        pass

    def handle_close(self):
        global telemetry_client_connected
        print('Telemetry> connection closed.')
        telemetry_client_connected = False
        self.close()

'''''''''''''''''''''''
    Telemetry server
'''''''''''''''''''''
class telemetry_server(asyncore.dispatcher):
    
    def __init__(self, host, port):
        global telemetry_client_connected
        telemetry_client_connected = False
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
    
    def handle_accept(self):
        global telemetry_client_connected
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Telemetry> incoming connection from ', repr(addr))
            self.handler = telemetry_handler(sock)
            telemetry_client_connected = True

    def sendTelemetry(self, data):
        global telemetry_client_connected
        if telemetry_client_connected:
            self.handler.send(data.encode("utf-8"))
            #self.flush()

'''''''''''''''''''''''''''''
    Remote configuration handler
'''''''''''''''''''''''''''

# constants (for client-server communication)
CMD_GET_API  = 'GET_API'
CMD_GET         = 'GET'
CMD_SET         = 'SET'
CMD_SPLIT      = ';'

class remote_configuration_handler(asyncore.dispatcher_with_send):
    
    def handle_close(self):
        print('Remote configuration> connection closed.')
        self.close()
        
    def handle_read(self):
        global param_min_speed      
        global param_max_speed       
        global param_acceleration
        global param_deceleration
        global param_speed_kp
        global param_speed_ki
        global param_speed_kd        
        global param_speed_kff        

        global param_lidar_direction_kp
        global param_lidar_direction_ki
        global param_lidar_direction_kd        
        global param_lidar_k_speed      
        global param_lidar_positional_error_threshold 

        global param_ai_direction_alpha
        global param_ai_direction_kp
        global param_ai_direction_ki
        global param_ai_direction_kd
        global param_ai_direction_k_speed
        global param_ai_steering_k_speed
     
        global param_steering_trim
        global param_dual_rate

        global remote_configuration_command_id

        data = self.recv(8192)
        if data:
            print('Receive command: ' + data.decode("utf-8"))
            res = data.decode("utf-8").rstrip(CMD_SPLIT).split(CMD_SPLIT)
            print(res)

            if (res[0] == CMD_GET_API):
                resp = "GET_API;min_speed;max_speed;acceleration;deceleration;speed_kp;speed_ki;speed_kd;speed_kff;lidar_direction_kp;lidar_direction_ki;lidar_direction_kd;lidar_k_speed;lidar_positional_error_threshold;ai_direction_alpha;ai_direction_kp;ai_direction_ki;ai_direction_kd;ai_direction_k_speed;ai_steering_k_speed;steering_trim;dual_rate"
                print(resp)
                self.send(resp.encode("utf-8"))
                print('Response sent.')
                return

            if (res[0] == CMD_GET):
                resp = "GET;" + \
                    str(param_min_speed) + ";"  +  \
                    str(param_max_speed) + ";"  +  \
                    str(param_acceleration) + ";" +  \
                    str(param_deceleration) + ";" +  \
                    str(param_speed_kp) + ";" +  \
                    str(param_speed_ki) + ";"  +  \
                    str(param_speed_kd) + ";"  +  \
                    str(param_speed_kff)  + ";" +  \
                    str(param_lidar_direction_kp) + ";" +  \
                    str(param_lidar_direction_ki) + ";"  +  \
                    str(param_lidar_direction_kd) + ";"  +  \
                    str(param_lidar_k_speed) + ";"  +  \
                    str(param_lidar_positional_error_threshold) + ";"  +  \
                    str(param_ai_direction_alpha) + ";" +  \
                    str(param_ai_direction_kp) + ";" +  \
                    str(param_ai_direction_ki) + ";"  +  \
                    str(param_ai_direction_kd) + ";"  +  \
                    str(param_ai_direction_k_speed) + ";"  +  \
                    str(param_ai_steering_k_speed) + ";"  +  \
                    str(param_steering_trim) + ";"  +  \
                    str(param_dual_rate)
                    
                print(resp)
                self.send(resp.encode("utf-8"))
                print('Response sent.')
                return

            if (res[0] == CMD_SET):
                param_min_speed = float(res[1])
                param_max_speed = float(res[2])
                param_acceleration = float(res[3])
                param_deceleration = float(res[4])
                param_speed_kp = float(res[5])
                param_speed_ki = float(res[6])
                param_speed_kd = float(res[7])
                param_speed_kff = float(res[8])
                param_lidar_direction_kp = float(res[9])
                param_lidar_direction_ki = float(res[10])
                param_lidar_direction_kd = float(res[11])
                param_lidar_k_speed = float(res[12])
                param_lidar_positional_error_threshold = int(res[13])
                param_ai_direction_alpha = float(res[14])
                param_ai_direction_kp = float(res[15])
                param_ai_direction_ki = float(res[16])
                param_ai_direction_kd = float(res[17])
                param_ai_direction_k_speed = float(res[18])
                param_ai_steering_k_speed = float(res[19])
                param_steering_trim = int(res[20])
                param_dual_rate = float(res[21])
                remote_configuration_command_id = 10
                return        

            if (res[0] == "KILL"):
                remote_configuration_command_id = 1
                return

            if (res[0] == "IA"):
                remote_configuration_command_id = 2
                return

            if (res[0] == "REC_START"):
                remote_configuration_command_id = 3
                return

            if (res[0] == "REC_STOP"):
                remote_configuration_command_id = 4
                return

            if (res[0] == "RUN_START"):
                remote_configuration_command_id = 5
                return

            if (res[0] == "RUN_STOP"):
                remote_configuration_command_id = 6
                return

            if (res[0] == "DATASET_START"):
                remote_configuration_command_id = 7
                return

            if (res[0] == "DATASET_STOP"):
                remote_configuration_command_id = 8
                return

                
'''''''''''''''''''''''
    Remote configuration server
'''''''''''''''''''''
class remote_configuration_server(asyncore.dispatcher):

    def __init__(self, host, port):
        global remote_configuration_command_id
        remote_configuration_command_id = 0 # idle
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
        
    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Remote configuration> Incoming connection from ', repr(addr))
            remote_configuration_handler(sock)

'''''''''''''''''''''''
    Camera helpers
'''''''''''''''''''''
        
def gstreamer_pipeline (capture_width=1280, capture_height=720, display_width=160, display_height=90, framerate=60, flip_method=2) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))
        
'''''''''''''''''''''''
    Controller
'''''''''''''''''''''
class main_controller():

    def __init__(self):
        
        # speed controller settings
        self.min_speed = 0.25 # m/s
        self.max_speed = 1.2 # m/s
        self.acceleration = 0.01 # m/s per 1/60eme
        self.deceleration = 0.1 # m/s per 1/60eme
        self.pid_speed = my_controller.pid(kp=10.0, ki=0.0, kd=100.0, integral_max=1000, output_max=50.0, alpha=0.2) 
        self.pid_speed_kff = 15.0 # feed forward apart from speed PID
        
        # speed controller state
        self.target_speed_ms = 0.0 # m/s (square)
        self.current_speed_ms = 0.0 # m/s (trapeze)
        self.actual_speed_ms = 0.0 # m/s from encoder (real)
        self.actual_speed_kmh = 0.0 # km.h from encoder
        self.actual_speed_error_ms = 0.0 # m/s

        # lidar steering controller settings
        self.pid_wall_following = my_controller.pid(kp=0.0033, ki=0.0003, kd=0.1, integral_max=1000, output_max=1.0, alpha=0.2) 
        self.lidar_k_speed = 0.01
        self.lidar_positional_error_threshold = 350

        # lidar steering controller state
        self.lidar_distance_gauche = 0
        self.lidar_distance_droit = 0
        self.lidar_distance_haut = 0        
        self.actual_lidar_direction_error = 0.0
        self.lidar_positional_error = 0.0    
        self.pid_wall = 0.0

        # AI steering controller settings
        self.pid_line_following = my_controller.pid(kp=0.8, ki=0.0, kd=0.0, integral_max=1000, output_max=1.0, alpha=0.2) 
        self.ai_direction_k_speed = 1.0
        self.ai_steering_k_speed = 0.2
        
        # AI steering controller state
        self.line_pos_unfiltered = 0.0
        self.line_pos = 0.0
        self.pid_line = 0.0

        # steering settings
        self.steering_trim = 4
        self.dual_rate = 0.5
        
        # controller state to STM32 
        self.steering = 128
        self.throttle = 128
        self.mode = 0
        
        # controller inputs data from STM32 controller board
        self.telemetry_speed = 0
        self.telemetry_manual_dir = 0
        self.telemetry_manual_thr = 0
        self.telemetry_auto_dir = 0
        self.telemetry_auto_thr = 0
        self.telemetry_start_button = 0  
        # serial port for STM32 data link 
        self.port = serial.Serial("/dev/ttyUSB0",baudrate=115200,timeout=3.0)

        # global state
        self.ia = False # control CNN and image processing
        self.running = False # control THROTTLE !!!
        self.recording = False # control recording video and data locally
        self.dataset = 0 # control recording of a dataset (picture,DIR,THR)

        # camera handler
        self.video_capture = 0
        self.video_broadcast = 0
        self.video_recorder = 0
        self.video_broadcast_IP_address ='10.42.0.112' # IPv4 address of PC connectd through WiFi and receiving video stream

        # camera settings
        self.rec_width = 1280
        self.rec_height = 720   
        self.new_width = 160
        self.new_height = 90

        # camera state
        self.image = 0
        self.gray = 0
        self.init_video()
        
        # AI handler
        self.model = 0
        
        # Data logger
        self.datalogger = my_datalogger.datalogger("datalogger")
        
        # Dataset
        self.dataset_directory = "dataset"
        self.dataset_counter = 0
        self.dataset_file = None
        
    def init_ai(self):

        from keras.models import load_model

        if not self.ia:
            print("Initializing AI...")
            # load model
            self.model = load_model('model.h5')
            self.model.summary()
            self.ia = True
            print("AI ready!")

    def init_video(self):
        print("Initializing Video...")

	# create gstreamer capture pipeline
        self.video_capture = cv2.VideoCapture(
            gstreamer_pipeline(
                capture_width = self.rec_width, 
                capture_height = self.rec_height, 
                display_width = self.new_width, 
                display_height = self.new_height, 
                framerate = 60, 
                flip_method = 2 ), 
            cv2.CAP_GSTREAMER
        )
        return_value, self.image = self.video_capture.read()
        assert(self.image.shape == (self.new_height,self.new_width,3))
        print("Video capture OK!")

        # init gstreamer broadcast pipeline
        self.video_broadcast = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=200 ! queue ! rtph264pay config-inerval=1 ! udpsink host='+self.video_broadcast_IP_address+' port=5500 sync=false',cv2.CAP_GSTREAMER,0, 20, (self.new_width,self.new_height), True)
        print("Video broadcast OK!")

        # init video recorder
        print("Video recorder OK!")

        print("Video ready!")

    def ai_processing(self):
        # process image before IA
        # V1 (color / CNN)
        # clean-up and grayscale
        image_smoothed= cv2.blur(self.image,(3,3))
        self.gray = cv2.cvtColor(image_smoothed,  cv2.COLOR_RGB2GRAY)
        self.line_pos_unfiltered = self.model.predict(self.gray.reshape(1, self.new_height, self.new_width, 1)).item(0)
        #print(str(self.line_pos))

    def process(self):
        
        # capture camera (blocking call : process period <= framerate 60Hz)
        return_value, self.image = self.video_capture.read()

        # reset control state
        self.steering = 128
        self.throttle = 128
        self.mode = 0

        # speed control (stage 1)
        if self.running:
            # fix target speed
            self.target_speed_ms = self.max_speed # default is maximum speed when running
        else:
            self.target_speed_ms = 0.0 # enforce 0 speed when halted

        # steering control...
  
        if self.ia: # steering computed from AI (line position estimation)
            # process current picture and predict line position
            self.ai_processing()
            
            # line following PID controller
            self.line_pos = self.line_pos * (1.0-self.ai_direction_alpha) + self.ai_direction_alpha * self.line_pos_unfiltered            
            self.pid_line = self.pid_line_following.compute(self.line_pos)
            
            ###print(str(self.pid_line) + ';' + str(self.pid_line_following.kp) + ';' + str(self.pid_line_following.ki) + ';' + str(self.pid_line_following.kd))

            # steering using normal rate
            self.steering = int(((self.pid_line+1.0)/2.0*255.0))
            
            # reduce current speed according line position error and steering force
            self.target_speed_ms -= ( abs(self.line_pos)*self.ai_direction_k_speed + abs(self.pid_line)*self.ai_steering_k_speed )
            self.target_speed_ms = max(self.target_speed_ms,self.min_speed) # limit to minimum speed
            
        else: # steering computed from LIDAR (wall following)
        
            # compute lidar positionnal error
            lid = constraint(self.lidar_distance_droit, 0, 150)
            lig = constraint(self.lidar_distance_gauche, 0, 150)
            self.actual_lidar_direction_error = constraint(lid - lig, -150, 150)
            
            # wall following PID controller
            self.pid_wall = self.pid_wall_following.compute(self.actual_lidar_direction_error)
            self.lidar_positional_error = abs(self.pid_wall_following.integral_error)
            ###print("actual_error: " + str(actual_error) + " positional_error: " + str(positional_error) )

            # direction and throttle, control with threshold
            if self.lidar_positional_error > self.lidar_positional_error_threshold:
                
                # high positional error             
                ###print("high positional error")
                
                # direction normal rate
                self.steering = int(((self.pid_wall+1.0)/2.0*255.0))

                # reduce current speed according lidar positional error
                self.target_speed_ms -= (self.lidar_positional_error - self.lidar_positional_error_threshold)*self.lidar_k_speed 
                self.target_speed_ms = max(self.target_speed_ms,self.min_speed) # limit to minimum speed
                
            else:
            
                # low positional error             
                ###print("low positional error")

                # direction low rate
                self.steering = int(((self.pid_wall*self.dual_rate+1.0)/2.0*255.0))

        # clamp max speed
        self.target_speed_ms = constraint(self.target_speed_ms,  self.min_speed,  self.max_speed)

        # compute current speed from target and time passing (trapeze)
        if self.current_speed_ms < self.target_speed_ms:
            self.current_speed_ms += self.acceleration
            self.current_speed_ms = min(self.current_speed_ms,self.max_speed) # limit to maximum speed
        if self.current_speed_ms > self.target_speed_ms:
            self.current_speed_ms -= self.deceleration
            self.current_speed_ms = max(self.current_speed_ms,self.min_speed) # limit to minimum speed
        self.current_speed_ms = constraint(self.current_speed_ms,  self.min_speed,  self.max_speed)
        
         # compute throttle according actual_speed
        self.actual_speed_error_ms = self.current_speed_ms - self.actual_speed_ms
        self.throttle = int( 128.0 + self.pid_speed.compute(self.actual_speed_error_ms) + self.pid_speed_kff *self.current_speed_ms )
            
        # manual / automatic control
        if self.running:
            # AI and LIDAR control throttle and direction
            self.mode = 1
        else:
            # AI and LIDAR control direction, but throttle is forced to neutral
            self.throttle = 128
            self.mode = 0
        
        #trim & limits
        self.steering += self.steering_trim
        self.steering = constraint(self.steering,  20,  235) # hard-coded limits
        self.throttle = constraint(self.throttle,  20,  180) # hard-coded limits
        
        # STM32 control link
        self.port.write("{:d};".format(self.steering).encode('ascii') + "{:d};".format(int(self.throttle)).encode('ascii') + "{:d}\r\n".format(self.mode).encode('ascii'))
        serial_line = self.port.readline()
        ####print(str(serial_line))
        fields = serial_line.decode('ascii').split(';')
        if(len(fields)>=9):
           self.lidar_distance_gauche = int(fields[0])
           self.lidar_distance_droit = int(fields[1])
           self.lidar_distance_haut = int(fields[2])
           self.telemetry_speed = int(fields[3])
           self.telemetry_manual_dir = int(fields[4])
           self.telemetry_manual_thr = int(fields[5])
           self.telemetry_auto_dir = int(fields[6])
           self.telemetry_auto_thr = int(fields[7])
           self.telemetry_start_button = int(fields[8])   
        # start condition ==> release  start button ('0' > '1' from STM32)
        if self.telemetry_start_button == 1 and not self.running:
            print("Start condition detected!")
            self.running = True
            self.datalogger.start()
        # stop condition ==> 
        if self.lidar_distance_haut > 0 and self.lidar_distance_haut < 110 and self.running:
            print("Stop condition detected!")
            self.running = False
            self.datalogger.stop()

        # camera recording
        if self.recording:
            self.video_recorder.write(self.image)
            
        # Speed computation
        self.actual_speed_ms = round( 100000.0/(4.0*float(self.telemetry_speed+1)) / 2.75 * 0.2,  2)
        self.actual_speed_kmh = self.actual_speed_ms * 3.6
                
        # Data logger
        log_list = []
        log_list.append( float(-1) )
        
        log_list.append( float(self.target_speed_ms) )
        log_list.append( float(self.current_speed_ms) )
        log_list.append( float(self.actual_speed_ms) ) 
        log_list.append( float(self.actual_speed_error_ms) ) 
        log_list.append( float(self.steering) )

        log_list.append( float(self.lidar_distance_gauche) )
        log_list.append( float(self.lidar_distance_droit) )
        log_list.append( float(self.lidar_distance_haut) )
        log_list.append( float(self.actual_lidar_direction_error) )
        log_list.append( float(self.lidar_positional_error) )
        log_list.append( float(self.pid_wall) )

        log_list.append( float(self.line_pos) )
        log_list.append( float(self.pid_line) )
        log_list.append( float(self.throttle) )

        log_list.append( float(self.mode) )

        self.datalogger.record(log_list)
        
        # Dataset recording
        if self.dataset:
            # compute picture filename
            picture_filename = self.dataset_directory + "/" + "pic_" + str(self.dataset_counter) + ".jpg"
            cv2.imwrite(picture_filename, self.image)
            self.dataset_file.write(picture_filename + ";" + str(self.telemetry_manual_dir) + ";" + str(self.telemetry_manual_thr) + "\r\n")
            self.dataset_file.flush()
            self.dataset_counter += 1
    
        # camera broadcast
        overlay = self.image.copy()
        # OSD
        cv2.putText(overlay, "TH:{:d}".format(self.throttle) + "  DR:{:d}".format(self.steering),  (4, 12), cv2.FONT_HERSHEY_PLAIN,1.0, (100, 100, 255), 1)
        cv2.circle(overlay, (int(self.image.shape[1]*(self.pid_wall+1.0)/2.0),int(self.image.shape[0]/2.0)+20), 10, (100, 100, 255) )
        cv2.line(overlay, (int(self.new_width/2),0 ),  (int(self.new_width/2),self.new_height), color=(0,0,255) )
        cv2.line(overlay, (0,int(self.new_height/2)),  (self.new_width,int(self.new_height/2)), color=(0,0,255) )
        pt_x = int((self.line_pos+1.0)/2.0*self.new_width)
        cv2.line(overlay, (pt_x,0), (pt_x,self.new_height), color=(255, 0,0) )
        self.image = cv2.addWeighted(overlay, 0.5, self.image, 0.5, 0)
        self.video_broadcast.write(self.image)
        
    def reload_settings(self):
        global param_min_speed      
        global param_max_speed       
        global param_acceleration
        global param_deceleration
        global param_speed_kp
        global param_speed_ki
        global param_speed_kd        
        global param_speed_kff        

        global param_lidar_direction_kp
        global param_lidar_direction_ki
        global param_lidar_direction_kd        
        global param_lidar_k_speed      
        global param_lidar_positional_error_threshold 

        global param_ai_direction_alpha
        global param_ai_direction_kp
        global param_ai_direction_ki
        global param_ai_direction_kd
        global param_ai_direction_k_speed
        global param_ai_steering_k_speed
     
        global param_steering_trim
        global param_dual_rate

        self.min_speed = param_min_speed
        self.max_speed = param_max_speed
        self.acceleration = param_acceleration
        self.deceleration = param_deceleration
        self.pid_speed.kp = param_speed_kp
        self.pid_speed.ki = param_speed_ki
        self.pid_speed.kd = param_speed_kd
        self.pid_speed_kff = param_speed_kff

        self.pid_wall_following.kp = param_lidar_direction_kp
        self.pid_wall_following.ki = param_lidar_direction_ki
        self.pid_wall_following.kd = param_lidar_direction_kd
        self.lidar_k_speed = param_lidar_k_speed
        self.lidar_positional_error_threshold = param_lidar_positional_error_threshold

        self.ai_direction_alpha = param_ai_direction_alpha
        self.pid_line_following.kp = param_ai_direction_kp
        self.pid_line_following.ki = param_ai_direction_ki
        self.pid_line_following.kd = param_ai_direction_kd
        self.ai_direction_k_speed = param_ai_direction_k_speed
        self.ai_steering_k_speed = param_ai_steering_k_speed
        
        self.steering_trim = param_steering_trim
        self.dual_rate = param_dual_rate

    def start_running(self):
        if not self.running:
            self.running = True
            self.datalogger.start()
        
    def stop_running(self):
        if self.running:
            self.running = False
            self.datalogger.stop()

    def start_recording(self):
        if not self.recording :
            filename =  "capture/capture_" + time.asctime().replace(' ', '_').replace(':', '-')  + ".avi"
            self.video_recorder = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc('M','J','P','G'), 10, (self.new_width,self.new_height))
            self.recording = True
        
    def stop_recording(self):
        if self.recording :
            self.video_recorder.release()
            self.recording = False

    def start_dataset(self):
        if self.dataset:
            self.dataset_file.close()
        filename =  self.dataset_directory + "/" + "dataset.txt" ## + time.asctime().replace(' ', '_').replace(':', '-') + "/" 
        print(filename)
        self.dataset_file = open(filename, 'w+')
        self.dataset_counter = 0
        self.dataset = True
        
    def stop_dataset(self):
        if self.dataset:
            self.dataset_file.close()
        self.dataset = False
        
    def quit(self):
        print("All done, quitting !")
        self.video_capture.release()
        self.video_broadcast.release()

def main():
    global remote_configuration_command_id
    # FPS debug
    last_time = time.time()
    frame_counter = 0
    fps = 0
    fps_counter = 0
    # Load default settings
    load_configuration_file()
    # Controller start
    controller = main_controller()
    controller.reload_settings()
    # Servers start
    remote_configuration_server("10.42.0.1", 6000)
    tserver = telemetry_server("10.42.0.1", 7001)
    # Game loop
    counter = 0
    while True:
        # Non blocking call
        asyncore.loop(timeout=0, count=1)
        #  Parse and execute remote commands
        if remote_configuration_command_id == 1:
            controller.quit()
            break
        elif remote_configuration_command_id == 2:
            controller.init_ai()
        elif remote_configuration_command_id == 3:
            print("Start recording...")
            controller.start_recording()
        elif remote_configuration_command_id == 4:
            print("Stop recording!")
            controller.stop_recording()
        elif remote_configuration_command_id == 5:
            print("Start running...")
            controller.start_running()
        elif remote_configuration_command_id == 6:
            print("Stop running!")
            controller.stop_running()
        elif remote_configuration_command_id == 7:
            print("Start recording DATASET...")
            controller.start_dataset()
        elif remote_configuration_command_id == 8:
            print("Stop recoring DATASET!")
            controller.stop_dataset()
        elif remote_configuration_command_id == 10:
            controller.reload_settings()
        remote_configuration_command_id = 0 # reset
        # Process
        controller.process()
        # Telemetry
        counter += 1
        msg = str(counter) + ';'
        msg += str( float(controller.lidar_distance_gauche) ) + ';' #cm
        msg += str( float(controller.lidar_distance_droit) ) + ';'  #cm
        msg += str( float(controller.lidar_distance_haut) ) + ';'  #cm
        
        msg += str( float(controller.actual_lidar_direction_error) ) + ';'
        msg += str( float(controller.lidar_positional_error) ) + ';'  
        msg += str( float(controller.pid_wall) ) + ';'

        msg += str( float(controller.target_speed_ms) ) + ';'
        msg += str( float(controller.current_speed_ms) ) + ';'
        msg += str( float(controller.actual_speed_ms) ) + ';' 
        msg += str( float(controller.actual_speed_error_ms) ) + ';'
        msg += str( float(controller.throttle) )  + ';'
        
        msg += str( float(controller.line_pos) ) + ';'
        msg += str( float(controller.pid_line) ) + ';' 
        msg += str( float(controller.steering) )
        
        #msg += str( float(controller.mode) ) 

        msg_length = str(len(msg)).ljust(4)
        tserver.sendTelemetry(msg_length)
        tserver.sendTelemetry(msg)
        # FPS debug
        frame_counter += 1
        if time.time()>=last_time+1.0:
            last_time=time.time()
            fps = frame_counter
            frame_counter = 0
        # Local trace
        if fps_counter % 6 == 0:
            print("fps:" + str(fps) + 
                "   DIR:" + str(controller.steering) + 
                "   THR:" + str(controller.throttle) + 
                "   MODE:" + str(controller.mode) + 
                
                "   LiG:" + str(controller.lidar_distance_gauche) +
                "   LiD:" + str(controller.lidar_distance_droit) +
                "   LiH:" + str(controller.lidar_distance_haut) + 
                
                "   mDIR:" + str(controller.telemetry_manual_dir) + 
                "   mTHR:" + str(controller.telemetry_manual_thr) + 

                "  Speed:" + str( controller.actual_speed_ms   )  + "m/s"
                "  Speed:" + str( controller.actual_speed_kmh  )  + "km/h"
            )
        fps_counter += 1
        
# Le C ca rassure !!
if __name__ == "__main__":
    main()
    
