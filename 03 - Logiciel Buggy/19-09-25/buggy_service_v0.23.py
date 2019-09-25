import time
import cv2
import serial
import socket
import asyncore
import numpy as np

# my modules
import my_controller
from my_math import  *
import my_datalogger

# global (for client-server communication)
global telemetry_client_connected
global param_lidar_direction_kp
global param_lidar_direction_ki
global param_lidar_direction_kd
global param_ai_direction_kp
global param_ai_direction_ki
global param_ai_direction_kd
global param_speed_kp
global param_speed_ki
global param_speed_kd
global param_speed_kff
global param_max_speed
global param_min_speed
global param_k_speed
global param_positional_error_threshold
global param_dual_rate
global param_direction_trim
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

def load_configuration_file():
    global param_lidar_direction_kp
    global param_lidar_direction_ki
    global param_lidar_direction_kd        
    global param_ai_direction_kp
    global param_ai_direction_ki
    global param_ai_direction_kd
    global param_speed_kp
    global param_speed_ki
    global param_speed_kd        
    global param_speed_kff        
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
            param_lidar_direction_kp = float(fields[1])
            param_lidar_direction_ki = float(fields[2])
            param_lidar_direction_kd = float(fields[3])
        if fields[0] == "LINE_FOLLOWING":
            param_ai_direction_kp = float(fields[1])
            param_ai_direction_ki = float(fields[2])
            param_ai_direction_kd = float(fields[3])
        if fields[0] == "SPEED":
            param_min_speed =  float(fields[1])
            param_max_speed =  float(fields[2])
            param_k_speed =  float(fields[3])
            param_positional_error_threshold =  int(fields[4]) # error integral (cm)
        if fields[0] == "DIRECTION":
            param_direction_trim =  int(fields[1])
            param_dual_rate =  float(fields[2])
        if fields[0] == "SPEED_PID":
            param_speed_kp = float(fields[1])
            param_speed_ki = float(fields[2])
            param_speed_kd = float(fields[3])
            param_speed_kff = float(fields[4])
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
        global param_lidar_direction_kp
        global param_lidar_direction_ki
        global param_lidar_direction_kd
        global param_ai_direction_kp
        global param_ai_direction_ki
        global param_ai_direction_kd
        global param_speed_kp
        global param_speed_ki
        global param_speed_kd
        global param_speed_kff
        global param_min_speed
        global param_max_speed
        global param_k_speed
        global param_positional_error_threshold
        global param_dual_rate
        global param_direction_trim        
        global remote_configuration_command_id

        data = self.recv(8192)
        if data:
            print('Receive command: ' + data.decode("utf-8"))
            res = data.decode("utf-8").rstrip(CMD_SPLIT).split(CMD_SPLIT)
            print(res)

            if (res[0] == CMD_GET_API):
                resp = "GET_API;lidar_direction_kp;lidar_direction_ki;lidar_direction_kd;ai_direction_kp;ai_direction_ki;ai_direction_kd;speed_kp;speed_ki;speed_kd;speed_kff;min_speed;max_speed;k_speed;positional_error_threshold;dual_rate;direction_trim"
                print(resp)
                self.send(resp.encode("utf-8"))
                print('Response sent.')
                return

            if (res[0] == CMD_GET):
                resp = "GET;" + str(param_lidar_direction_kp) + ";" + str(param_lidar_direction_ki) + ";"  + str(param_lidar_direction_kd) + ";"  + str(param_ai_direction_kp) + ";" + str(param_ai_direction_ki) + ";"  + str(param_ai_direction_kd) + ";"  + str(param_speed_kp) + ";" + str(param_speed_ki) + ";"  + str(param_speed_kd) + ";"  + str(param_speed_kff)  + ";" + str(param_min_speed) + ";"  + str(param_max_speed) + ";"  + str(param_k_speed) + ";"  + str(param_positional_error_threshold) + ";"  + str(param_dual_rate) + ";"  + str(param_direction_trim)
                print(resp)
                self.send(resp.encode("utf-8"))
                print('Response sent.')
                return

            if (res[0] == CMD_SET):
                param_lidar_direction_kp = float(res[1])
                param_lidar_direction_ki = float(res[2])
                param_lidar_direction_kd = float(res[3])
                param_ai_direction_kp = float(res[4])
                param_ai_direction_ki = float(res[5])
                param_ai_direction_kd = float(res[6])
                param_speed_kp = float(res[7])
                param_speed_ki = float(res[8])
                param_speed_kd = float(res[9])
                param_speed_kff = float(res[10])
                param_min_speed = float(res[11])
                param_max_speed = float(res[12])
                param_k_speed = float(res[13])
                param_positional_error_threshold = int(res[14])
                param_dual_rate = float(res[15])
                param_direction_trim = int(res[16])
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

        # control algorithms
        self.pid_line_following = my_controller.pid(kp=0.0, ki=0.0, kd=0.0, integral_max=1000, output_max=1.0, alpha=0.5) 
        self.pid_wall_following = my_controller.pid(kp=0.0033, ki=0.0003, kd=0.1, integral_max=1000, output_max=1.0, alpha=0.1) 
        self.pid_speed = my_controller.pid(kp=70.0, ki=0.0, kd=100.0, integral_max=1000, output_max=50.0, alpha=0.5) 
        
        # controller settings
        self.pid_speed_kff = 0.0 # feed forward apart from PID
        self.min_speed = 0.5 # m/s
        self.max_speed = 2.0 # m/s
        self.k_speed = 0.01
        self.positional_error_threshold = 350
        self.dual_rate = 0.5
        self.direction_trim = 0
        
        # controller state
        self.direction = 128
        self.throttle = 128
        self.mode = 0
        
        self.actual_error = 0.0
        self.pid_wall = 0.0
        self.pid_line = 0.0
        self.positional_error = 0.0    
        
        self.actual_speed_error_ms = 0.0 # m/s
        self.target_speed_ms = 0.0 # m/s
        
        # controller inputs data from STM32 controller board
        self.lidar_distance_gauche = 0
        self.lidar_distance_droit = 0
        self.lidar_distance_haut = 0
        self.telemetry_speed = 0
        self.telemetry_manual_dir = 0
        self.telemetry_manual_thr = 0
        self.telemetry_auto_dir = 0
        self.telemetry_auto_thr = 0
        self.telemetry_start_button = 0  
        self.actual_speed_ms = 0.0
        self.actual_speed_kmh = 0.0
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
        
        # AI state
        self.line_pos = 0
        
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
            self.model = load_model('paul_model.h5')
            self.model.summary()
            self.ia = True
            print("AI ready!")

    def init_video(self):
        print("Initializing Video...")

	# create gstreamer capture pipeline
        self.video_capture = cv2.VideoCapture(gstreamer_pipeline(capture_width=self.rec_width, capture_height=self.rec_height, display_width=self.new_width, display_height=self.new_height, framerate=60, flip_method=2), cv2.CAP_GSTREAMER)
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
        self.line_pos = self.model.predict(self.gray.reshape(1, self.new_height, self.new_width, 1)).item(0)
        #print(str(self.line_pos))
        # draw line on image
        pt1_x = int((self.line_pos+1.0)/2.0*self.new_width)
        pt1_y = int(0)
        pt2_x = int((self.line_pos+1.0)/2.0*self.new_width)
        pt2_y = int(self.new_height)
        ###print ('coordonées : pt1({}, {}), pt2({},{})'.format(pt1_x,pt1_y,pt2_x,pt2_y))
        cv2.line(self.image,(pt1_x,pt1_y),(pt2_x,pt2_y),(255,0,0),5)

    def process(self):
        
        # capture camera (blocking call : process period <= framerate 60Hz)
        return_value, self.image = self.video_capture.read()

        # reset control state
        self.direction = 128
        self.throttle = 128
        self.mode = 0
        
        # based on AI
        if self.ia:
            self.ai_processing()
            
            #pid management
            self.pid_line = self.pid_line_following.compute(self.line_pos)
            #print(str(self.pid_line))
            print(str(self.pid_line) + ';' + str(self.pid_line_following.kp) + ';' + str(self.pid_line_following.ki) + ';' + str(self.pid_line_following.kd))
            # direction normal rate
            self.direction = int(((self.pid_line+1.0)/2.0*255.0))
        # based on LIDARs    
        else:
            # PID update according positional error given by left and right front LIDARs
            lid = constraint(self.lidar_distance_droit, 0, 150)
            lig = constraint(self.lidar_distance_gauche, 0, 150)
            self.actual_error = constraint(lid - lig, -150, 150)
            self.pid_wall = self.pid_wall_following.compute(self.actual_error)
            self.positional_error = abs(self.pid_wall_following.integral_error)
            ###print("actual_error: " + str(actual_error) + " positional_error: " + str(positional_error) )

            # direction and throttle, control with threshold
            if self.positional_error > self.positional_error_threshold:
                
                # high positional error             
                ###print("high positional error")
                
                # direction normal rate
                self.direction = int(((self.pid_wall+1.0)/2.0*255.0))

                # compute target speed and reduce throttle according position error
                self.target_speed_ms = float(self.max_speed) - self.k_speed*(float)(self.positional_error- self.positional_error_threshold)
                self.target_speed_ms = constraint(self.target_speed_ms,  self.min_speed,  self.max_speed)
                ###print("max_speed:" + str(self.max_speed) + "   delta:" + str(self.k_speed*(float)(positional_error- self.positional_error_threshold)) + "  TRG_SPEED:" + str(self.target_speed_ms))

            else:
            
                # low positional error             
                ###print("low positional error")

                # direction low rate
                self.direction = int(((self.pid_wall*self.dual_rate+1.0)/2.0*255.0))

                # max throttle
                self.target_speed_ms = self.max_speed

            # compute throttle according actual_speed
            self.speed_error_ms = self.target_speed_ms - self.actual_speed_ms
            self.throttle = int( 128.0 + self.pid_speed.compute(self.speed_error_ms) + self.pid_speed_kff *self.target_speed_ms )
            ###print( "target_speed_ms: " + str(self.target_speed_ms) + "    actual_speed_ms: " + str(self.actual_speed_ms) + "    self.speed_error_ms: " + str(self.speed_error_ms) + "    THR:" + str(self.throttle) )
            self.throttle = constraint(self.throttle,  100,  180)
            
        # throttle control
        if self.running:
            self.mode = 1
        else:
            self.throttle = 128
            self.mode = 0
        
        #trim & limits
        self.direction += self.direction_trim
        self.direction = constraint(self.direction,  20,  235) # hard-coded limits
        self.throttle = constraint(self.throttle,  80,  180) # hard-coded limits
        
        # STM32 control link
        self.port.write("{:d};".format(self.direction).encode('ascii') + "{:d};".format(int(self.throttle)).encode('ascii') + "{:d}\r\n".format(self.mode).encode('ascii'))
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
        log_list.append( float(self.lidar_distance_gauche) )
        log_list.append( float(self.lidar_distance_droit) )
        log_list.append( float(self.lidar_distance_haut) )
        log_list.append( float(self.actual_error) )
        log_list.append( float(self.positional_error) )
        log_list.append( float(self.pid_wall) )
        log_list.append( float(self.pid_line) ) 
        log_list.append( float(self.direction) )
        log_list.append( float(self.target_speed_ms) ) 
        log_list.append( float(self.actual_speed_ms) ) 
        log_list.append( float(self.speed_error_ms) ) 
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
        cv2.putText(overlay, "TH:{:d}".format(self.throttle) + "  DR:{:d}".format(self.direction),  (4, 12), cv2.FONT_HERSHEY_PLAIN,1.0, (100, 100, 255), 1)
        cv2.circle(overlay, (int(self.image.shape[1]*(self.pid_wall+1.0)/2.0),int(self.image.shape[0]/2.0)+20), 10, (100, 100, 255), 1)
        cv2.line(overlay, (int(self.new_width/2),0 ),  (int(self.new_width/2),self.new_height), color=(0,0,255) )
        cv2.line(overlay, (0,int(self.new_height/2)),  (self.new_width,int(self.new_height/2)), color=(0,0,255) )
        self.image = cv2.addWeighted(overlay, 0.5, self.image, 0.5, 0)
        self.video_broadcast.write(self.image)

        
    def reload_settings(self):
        global param_lidar_direction_kp
        global param_lidar_direction_ki
        global param_lidar_direction_kd
        global param_ai_direction_kp
        global param_ai_direction_ki
        global param_ai_direction_kd
        global param_speed_kp
        global param_speed_ki
        global param_speed_kd
        global param_speed_kff
        global param_min_speed
        global param_max_speed
        global param_k_speed
        global param_positional_error_threshold
        global param_dual_rate
        global param_direction_trim        
        self.pid_wall_following.kp = param_lidar_direction_kp
        self.pid_wall_following.ki = param_lidar_direction_ki
        self.pid_wall_following.kd = param_lidar_direction_kd
        self.pid_line_following.kp = param_ai_direction_kp
        self.pid_line_following.ki = param_ai_direction_ki
        self.pid_line_following.kd = param_ai_direction_kd
        self.pid_speed.kp = param_speed_kp
        self.pid_speed.ki = param_speed_ki
        self.pid_speed.kd = param_speed_kd
        self.pid_speed_kff = param_speed_kff
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
        msg += str( float(controller.actual_error) ) + ';'  #cm lidar error
        msg += str( float(controller.positional_error) ) + ';'  #cm lidar error integral
        msg += str( float(controller.pid_wall) ) + ';' 
        msg += str( float(controller.pid_line) ) + ';' 
        msg += str( float(controller.direction) ) + ';' 
        msg += str( float(controller.throttle) ) + ';' 
        msg += str( float(controller.mode) ) + ';' 
        msg += str( float(controller.actual_speed_ms) )
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
                "   DIR:" + str(controller.direction) + 
                "   THR:" + str(controller.throttle) + 
                "   MODE:" + str(controller.mode) + 
                "   LiG:" + str(controller.lidar_distance_gauche) +
                "   LiD:" + str(controller.lidar_distance_droit) +
                "   LiH:" + str(controller.lidar_distance_haut) + 
                "   mDIR:" + str(controller.telemetry_manual_dir) + 
                "   mTHR:" + str(controller.telemetry_manual_thr) + 
###                "  Speed:" + str( controller.telemetry_speed  ) +
                "  Speed:" + str( controller.actual_speed_ms   )  + "m/s"
                "  Speed:" + str( controller.actual_speed_kmh  )  + "km/h"
            )
        fps_counter += 1
        
# Le C ca rassure !!
if __name__ == "__main__":
    main()
    
