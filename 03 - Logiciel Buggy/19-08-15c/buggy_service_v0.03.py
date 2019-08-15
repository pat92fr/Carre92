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

class buggyServer:

    def __init__(self):

        self.pid_params = {'kp':1.5, 'kd':3.0, 'kd2':2.0}

        # global state
        self.run_ai = True

        self.s = 0
        self.model = 0
        self.cap_send = 0
        self.out_send = 0

        self.ia_started = False

        self.rec_width = 0
        self.rec_height = 0

        #self.pc_IP ='10.42.0.188' #PC Section Robotique !
        #self.pc_IP ='10.42.0.112' #PC HP portable Patrick !
        self.pc_IP ='192.168.1.34' #PC Home Patrick !
        #self.pc_IP ='10.0.10.50' #test card


        #serial port for STM32 communication 
        self.port = serial.Serial("/dev/ttyTHS1",baudrate=115200,timeout=3.0)

        self.y_1 = 0
        self.y_2 = 0

        self.image = 0
        self.mode = "hires"

        if self.mode == "hires":

            self.rec_width = 1280
            self.rec_height = 720   

            self.new_width = 160
            self.new_height = 90

        if self.mode == "lores":

            self.rec_width = 1280
            self.rec_height = 720

            self.new_width = 160
            self.new_height = 90


        self.init_socket()
        self.init_cam()


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



    def init_IA(self):

        from keras.models import load_model
        import h5py

        # load model
        self.model = load_model('model.h5')
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


    def buggyservice(self):

        # game loop
        last_time = time.time()
        frame_counter = 0
        last_error = 0
        next_error = 0
        error = 0
        fps = 0
        y_byte = 0

        while True:

            while(True):
                value = 2
                self.conn.send(fps.to_bytes(2, byteorder='big') + y_byte.to_bytes(2, byteorder='big'))
                command = self.conn.recv(1024)


                if (command == b'i'):
                    if self.ia_started:
                        print("IA already stared")
                    else:
                        print("initializing IA")
                        self.init_IA()

                if (command == b'q'):
                    print("all done, quitting !")
                    self.s.close()
                    self.cap_send.release()
                    self.out_send.release()
                    return

                return_value, self.image = self.cap_send.read()
                assert(self.image.shape == (self.new_height,self.new_width,3))

                if self.ia_started:
                    self.image_processing()
                    
                    #pid management
                    last_error = error

                    error = error*0.5 + 0.5*self.y_1
                    next_error = next_error*0.5 + 0.5*self.y_2
                    pid = error * self.pid_params['kp'] + (error-last_error) * self.pid_params['kd'] + (next_error-error) * self.pid_params['kd2'] 
                    if pid>1.0:
                        pid=1.0
                    if pid<-1.0:
                        pid=-1.0

                    # serial
                    y_byte = int(((pid+1.0)/2.0*256.0))
                    ###print("fps:" + str(fps) + "   error:" + str(self.y_1) + "   pid:" + str(pid) + "   servo:" + str(y_byte))
                    self.port.write("{:d}\r\n".format(y_byte).encode('ascii'))

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
