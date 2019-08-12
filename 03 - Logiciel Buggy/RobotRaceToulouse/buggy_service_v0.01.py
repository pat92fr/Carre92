from threading import Thread
import time
import sys
import cv2
import serial
import socket


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

        self.pc_IP ='10.42.0.188' #buggy !
        self.pc_IP ='10.0.10.50' #test card


        #serial port for STM32 communication 
        self.port = serial.Serial("/dev/ttyTHS1",baudrate=115200,timeout=3.0)


        

        self.y_1 = 0
        self.y_2 = 0

        self.image = 0
        self.mode = "lores"

        if self.mode == "hires":

            self.rec_width = 1280
            self.rec_height = 720   

            self.new_width = 160
            self.new_height = 90

        if self.mode == "lores":

            self.rec_width = 640
            self.rec_height = 480

            self.new_width = 160
            self.new_height = 120


        self.init_socket()
        self.init_cam()


    def init_socket(self):

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

        self.cap_send = cv2.VideoCapture(0)
        self.cap_send.set(cv2.CAP_PROP_FRAME_WIDTH, self.rec_width)
        self.cap_send.set(cv2.CAP_PROP_FRAME_HEIGHT, self.rec_height)
        self.cap_send.set(cv2.CAP_PROP_FPS, 30)

        if self.mode == "hires":
            fourcc = cv2.VideoWriter_fourcc('M','J','P','G')

        return_value, self.image = self.cap_send.read()

        assert(self.image.shape == (self.rec_height,self.rec_width,3))

        #int gstreamer sink
        self.out_send = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=200 ! queue ! rtph264pay config-inerval=1 ! udpsink host='+self.pc_IP+' port=5500 sync=false',cv2.CAP_GSTREAMER,0, 20, (self.new_width,self.new_height), True)

    def image_processing(self):


        # parameters
        crop_height = 32
        height_start_1 = 0 
        height_end_1 = height_start_1+crop_height 
        height_start_2 = height_end_1
        height_end_2 = height_start_2+crop_height 

        # process image before IA


        # crop
        im_croped_1 = self.image[self.new_height-height_end_1:self.new_height-height_start_1,:self.new_width]
        assert(im_croped_1.shape == (crop_height,self.new_width,3))
        im_croped_2 = self.image[self.new_height-height_end_2:self.new_height-height_start_2,:self.new_width]
        assert(im_croped_2.shape == (crop_height,self.new_width,3))
        
        #IA processing
        self.y_1 = self.model.predict(im_croped_1.reshape(1,crop_height,self.new_width,3)/255.0)[0]
        self.y_2 = self.model.predict(im_croped_2.reshape(1,crop_height,self.new_width,3)/255.0)[0]

        # draw line on image
        pt1_x = int((self.y_1+1.0)/2.0*self.new_width)
        pt1_y = int((self.new_height-height_start_1-(crop_height/2)))

        pt2_x = int((self.y_2+1.0)/2.0*self.new_width)
        pt2_y = int((self.new_height-height_end_2+(crop_height/2)))

        print ('coordonées : pt1({}, {}), pt2({},{})'.format(pt1_x,pt1_y,pt2_x,pt2_y))

        #cv2.line(self.image,(int((self.y_1+1.0)/2.0*self.new_width),int((self.new_height-height_start_1-crop_height/2)*4)),(int((self.y_2+1.0)/2.0*self.new_width),int((self.new_height-height_end_2+crop_height/2)*4)),(255,0,0),5)
        cv2.line(self.image,(pt1_x,pt1_y),(pt2_x,pt2_y),(255,0,0),5)


    def buggyservice(self):



        # game loop
        last_time = time.time()
        frame_counter = 0
        last_error = 0
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

                #resize
                dim = (self.new_width,self.new_height)
                self.image = cv2.resize(self.image, dim, interpolation = cv2.INTER_AREA)
                assert(self.image.shape == (self.new_height,self.new_width,3))
     

                if self.ia_started:
                    self.image_processing()
                    
                    #pid management
                    last_error = error

                    error = self.y_1
                    next_error = self.y_2
                    pid = error * self.pid_params['kp'] + (error-last_error) * self.pid_params['kd'] + (next_error-error) * self.pid_params['kd2'] 
                    if pid>1.0:
                        pid=1.0
                    if pid<-1.0:
                        pid=-1.0

                    # serial
                    y_byte = int(((pid+1.0)/2.0*256.0))
                    #print("fps:" + str(fps) + "   error:" + str(self.y_1) + "   pid:" + str(pid) + "   servo:" + str(y_byte))
                    self.port.write("{:d}\r\n".format(y_byte).encode('ascii'))

                self.out_send.write(self.image)

                #fps counter
                frame_counter += 1
                if time.time()>=last_time+1.0:
                    last_time=time.time()
                    #print(str(frame_counter))
                    fps = frame_counter
                    frame_counter = 0

def main():

    serv = buggyServer()
    serv.buggyservice()

if __name__ == "__main__":
    main()
