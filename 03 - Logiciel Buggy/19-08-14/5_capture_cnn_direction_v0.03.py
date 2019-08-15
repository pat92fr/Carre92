from threading import Thread
import time
import sys
import cv2
import serial
from keras.models import load_model
import h5py

# parameters
width = 160 
height = 120 
crop_height = 32
height_start_1 = 10 
height_end_1 = height_start_1+crop_height 
height_start_2 = height_end_1
height_end_2 = height_start_2+crop_height 
pid_params = {'kp':1.5, 'kd':3.0, 'kd2':2.0}

# global state
run_ai = True

# IHM
def menu():
    global run_ai

    def print_menu():
        print("current PID params are:" + str(pid_params))
        print("0: exit")
        print("1: change Kp")
        print("2: change Kd")
        print("3: change Kd2")
        #print("Enter your choice [0,3]:")

    loop = True
    print_menu()
    while loop:
        choice = input("Enter your choice [0,3]:")
        if choice == '0':
            loop = False
            run_ai = False
        elif choice == '1':
            print("current value of Kp is:" + str(pid_params['kp']))
            pid_params['kp'] = float( input("Enter Kp:") )
            print(str(pid_params['kp']))
        elif choice == '2':
            print("current value of Kd is:" + str(pid_params['kd']))
            pid_params['kd'] = float( input("Enter Kd:") )
            print(str(pid_params['kd']))
        elif choice == '3':
            print("current value of Kd2 is:" + str(pid_params['kd2']))
            pid_params['kd2'] = float( input("Enter Kd2:") )
            print(str(pid_params['kd2']))
        else:
            print("Wrong menu choice.")
            
class cli(Thread):
    
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        menu()

##main

# load model
model = load_model('model.h5')
model.summary()



# open video stream
camera = cv2.VideoCapture('v4l2src ! video/x-raw,framerate=20/1 ! videoscale ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


out_send = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! rtph264pay ! udpsink host=10.42.0.188 port=5000',cv2.CAP_GSTREAMER,0, 20, (640,480), True)

# open video stream
#camera = cv2.VideoCapture(0)
# camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# open serial
port = serial.Serial("/dev/ttyTHS1",baudrate=115200,timeout=3.0)

thread_cli = cli()
thread_cli.start()

# game loop
last_time = time.time()
frame_counter = 0
last_error = 0
error = 0
fps = 0
while run_ai:
    return_value, image = camera.read()
    assert(image.shape == (480,640,3))
    #resize
    dim = (width,height)
    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    assert(im_resized.shape == (height,width,3))
    # crop
    im_croped_1 = im_resized[height-height_end_1:height-height_start_1,:width]
    assert(im_croped_1.shape == (crop_height,width,3))
    im_croped_2 = im_resized[height-height_end_2:height-height_start_2,:width]
    assert(im_croped_2.shape == (crop_height,width,3))
    #make a prediction
    y_1 = model.predict(im_croped_1.reshape(1,crop_height,width,3)/255.0)[0]
    y_2 = model.predict(im_croped_2.reshape(1,crop_height,width,3)/255.0)[0]
    #print(str(y_1))
    #print(str(y_2))
    # PID

    #send image + line to gstreamer
    cv2.line(image,(int((y_1+1.0)/2.0*640),int((height-height_start_1-crop_height/2)*4)),(int((y_2+1.0)/2.0*640),int((height-height_end_2+crop_height/2)*4)),(255,0,0),5)
    out_send.write(image)

    last_error = error
    error = y_1[0]
    next_error = y_2[0]
    pid = error * pid_params['kp'] + (error-last_error) * pid_params['kd'] + (next_error-error) * pid_params['kd2'] 
    if pid>1.0:
        pid=1.0
    if pid<-1.0:
        pid=-1.0
    #fps
    frame_counter += 1
    if time.time()>=last_time+1.0:
        last_time=time.time()
        print(str(frame_counter))
        fps = frame_counter
        frame_counter = 0
    # serial
    y_byte = int(((pid+1.0)/2.0*256.0))
    #print("fps:" + str(fps) + "   error:" + str(y_1) + "   pid:" + str(pid) + "   servo:" + str(y_byte))
    port.write("{:d}\r\n".format(y_byte).encode('ascii'))  

thread_cli.join()
# when everything done, release the capture
camera.release()

