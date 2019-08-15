import numpy as np
import math
import cv2
from keras.models import load_model
import h5py
import time
import serial
from PIL import Image

def gstreamer_pipeline (capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=60, flip_method=0) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))

# serial
port = serial.Serial("/dev/ttyTHS1",baudrate=115200,timeout=3.0)

# parameters
width = 160 
height = 90 
crop_height = 32
height_start_1 = 0 
height_end_1 = height_start_1+crop_height 
height_start_2 = height_end_1
height_end_2 = height_start_2+crop_height 

# load model
model = load_model('model.h5')
print("Loaded model from disk")
# summarize model.
model.summary()

# open video stream
###camera = cv2.VideoCapture(0)
camera = cv2.VideoCapture(gstreamer_pipeline(capture_width=1280, capture_height=720, display_width=320, display_height=180, framerate=60, flip_method=2), cv2.CAP_GSTREAMER)

#camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
last_time = time.time()
frame_counter = 0
fps = 0
while(True):
    return_value, image = camera.read()
    #cv2.imshow('frame',image)
    #print(str(image.shape))
    assert(image.shape == (180,320,3))
    #print(str(image.shape))
    #resize
    dim = (width,height)
    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    #assert(im_resized.shape == (height,width,3))
    
    # crop
    im_croped_1 = im_resized[height-height_end_1:height-height_start_1,:width]
    assert(im_croped_1.shape == (crop_height,width,3))
    im_croped_2 = im_resized[height-height_end_2:height-height_start_2,:width]
    assert(im_croped_2.shape == (crop_height,width,3))
    #make a prediction
    y_1 = model.predict(im_croped_1.reshape(1,crop_height,width,3)/255.0)[0]
    #y_2 = model.predict(im_croped_2.reshape(1,crop_height,width,3)/255.0)[0]
    #print(str(y_1))
    #print(str(y_2))
    #cv2.line(image,(int((y_1+1.0)/2.0*640),int((height-height_start_1-crop_height/2)*4)),(int((y_2+1.0)/2.0*640),int((height-height_end_2+crop_height/2)*4)),(255,0,0),5)
    #cv2.imshow('frame',image)
    #img2 = Image.fromarray(image, 'RGB')
    #img2.show()
    #fps
    frame_counter += 1
    if time.time()>=last_time+1.0:
        last_time=time.time()
        #print(str(frame_counter))
        fps = frame_counter
        frame_counter = 0
    # serial
    y_byte = int(((y_1+1.0)/2.0*256.0)[0])
    print("dir:" + str(y_byte)+ " "+ "  fps:" + str(fps))
    port.write("{:d}\r\n".format(y_byte).encode('ascii'))    
    # stop condition
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break

# When everything done, release the capture
camera.release()
cv2.destroyAllWindows()
