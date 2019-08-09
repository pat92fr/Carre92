import numpy as np
import math
import cv2
from keras.models import load_model
import h5py
import time
import serial
from PIL import Image

# serial
port = serial.Serial("/dev/ttyTHS1",baudrate=115200,timeout=3.0)

# parameters
width = 160 
height = 120 
crop_height = 32
height_start_1 = 10 
height_end_1 = height_start_1+crop_height 
height_start_2 = height_end_1
height_end_2 = height_start_2+crop_height 

# load model
model = load_model('model.h5')
print("Loaded model from disk")
# summarize model.
model.summary()

# open video stream
camera = cv2.VideoCapture('v4l2src ! video/x-raw,framerate=20/1 ! videoscale ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
last_time = time.time()
frame_counter = 0

out_send = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! rtph264pay ! udpsink host=10.42.0.188 port=5000',cv2.CAP_GSTREAMER,0, 20, (640,480), True)


while(True):
    return_value, image = camera.read()
    #cv2.imshow('frame',image)
    assert(image.shape == (480,640,3))
    #print(str(image.shape))
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
    cv2.line(image,(int((y_1+1.0)/2.0*640),int((height-height_start_1-crop_height/2)*4)),(int((y_2+1.0)/2.0*640),int((height-height_end_2+crop_height/2)*4)),(255,0,0),5)
    #cv2.imshow('frame',image)
    #img2 = Image.fromarray(image, 'RGB')
    
    out_send.write(image)
    #img2.show()
    #fps
    frame_counter += 1
    if time.time()>=last_time+1.0:
        last_time=time.time()
        print(str(frame_counter))
        frame_counter = 0
    # serial
    y_byte = int(((y_1+1.0)/2.0*256.0)[0])
    print(str(y_byte))
    port.write("{:d}\r\n".format(y_byte).encode('ascii'))    
    # stop condition
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
camera.release()
cv2.destroyAllWindows()
