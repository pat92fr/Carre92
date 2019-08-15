import numpy as np
import math
import cv2
from keras.models import load_model
import h5py
import time
import serial
from PIL import Image
import threading

def gstreamer_pipeline (capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=60, flip_method=0) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))

class VideoCaptureAsync:
    def __init__(self, width=160, height=90):
        #self.cap = cv2.VideoCapture(self.src)
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap = cv2.VideoCapture(gstreamer_pipeline(capture_width=1280, capture_height=720, display_width=width, display_height=height, framerate=60, flip_method=2), cv2.CAP_GSTREAMER)
        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()

    def set(self, var1, var2):
        self.cap.set(var1, var2)

    def start(self):
        if self.started:
            print('[!] Asynchroneous video capturing has already been started.')
            return None
        self.started = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def stop(self):
        self.started = False
        self.thread.join()

    def __exit__(self, exec_type, exc_value, traceback):
        self.cap.release()

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
model.summary()

# open video stream
camera = VideoCaptureAsync(width,height)
camera.start()

# loop
### 64 fps with serial without predict without imshow
### 51 fps with imshow and serial without predict

last_time = time.time()
frame_counter = 0
fps = 0
try:
  while(True):
    return_value, image = camera.read()
    ###image = np.zeros((90,160,3))
    #print(str(image.shape))
    assert(image.shape == (90,160,3))
    #resize
    #dim = (width,height)
    #im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    #assert(im_resized.shape == (height,width,3))

    # crop
    im_croped_1 = image[height-height_end_1:height-height_start_1,:width]
    assert(im_croped_1.shape == (crop_height,width,3))
    im_croped_2 = image[height-height_end_2:height-height_start_2,:width]
    assert(im_croped_2.shape == (crop_height,width,3))

    #make a prediction
    x_1 = im_croped_1.reshape(crop_height,width,3)/255.0
    x_2 = im_croped_2.reshape(crop_height,width,3)/255.0
    x = np.array([x_1, x_2])
    y = model.predict(x)
    y_1 = y[0]
    y_2 = y[1]
    #print(str(y_1))
    #print(str(y_2))

    #view
    if False:
      cv2.imshow('frame',image)
      # stop condition
      cv2.line(image,(int((y_1+1.0)/2.0*640),int((height-height_start_1-crop_height/2)*4)),(int((y_2+1.0)/2.0*640),int((height-height_end_2+crop_height/2)*4)),(255,0,0),5)
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break
    
    #fps
    frame_counter += 1
    if time.time()>=last_time+1.0:
        last_time=time.time()
        fps = frame_counter
        frame_counter = 0

    # serial
    y_byte = int(((y_1+1.0)/2.0*256.0)[0])
    print("dir:" + str(y_byte)+ " "+ "  fps:" + str(fps))
    port.write("{:d}\r\n".format(y_byte).encode('ascii'))    

except KeyboardInterrupt:
  print('\nDone')
# When everything done, release the capture
camera.stop()
cv2.destroyAllWindows()
