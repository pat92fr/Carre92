import cv2
import time
from PIL import Image

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 30)
last_time = time.time()
frame_counter = 0
fps = 0
if camera.isOpened():
  while True:
      return_value, image = camera.read()
      cv2.imshow('CSI Camera',image)
      keyCode = cv2.waitKey(1) & 0xff
      #fps counter
      frame_counter += 1
      if time.time()>=last_time+1.0:
        last_time=time.time()
        print(str(frame_counter))
        fps = frame_counter
        frame_counter = 0   
  camera.release()
  cv2.destroyAllWindows()

