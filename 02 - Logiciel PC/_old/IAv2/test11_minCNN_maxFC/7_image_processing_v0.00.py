import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
import random
from datetime import datetime
from IPython.display import SVG
import os

### parameters
##width = 160 # 1280 / 8
##height = 32 #
##inmage_shape = (height, width, 3)
# input picture
width = 160 # 1280 / 8
height = 90 # 720 / 8
crop_height = 32 # power of 2
height_start_1 = 10 # low offset from bottom
height_end_1 = height_start_1+crop_height # high offset from bottom
height_start_2 = 42
height_end_2 = height_start_2+crop_height

random.seed(datetime.now())

def do_canny(frame):
    # Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    plt.imshow(gray)
    plt.pause(2)
    # Applies a 5x5 gaussian blur with deviation of 0 to frame - not mandatory since Canny will do this for us
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    plt.imshow(blur)
    plt.pause(2)    
    # Applies Canny edge detector with minVal of 50 and maxVal of 150
    canny = cv2.Canny(blur, 50, 150)
    # find otsu's threshold value with OpenCV function
    ret, otsu = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    return canny, otsu

list = os.listdir('images_test/')
for i in list:
    print("images_test/"+i)
    image = cv2.imread("images_test/"+i,cv2.IMREAD_COLOR)
    assert(image.shape == (720,1280,3))
    # resize
    dim = (width,height)
    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)    
    # to gray
    image_gray = cv2.cvtColor(im_resized, cv2.COLOR_RGB2GRAY)
    # applies a 5x5 gaussian blur with deviation of 0 to frame - not mandatory since Canny will do this for us
    image_blur = cv2.GaussianBlur(image_gray, (5, 5), 0)
    image_equ = cv2.equalizeHist(image_blur)
    #image_edged = cv2.Canny(image_blur, 300, 255)
    #plt.imshow(image_blur, cmap='gray')
    #plt.imshow(image_edged, cmap='gray')
    #plt.pause(2)

    #hsl = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    #lower_white = np.array([0,100,0], dtype=np.uint8)
    #upper_white = np.array([255,255,255], dtype=np.uint8)
    #mask = cv2.inRange(hsl, lower_white, upper_white)
    #res = cv2.bitwise_and(image,image, mask= mask)

    mask = cv2.inRange(image_equ, 200, 255)
    res = cv2.bitwise_and(image_equ,image_equ, mask= mask)
    

    plt.imshow(res, cmap='gray')
    plt.pause(2)
    #cv2.imshow('frame',image)
    #cv2.imshow('mask',mask)
    #cv2.imshow('res',res)

    
##    # resize
##    dim = (width,height)
##    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
##    assert(im_resized.shape == (height,width,3))
##    # crop
##    im_croped_1 = im_resized[height-height_end_1:height-height_start_1,:width]
##    assert(im_croped_1.shape == (crop_height,width,3))
##    plt.imshow(im_croped_1)
##    plt.pause(2)
##
##    im_canny, im_otsu = do_canny(im_croped_1)
##    print(str(im_canny.shape))
##
##    # cv.HoughLinesP(frame, distance resolution of accumulator in pixels (larger = less precision), angle resolution of accumulator in radians (larger = less precision), threshold of minimum number of intersections, empty placeholder array, minimum length of line in pixels, maximum distance in pixels between disconnected lines)
##    #hough = cv2.HoughLinesP(im_canny, 2, np.pi / 180, 100, np.array([]), minLineLength = 100, maxLineGap = 50)
##
##
##
##    plt.imshow(im_canny)
##    plt.pause(2)
##    plt.imshow(im_otsu)
##    plt.pause(2)
    
### load dataset
##X = np.transpose(np.loadtxt("Xtrain.txt", dtype=float))
##Y = np.transpose(np.loadtxt('Ytrain.txt', dtype=float))
##m = X.shape[0]
##print(str(m))
##
##X_train = X.reshape(m,height,width,3)
##Y_train = Y
##
##for i in range(0,m):
##    x = X[i]
##    plt.imshow(x.reshape((height, width, 3)))
##    plt.show()
