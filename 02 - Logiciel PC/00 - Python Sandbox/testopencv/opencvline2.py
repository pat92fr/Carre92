import cv2
import math
import numpy as np

###https://wroscoe.github.io/compound-eye-autopilot.html

width = 320
height_crop = 96
height = 240

blur_kernel_size = 11
canny_threshold1 = 100
canny_threshold2 = 130
hough_rho = 6.
hough_theta = 6.0*np.pi/180.0
hough_threshold = 20
hough_min_line_length = 40
hough_max_gap = 10

def findparallel(lines):
    lines1 = []
    for i in range(len(lines)):
        for j in range(i,len(lines)):
            if (i == j):continue
            if (abs(get_orientation(lines[i][0]) - get_orientation(lines[j][0])) <= 4.0):          
                 #You've found a parallel line!
                 lines1.append((i))
    return lines1

def get_orientation(line):
    '''get orientation of a line, using its length
       https://en.wikipedia.org/wiki/Atan2
    '''
    orientation = math.atan2(abs((line[0] - line[2])), abs((line[1] - line[3])))
    return math.degrees(orientation)
    
cap = cv2.VideoCapture(0)
while True:
    # Read image 
    ret, image = cap.read()
    ###print(str(image.shape))
    image = cv2.resize(image,(width,height))
    resized = image[int(height/2)-int(height_crop/2):int(height/2)+int(height_crop/2), 0:width]
    image2 = resized.copy()
    #remove colors and show greyscale
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    #blur images to avoid recognizing small lines
    blur = cv2.GaussianBlur(gray,(blur_kernel_size,blur_kernel_size),0)
    #use canny threshold to find edges of shapes
    edges = cv2.Canny(blur, 100, 130)
    #use hough trans to find lines
    lines = cv2.HoughLinesP(edges, rho = hough_rho, theta = hough_theta, threshold = hough_threshold, minLineLength = hough_min_line_length, maxLineGap = hough_max_gap)
    line_coord_arr = []
    line_count = 0
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            line_coord = np.array([[[x1, y1], [x2, y2]]], dtype=float)
            line_count += 1
            cv2.line(image2, (x1, y1), (x2, y2), (255, 0, 0), 3)
            ##print(str(get_orientation(line[0])))

        ##print(str(len(lines)))
        parallel_lines = findparallel(lines)
        print(str(len(parallel_lines))+"/"+str(len(lines)))


    output_image = cv2.resize(resized, (width,height_crop))
    cv2.imshow('Webcam',output_image)
    cv2.resizeWindow('Webcam', width,height_crop)
    cv2.moveWindow("Webcam", 20,20);

    output_gray = cv2.resize(gray, (width,height_crop))
    cv2.imshow('Gray',output_gray)
    cv2.resizeWindow('Gray', width,height_crop)
    cv2.moveWindow("Gray", 20,20+(height_crop+20)*1);

    output_blur = cv2.resize(blur, (width,height_crop))
    cv2.imshow('blur',output_blur)
    cv2.resizeWindow('blur', width,height_crop)
    cv2.moveWindow("blur", 20,20+(height_crop+20)*2);

    output_edges = cv2.resize(edges, (width,height_crop))
    cv2.imshow('edges',output_edges)
    cv2.resizeWindow('edges', width,height_crop)
    cv2.moveWindow("edges", 20,20+(height_crop+20)*3);
    
    output_image2 = cv2.resize(image2, (width,height_crop))
    cv2.imshow('lines',output_image2)
    cv2.resizeWindow('lines', width,height_crop)
    cv2.moveWindow("lines", 20,20+(height_crop+20)*4);
    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

