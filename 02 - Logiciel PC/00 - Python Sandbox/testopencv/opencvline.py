import cv2
import numpy as np

depth = 3
width = 640
height_crop = 128
height = 480
cap = cv2.VideoCapture(0)
buffer_image = np.zeros((10,height_crop,width),dtype='float')
while True:
    # Read image 
    ret, image = cap.read()
    ###print(str(image.shape))
    image = cv2.resize(image,(width,height))
    ###print(str(image.shape))
    resized = image[int(height/2):int(height/2)+height_crop, 0:width]
    ###print(str(resized.shape))
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    ###print(str(gray.shape))

    # TODO : normalize
    #gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)

    # time buffer
    buffer_image[0:depth-1] = buffer_image[1:depth]
    buffer_image[depth-1] = gray
    integral_image = buffer_image.sum(axis=0)/depth
    ###print(str(integral_image.shape))
    integral_image = np.uint8(integral_image)

    #kernel = np.ones((5,5),np.float32)/25
    blur = cv2.GaussianBlur(integral_image,(5,5),0) #cv2.filter2D(integral_image,-1,kernel)
    ###print(str(blur.shape))
     
    edges = cv2.Canny(blur, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 40, minLineLength=30, maxLineGap=10)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(resized, (x1, y1), (x2, y2), (255, 0, 0), 3)

    output_image = cv2.resize(resized, (width,height_crop))
    cv2.imshow('Webcam',output_image)
    cv2.resizeWindow('Webcam', width,height_crop)
    cv2.moveWindow("Webcam", 20,20);

    output_gray = cv2.resize(gray, (width,height_crop))
    cv2.imshow('Gray',output_gray)
    cv2.resizeWindow('Gray', width,height_crop)
    cv2.moveWindow("Gray", 20,20+(height_crop+20)*1);

    output_integral_image = cv2.resize(integral_image, (width,height_crop))
    cv2.imshow('Integral',output_integral_image)
    cv2.resizeWindow('Integral', width,height_crop)
    cv2.moveWindow("Integral", 20,20+(height_crop+20)*2);

    output_blur = cv2.resize(blur, (width,height_crop))
    cv2.imshow('blur',output_blur)
    cv2.resizeWindow('blur', width,height_crop)
    cv2.moveWindow("blur", 20,20+(height_crop+20)*3);
    
    output_edges = cv2.resize(edges, (width,height_crop))
    cv2.imshow('Canny',output_edges)
    cv2.resizeWindow('Canny', width,height_crop)
    cv2.moveWindow("Canny", 20,20+(height_crop+20)*4);
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

