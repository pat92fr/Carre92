import cv2
import numpy as np

cap = cv2.VideoCapture(1)
while True:
    # Read image 
    ret, image = cap.read()
    image = cv2.resize(image, (240,320))
    image = image[120:120+64, 0:320]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((5,5),np.float32)/25
    gray = cv2.filter2D(gray,-1,kernel)
    #gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    
    edges = cv2.Canny(gray, 20, 120)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 40, minLineLength=30, maxLineGap=10)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 3)

    output = cv2.resize(image, (640,256))
    cv2.imshow('Webcam',output)
    cv2.resizeWindow('Webcam', 600,256)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

