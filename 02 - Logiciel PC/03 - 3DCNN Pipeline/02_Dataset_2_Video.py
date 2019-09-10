import numpy as np
import cv2

import my_constants as consts
import my_parameters as params
import my_datasettools as dtools


def mousePosition(event,x,y,flags,param):
    if event == cv2.EVENT_MOUSEMOVE:
        print(x,y)

## MAIN ########################################################################

X = np.array([])
Y = np.array([])
for directory in params.dataset_dir:
    print(directory)
    # load dataset file
    Xpictures,Ypictures = dtools.load_dataset(directory,  params.dataset_filename, display = False)
    # concate
    if X.size == 0:
        X = Xpictures
        Y = Ypictures
    else:
        X = np.concatenate((X,Xpictures), axis=0)
        Y = np.concatenate((Y,Ypictures), axis=0)

fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter(params.video_dir + '/' + params.dataset_preview_filename+ '.avi', fourcc, 60.0, (consts.picture_initial_width,consts.picture_initial_height))
cv2.namedWindow('image')
cv2.setMouseCallback('image',mousePosition)
for x, y in zip(X, Y):
    frame = x.reshape(consts.picture_initial_height,consts.picture_initial_width)
    frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)
    overlay = frame.copy()
    output = frame.copy()
    cv2.line(overlay, (0,0), (consts.picture_initial_width,params.picture_height_crop), color=(255, 0, 0), thickness = 1)
    cv2.rectangle(overlay, (0,0), (consts.picture_initial_width,params.picture_height_crop), color=(255, 0, 0), thickness =-1)
    cv2.line(overlay, (0,int(consts.picture_initial_height/2)), (consts.picture_initial_width,int(consts.picture_initial_height/2)), color=(255, 0, 0), thickness = 1)
    cv2.line(overlay, (int(consts.picture_initial_width/2),0), (int(consts.picture_initial_width/2),consts.picture_initial_height), color=(255, 0, 0), thickness = 1)
    cv2.line(overlay, (int((y+1.0)/2.0*consts.picture_initial_width), 0), (int((y+1.0)/2.0*consts.picture_initial_width),consts.picture_initial_height), color=(0, 255, 0), thickness = 2)
    alpha = 0.30
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, output)
    out.write(output)
    cv2.imshow('image',output)
    cv2.waitKey(1)
out.release()
cv2.destroyAllWindows()
