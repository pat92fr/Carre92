import numpy as np
import cv2

import my_constants as consts
import my_parameters as params
import my_datasettools as dtools

## GLOBAL ########################################################################

display_ratio = 8

mouse_x = 0.0

def mousePosition(event,x,y,flags,param):
    global mouse_x
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x = float(x)/(consts.picture_initial_width*display_ratio)*2.0-1.0
        #print(x,y)

## MAIN ########################################################################

cv2.namedWindow('image')
cv2.setMouseCallback('image',mousePosition)
for dataset_dir in params.dataset_dir:
    print(dataset_dir)
    # load dataset.txt 
    print("Load dataset file...")
    dataset_file = open(dataset_dir+"/"+params.dataset_filename, "r")
    content = dataset_file.read()
    dataset_file.close()
    print("Done.")
    # parse dataset.txt 
    print("Parse dataset file...")
    lines = content.splitlines()
    m = len(lines)
    print(" m=" + str(m) + " examples")
    X = []
    Y = []
    # display and capture mouse position
    frame_counter = 0
    for l in lines:
        fields = l.split(';')
        filename = fields[0]
        # load picture
        x = dtools.load_and_preprocess_picture(filename)
        # display example
        frame = x.reshape(consts.picture_initial_height,consts.picture_initial_width)
        frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)
        overlay = frame.copy()
        output = frame.copy()
        cv2.line(overlay, (0,0), (consts.picture_initial_width,params.picture_height_crop), color=(255, 0, 0), thickness = 1)
        cv2.rectangle(overlay, (0,0), (consts.picture_initial_width,params.picture_height_crop), color=(255, 0, 0), thickness =-1)
        cv2.line(overlay, (0,int(consts.picture_initial_height/2)), (consts.picture_initial_width,int(consts.picture_initial_height/2)), color=(255, 0, 0), thickness = 1)
        #cv2.line(overlay, (int(consts.picture_initial_width/2),0), (int(consts.picture_initial_width/2),consts.picture_initial_height), color=(255, 0, 0), thickness = 1)
        alpha = 0.40
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, output)
        resized = cv2.resize(output, (consts.picture_initial_width*display_ratio,consts.picture_initial_height*display_ratio), interpolation = cv2.INTER_AREA)
        cv2.imshow('image',resized)
        if frame_counter == 0:
            cv2.waitKey(3000)
        else:
            cv2.waitKey(int(1000/60))
        X.append(filename)
        Y.append(mouse_x)
        print(str(frame_counter)+'/'+str(m)+' tag:'+str(mouse_x))
        frame_counter += 1
        ##if frame_counter == 680:
        ##    break
    print("Done.")
    # open label.txt
    print("Build label file...")
    label_file = open(dataset_dir+"/"+params.label_filename, "w")
    for x, y in zip(X, Y):
        label_file.write(x+';'+str(float(y))+'\n')
    label_file.close()
    print("Done.")
cv2.destroyAllWindows()
