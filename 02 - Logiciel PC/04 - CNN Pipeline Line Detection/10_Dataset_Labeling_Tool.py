## PARAMETER ###################################################################

dataset_list = [ "dataset_002" ] # list the dataset directory names to be labelled
display_ratio = 8       # picture zoom
framerate_ratio = 0.9   # framerate slow down < 1.0

## GLOBAL ######################################################################

import numpy as np
import cv2
import os
import shutil
import my_constants as consts
import my_parameters as params
import my_datasettools as dtools


## FUNCTION ####################################################################

mouse_x = 0.0
def mousePosition(event,x,y,flags,param):
    global mouse_x
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x = float(x)/(consts.picture_initial_width*display_ratio)*2.0-1.0

## MAIN ########################################################################

# init
cv2.namedWindow('image')
cv2.setMouseCallback('image',mousePosition)

# for each dataset in the list
for dataset_dir in dataset_list:
    print(dataset_dir)
    
    # load curent dataset.txt 
    print("Load dataset file...")
    dataset_file = open(params.base_dataset_directory+dataset_dir+"/"+consts.dataset_filename, "r")
    content = dataset_file.read()
    dataset_file.close()
    print("Done.")

    # parse curentdataset.txt 
    print("Parse dataset file...")
    lines = content.splitlines()
    m = len(lines)
    print(" m=" + str(m) + " examples")

    # for each picture of current dataset
    X = []
    Y = []
    frame_counter = 0
    for l in lines:
        fields = l.split(';')
        filename = fields[0]
        # load picture
        x = dtools.load_and_preprocess_picture(params.base_dataset_directory+filename)
        # display example
        frame = x.reshape(consts.picture_initial_height,consts.picture_initial_width)
        frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)
        # OSD
        overlay = frame.copy()
        output = frame.copy()
        cv2.line(overlay, (0,0), (consts.picture_initial_width,consts.picture_height_crop), color=(255, 0, 0), thickness = 1)
        cv2.rectangle(overlay, (0,0), (consts.picture_initial_width,consts.picture_height_crop), color=(255, 0, 0), thickness =-1)
        cv2.line(overlay, (0,int(consts.picture_initial_height/2)), (consts.picture_initial_width,int(consts.picture_initial_height/2)), color=(255, 0, 0), thickness = 1)
        #cv2.line(overlay, (int(consts.picture_initial_width/2),0), (int(consts.picture_initial_width/2),consts.picture_initial_height), color=(255, 0, 0), thickness = 1)
        cv2.putText(overlay, "{:d}".format(frame_counter) + "/{:d}".format(m),(4, 12), cv2.FONT_HERSHEY_PLAIN,1.0, (100, 100, 255), 1)
        alpha = 0.40
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, output)
        resized = cv2.resize(output, (consts.picture_initial_width*display_ratio,consts.picture_initial_height*display_ratio), interpolation = cv2.INTER_AREA)
        cv2.imshow('image',resized)
        # delay
        if frame_counter == 0: # first frame
            cv2.waitKey(3000)
        else: # next frames
            cv2.waitKey(int(1000.0/60.0*1.0/framerate_ratio))
        X.append(filename)
        Y.append(mouse_x)
        
        # trace
        print(str(frame_counter)+'/'+str(m)+' tag:'+str(mouse_x))
        
        frame_counter += 1
        #if frame_counter == 680:
        #    break
    print("Done.")

    # backup previous label.txt
    label_full_path_name = params.base_dataset_directory+dataset_dir+"/"+consts.label_filename
    if os.path.isfile(label_full_path_name):
        shutil.copy(label_full_path_name,label_full_path_name+'.bak')

    # write label.txt
    print("Build label file...")
    label_file = open(params.base_dataset_directory+dataset_dir+"/"+consts.label_filename, "w")
    for x, y in zip(X, Y):
        label_file.write(x+';'+str(float(y))+'\n')
    label_file.close()
    print("Done.")

#close
cv2.destroyAllWindows()
