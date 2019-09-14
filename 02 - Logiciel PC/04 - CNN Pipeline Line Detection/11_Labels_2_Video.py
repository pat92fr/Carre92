## PARAMETER ###################################################################

dataset_list = [ "dataset_002" ] # list the dataset directory names to be labelled

## GLOBAL ######################################################################

import numpy as np
import cv2
import os
import shutil
import my_constants as consts
import my_parameters as params
import my_datasettools as dtools

## MAIN ########################################################################

# mkdir
video_directory = params.base_dataset_directory + '/' + consts.video_directory
if not os.path.exists(video_directory):
        os.makedirs(video_directory)

# backup previous video
video_full_path_name = video_directory + '/' + consts.dataset_preview_filename
if os.path.isfile(video_full_path_name):
    shutil.copy(video_full_path_name,video_full_path_name +'.bak')

# init
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter(
    video_full_path_name, 
    fourcc, 60.0, 
    (consts.picture_initial_width,consts.picture_initial_height)
)
cv2.namedWindow('image')


# for each dataset in the list
for dataset_dir in dataset_list:
    print(dataset_dir)

    # load label.txt 
    print("Load label file...")
    label_file = open(params.base_dataset_directory+dataset_dir+'/'+consts.label_filename, "r")
    content = label_file.read()
    label_file.close()
    print("Done.")

    # parse label.txt 
    print("Parse label file...")
    lines = content.splitlines()
    m = len(lines)
    print(" m=" + str(m) + " examples")

    # for each example (picture,linepos) of current dataset
    frame_counter = 0
    for l in lines:
        fields = l.split(';')
        filename = fields[0]
        y = float(fields[1])
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
        cv2.line(overlay, (int(consts.picture_initial_width/2),0), (int(consts.picture_initial_width/2),consts.picture_initial_height), color=(255, 0, 0), thickness = 1)
        cv2.line(overlay, (int((y+1.0)/2.0*consts.picture_initial_width), 0), (int((y+1.0)/2.0*consts.picture_initial_width),consts.picture_initial_height), color=(0, 255, 0), thickness = 2)
        cv2.putText(overlay, "{:d}".format(frame_counter) + "/{:d}".format(m),(4, 12), cv2.FONT_HERSHEY_PLAIN,1.0, (100, 100, 255), 1)
        alpha = 0.40
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, output)
        out.write(output)
        cv2.imshow('image',output)
        # delay
        cv2.waitKey(1)
        frame_counter += 1

#close
out.release()
cv2.destroyAllWindows()
