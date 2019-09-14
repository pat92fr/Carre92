import numpy as np
import cv2
from keras import models
from keras.models import load_model

import my_constants as consts
import my_parameters as params
import my_datasettools as dtools

## GLOBALS ########################################################################

fifo_shape = (params.depth*params.skip,consts.picture_initial_height,consts.picture_initial_width,1)
fifo = np.zeros( fifo_shape )
sequence_shape = (params.depth,consts.picture_initial_height,consts.picture_initial_width,1)
xsequence = np.zeros( sequence_shape )

## MAIN ########################################################################

# open model
print("Load model from disk ...")
model = load_model(params.model_dir+"/"+params.model_filename + ".h5")
model.summary()
print("Done.")
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter(params.video_dir + '/' + params.dataset_prediction_filename+ '.avi', fourcc, 60.0, (consts.picture_initial_width,consts.picture_initial_height))
cv2.namedWindow('image')
for directory in params.testset_dir:
    print(directory)
    # load label.txt 
    print("Load label file...")
    label_file = open(directory+"/"+params.label_filename, "r")
    content = label_file.read()
    label_file.close()
    print("Done.")
    # parse label.txt 
    print("Parse label file...")
    lines = content.splitlines()
    m = len(lines)
    print(" m=" + str(m) + " examples")
    # display
    frame_counter = 0
    for l in lines:
        fields = l.split(';')
        filename = fields[0]
        y_ref = float(fields[1])
        # load picture
        x = dtools.load_and_preprocess_picture(filename)
        yprediction = model.predict(x.reshape(1,consts.picture_initial_height,consts.picture_initial_width,1))
        y = yprediction.item(0)
        # display example
        frame = x.reshape(consts.picture_initial_height,consts.picture_initial_width)
        frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)
        overlay = frame.copy()
        output = frame.copy()
        cv2.line(overlay, (0,0), (consts.picture_initial_width,params.picture_height_crop), color=(255, 0, 0), thickness = 1)
        cv2.rectangle(overlay, (0,0), (consts.picture_initial_width,params.picture_height_crop), color=(255, 0, 0), thickness =-1)
        cv2.line(overlay, (0,int(consts.picture_initial_height/2)), (consts.picture_initial_width,int(consts.picture_initial_height/2)), color=(255, 0, 0), thickness = 1)
        cv2.line(overlay, (int(consts.picture_initial_width/2),0), (int(consts.picture_initial_width/2),consts.picture_initial_height), color=(255, 0, 0), thickness = 1)
        cv2.line(overlay, (int((y+1.0)/2.0*consts.picture_initial_width), 0), (int((y+1.0)/2.0*consts.picture_initial_width),consts.picture_initial_height), color=(0, 0, 255), thickness = 2)
        cv2.line(overlay, (int((y_ref+1.0)/2.0*consts.picture_initial_width), 0), (int((y_ref+1.0)/2.0*consts.picture_initial_width),consts.picture_initial_height), color=(0, 255, 0), thickness = 2)
        alpha = 0.40
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, output)
        out.write(output)
        cv2.imshow('image',output)
        cv2.waitKey(1)
out.release()
cv2.destroyAllWindows()

