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
# 
X = np.array([])
Y = np.array([])
for directory in params.testset_dir:
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
out = cv2.VideoWriter(params.video_dir + '/' + params.dataset_prediction_filename+ '.avi', fourcc, 60.0, (consts.picture_initial_width,consts.picture_initial_height))
for x, y in zip(X, Y):
    # fifo
    fifo[:-1] = fifo[1:]; fifo[-1] = x
    assert(fifo.shape == fifo_shape)
    xsequence = fifo[0:params.depth*params.skip:params.skip]
    assert(xsequence.shape == sequence_shape)
    #prediction
    yprediction = model.predict(xsequence.reshape(1,params.depth,consts.picture_initial_height,consts.picture_initial_width,1))
    dir = yprediction.item(0)
    # display & osd
    frame = x.reshape(consts.picture_initial_height,consts.picture_initial_width)
    frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)
    overlay = frame.copy()
    output = frame.copy()
    cv2.line(overlay, (0,0), (consts.picture_initial_width,params.picture_height_crop), color=(255, 0, 0), thickness = 1)
    cv2.rectangle(overlay, (0,0), (consts.picture_initial_width,params.picture_height_crop), color=(255, 0, 0), thickness =-1)
    cv2.line(overlay, (0,int(consts.picture_initial_height/2)), (consts.picture_initial_width,int(consts.picture_initial_height/2)), color=(255, 0, 0), thickness = 1)
    cv2.line(overlay, (int(consts.picture_initial_width/2),0), (int(consts.picture_initial_width/2),consts.picture_initial_height), color=(255, 0, 0), thickness = 1)
    cv2.line(overlay, (int((y+1.0)/2.0*consts.picture_initial_width), 0), (int((y+1.0)/2.0*consts.picture_initial_width),consts.picture_initial_height), color=(0, 255, 0), thickness = 2)
    cv2.line(overlay, (int((dir+1.0)/2.0*consts.picture_initial_width), 0), (int((dir+1.0)/2.0*consts.picture_initial_width),consts.picture_initial_height), color=(0, 0, 255), thickness = 2)
    alpha = 0.30
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, output)
    out.write(output)
    cv2.imshow('frame',output)
    cv2.waitKey(1)
out.release()
cv2.destroyAllWindows()
