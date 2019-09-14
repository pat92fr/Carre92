## PARAMETER ###################################################################

dataset_list = [ "dataset_007" ] # list the dataset directory names to be visualized
layer_start = 2 # bypass crop and lambda layers
layer_count = 3 # how many layers to visualize (cannot exceed the number of CONV/POOL layers of model)

## GLOBAL ######################################################################

import numpy as np
import cv2
import os
import shutil
from keras import models
from keras.models import load_model
import matplotlib.pyplot as plt

import my_constants as consts
import my_parameters as params
import my_datasettools as dtools

## MAIN ########################################################################

# open model
print("Load model from disk ...")
model = load_model(consts.model_directory+"/"+consts.model_filename)
model.summary()
print("Done.")

# internal activation model
layer_outputs = [layer.output for layer in model.layers[layer_start:layer_start+layer_count]] # Extracts the outputs of the top 4 layers
activation_model = models.Model(inputs=model.input, outputs=layer_outputs) # Creates a model that will return these outputs, given the model input
layer_names = []
# for layer in model.layers[:layer_count]:
#     layer_names.append(layer.name) # Names of the layers, so you can have them as part of your plot

# mkdir
video_directory = params.base_dataset_directory + '/' + consts.video_directory
if not os.path.exists(video_directory):
        os.makedirs(video_directory)

# backup previous video
video_full_path_name = video_directory + '/' + consts.dataset_prediction_filename
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
cv2.namedWindow('activations')

# for each dataset in the list
for dataset_dir in dataset_list:
    print(dataset_dir)

    # load label.txt 
    print("Loading label file...")
    label_file = open(params.base_dataset_directory+dataset_dir+'/'+consts.label_filename, "r")
    content = label_file.read()
    label_file.close()
    print("Done.")

    # parse label.txt 
    print("Parsing label file...")
    lines = content.splitlines()
    m = len(lines)
    print(" m=" + str(m) + " examples")
    print("Done.")

    # for each example (picture,linepos) of current dataset
    print("Making video...")
    frame_counter = 0
    for l in lines:
        fields = l.split(';')
        filename = fields[0]
        y_ref = float(fields[1])
        # load picture
        x = dtools.load_and_preprocess_picture(params.base_dataset_directory+filename)
        # use model
        y_pre = model.predict(x.reshape(1,consts.picture_initial_height,consts.picture_initial_width,1)).item(0)
        # internal activation model
        activations = activation_model.predict(x.reshape(1,consts.picture_initial_height,consts.picture_initial_width,1))
        plot_id = 0
        whole_display_grid = np.zeros((720,160*layer_count),np.uint8)
        for layer_activation in activations: 
            n_features = layer_activation.shape[-1] # Number of features in the feature map
            size_h = layer_activation.shape[1] #The feature map has shape (1, size, size, n_features).
            size_w = layer_activation.shape[2] #The feature map has shape (1, size, size, n_features).
            display_grid = np.zeros((size_h * n_features, size_w),np.uint8)
            for feature in range(n_features): # Tiles each filter into a big vertical grid
                channel_image = layer_activation[0,:, :,feature]
                channel_image -= channel_image.mean() # Post-processes the feature to make it visually palatable
                channel_image /= channel_image.std()
                channel_image *= 64
                channel_image += 128
                channel_image = np.clip(channel_image, 0, 255).astype('uint8')
                display_grid[feature * size_h : (feature + 1) * size_h, 0 : size_w] = channel_image
            frame_activation = cv2.resize(display_grid,160,720)
            cv2.imshow('activations',frame_activation)
            plot_id +=1
            #whole_display_grid[:,] = 
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
        cv2.line(overlay, (int((y_ref+1.0)/2.0*consts.picture_initial_width), 0), (int((y_ref+1.0)/2.0*consts.picture_initial_width),consts.picture_initial_height), color=(0, 255, 0), thickness = 2)
        cv2.line(overlay, (int((y_pre+1.0)/2.0*consts.picture_initial_width), 0), (int((y_pre+1.0)/2.0*consts.picture_initial_width),consts.picture_initial_height), color=(0, 0, 255), thickness = 2)
        cv2.putText(overlay, "{:d}".format(frame_counter) + "/{:d}".format(m),(4, 12), cv2.FONT_HERSHEY_PLAIN,1.0, (100, 100, 255), 1)
        alpha = 0.40
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, output)
        out.write(output)
        cv2.imshow('image',output)
        # delay
        cv2.waitKey(1)
        frame_counter += 1

    out.release()
    print("Done.")

#close
cv2.destroyAllWindows()


