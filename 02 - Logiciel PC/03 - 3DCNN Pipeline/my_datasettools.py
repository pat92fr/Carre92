import cv2
import numpy as np
import matplotlib.pyplot as plt

import my_constants as consts
import my_parameters as params

## FUNCTIONS ###################################################################

def load_and_preprocess_picture(filename):
    # read picture
    frame = cv2.imread(filename,cv2.IMREAD_COLOR)
    assert(frame.shape == consts.picture_initial_shape)
    # smoothing and gray scale
    frame= cv2.blur(frame,params.blur_kernel)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    assert(frame.shape == (consts.picture_initial_height,consts.picture_initial_width))
    # reshape for conv layers
    frame = frame.reshape(consts.picture_initial_height,consts.picture_initial_width,1)
    # out
    return frame

def load_dataset(dataset_dir,  dataset_filename,  display = False):
    print("Load dataset file...")
    file = open(dataset_dir+"/"+dataset_filename, "r")
    content = file.read()
    file.close()
    print("Done.")
    print("Parse dataset file...")
    lines = content.splitlines()
    m = len(lines)
    print(" m=" + str(m) + " examples")
    X = []
    Y = []
    filtered_direction = 0.0
    direction_beta = 1.0-params.direction_alpha
    filtered_counter = 0
    for l in lines:
        fields = l.split(';')
        filename = fields[0]
        # build example
        x = load_and_preprocess_picture(filename)
        y = 0.0
        filtered_counter += 1
        # EWMA on DIR (with bias correction)
        raw_dir = float(fields[1])/255.0*2.0-1.0
        filtered_direction = params.direction_alpha * raw_dir + direction_beta*filtered_direction
        corrected_direction = filtered_direction / (1.0 - pow(direction_beta, float(filtered_counter)))
        y = corrected_direction
        # append example
        X.append(x)
        Y.append(y)
        # display example
        if display:
            print(str(y))
            plt.clf()
            plt.imshow(x.reshape(consts.picture_initial_height,consts.picture_initial_width), cmap = 'gray' )
            plt.axvline(x=(y+1.0)/2.0*consts.picture_initial_width,linewidth=2, color='g')
            plt.axvline(x=consts.picture_initial_width/2,linewidth=2)
            plt.axhline(y=params.picture_height_crop,linewidth=2, marker="v", linestyle='--')
            plt.axhline(y=consts.picture_initial_height/2,linewidth=2)
            plt.pause(0.0001)        
    print("Done.")
    X = np.array(X, ndmin=4)
    Y = np.array(np.transpose(Y))
    print("X.shape:" + str(X.shape))
    print("Y.shape:" + str(Y.shape))
    assert( X.shape == (m,consts.picture_initial_height, consts.picture_initial_width, 1))
    assert( Y.shape == (m,))
    return X, Y 

