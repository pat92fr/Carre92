import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
from keras import models
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Dropout
from keras.optimizers import Adam
from keras.models import load_model
import random
from datetime import datetime
import h5py
from sklearn.utils import shuffle
from keras.regularizers import l2
import os

def image2vector(image):
    """
    Argument:
    image -- a numpy array of shape (length, height, depth)
    
    Returns:
    v -- a vector of shape (length*height*depth, 1)
    """
    size = image.shape[0] * image.shape[1] * image.shape[2]
    v = image.reshape(size, 1)
    return v

# parameters

# load model
model = load_model('model.h5')
print("Loaded model from disk")
# summarize model.
model.summary()
# internal vizualisation
layer_count = 3
layer_outputs = [layer.output for layer in model.layers[:layer_count]] # Extracts the outputs of the top 4 layers
activation_model = models.Model(inputs=model.input, outputs=layer_outputs) # Creates a model that will return these outputs, given the model input
layer_names = []
for layer in model.layers[:layer_count]:
    layer_names.append(layer.name) # Names of the layers, so you can have them as part of your plot
images_per_row = 1

# input picture
width = 160 # 1280 / 8
height = 90 # 720 / 8
crop_height = 32 # power of 2
height_start_1 = 10 # low offset from bottom
height_end_1 = height_start_1+crop_height # high offset from bottom
height_start_2 = 42
height_end_2 = height_start_2+crop_height

# load images
list = os.listdir('images_test/')
for i in list:
    print("images_test/"+i)
    image = cv2.imread("images_test/"+i,cv2.IMREAD_COLOR)
    assert(image.shape == (720,1280,3))
    # resize
    dim = (width,height)
    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    assert(im_resized.shape == (height,width,3))
    # to gray
    image_gray = cv2.cvtColor(im_resized, cv2.COLOR_RGB2GRAY)
    # crop
    im_croped_1 = image_gray[height-height_end_1:height-height_start_1,:width]
    assert(im_croped_1.shape == (crop_height,width))
    im_croped_2 = image_gray[height-height_end_2:height-height_start_2,:width]
    assert(im_croped_2.shape == (crop_height,width))
    # build x
    #x_1 = image2vector(im_croped_1)
    #x_2 = image2vector(im_croped_2)
    x_1 = im_croped_1
    x_2 = im_croped_2
    #assert(x_1.shape == (height*width*3,1))
    #assert(x_2.shape == (height*width*3,1))
    assert(x_1.shape == (crop_height,width))
    assert(x_2.shape == (crop_height,width))
    #make a prediction
    #y_1 = model.predict(np.transpose(x_1/255.0))
    #y_2 = model.predict(np.transpose(x_2/255.0))
    y_1 = model.predict(x_1.reshape(1,crop_height,width,1)/255.0)
    y_2 = model.predict(x_2.reshape(1,crop_height,width,1)/255.0)
    print(str(y_1))
    print(str(y_2))
    # internal vizualisation
    activations = activation_model.predict(x_1.reshape(1,crop_height,width,1)/255.0)     # Returns a list of five Numpy arrays: one array per layer activation
    for layer_name, layer_activation in zip(layer_names, activations): # Displays the feature maps
        n_features = layer_activation.shape[-1] # Number of features in the feature map
        size_h = layer_activation.shape[1] #The feature map has shape (1, size, size, n_features).
        size_w = layer_activation.shape[2] #The feature map has shape (1, size, size, n_features).
        #print(str(layer_activation.shape[0]))
        #print(str(layer_activation.shape[1]))
        #print(str(layer_activation.shape[2]))
        n_cols = n_features // images_per_row # Tiles the activation channels in this matrix
        display_grid = np.zeros((size_h * n_cols, images_per_row * size_w))
        for col in range(n_cols): # Tiles each filter into a big horizontal grid
            for row in range(images_per_row):
                channel_image = layer_activation[0,
                                             :, :,
                                             col * images_per_row + row]
                channel_image -= channel_image.mean() # Post-processes the feature to make it visually palatable
                channel_image /= channel_image.std()
                channel_image *= 64
                channel_image += 128
                channel_image = np.clip(channel_image, 0, 255).astype('uint8')
                display_grid[col * size_h : (col + 1) * size_h, # Displays the grid
                         row * size_w : (row + 1) * size_w] = channel_image
        scale = 1. / size_h
        plt.clf()
        plt.figure(figsize=(scale * display_grid.shape[1],
                        scale * display_grid.shape[0]))
        plt.title(layer_name)
        plt.grid(False)
        plt.imshow(display_grid, aspect='auto', cmap='viridis')
        plt.savefig('predict/'+i+'_'+layer_name+'.png')
        #plt.show()
        #plt.pause(1)
        plt.close('all')

    
    plt.clf()
    plt.imshow(image)
    plt.plot([(y_1[0]+1.0)/2.0*1280, (y_2[0]+1.0)/2.0*1280], [(height-height_start_1-crop_height/2)*8,(height-height_end_2+crop_height/2)*8],linewidth=5)
    plt.savefig('predict/'+i+'_overlay.png')
    #plt.show()
    plt.pause(1)
    #plt.close('all')







