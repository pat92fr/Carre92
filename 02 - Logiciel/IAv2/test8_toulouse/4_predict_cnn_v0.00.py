import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
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
    # crop
    im_croped_1 = im_resized[height-height_end_1:height-height_start_1,:width]
    assert(im_croped_1.shape == (crop_height,width,3))
    im_croped_2 = im_resized[height-height_end_2:height-height_start_2,:width]
    assert(im_croped_2.shape == (crop_height,width,3))
    # build x
    #x_1 = image2vector(im_croped_1)
    #x_2 = image2vector(im_croped_2)
    x_1 = im_croped_1
    x_2 = im_croped_2
    #assert(x_1.shape == (height*width*3,1))
    #assert(x_2.shape == (height*width*3,1))
    assert(x_1.shape == (crop_height,width,3))
    assert(x_2.shape == (crop_height,width,3))
    #make a prediction
    #y_1 = model.predict(np.transpose(x_1/255.0))
    #y_2 = model.predict(np.transpose(x_2/255.0))
    y_1 = model.predict(x_1.reshape(1,crop_height,width,3)/255.0)
    y_2 = model.predict(x_2.reshape(1,crop_height,width,3)/255.0)
    print(str(y_1))
    print(str(y_2))
    plt.clf()
    plt.imshow(image)
    plt.plot([(y_1[0]+1.0)/2.0*1280, (y_2[0]+1.0)/2.0*1280], [(height-height_start_1-crop_height/2)*8,(height-height_end_2+crop_height/2)*8],linewidth=5)
    #plt.show()
    plt.pause(1)







