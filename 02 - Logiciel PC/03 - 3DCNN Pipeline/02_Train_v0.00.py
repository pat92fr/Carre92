import numpy as np
import matplotlib.pyplot as plt
import random
import math
import cv2
import os
import time
from datetime import datetime
from PIL import Image
import skimage

from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

from IPython.display import SVG

import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, BatchNormalization, Activation
from keras.layers import Conv2D, MaxPooling2D, AveragePooling2D
from keras.layers import SpatialDropout2D
from keras.optimizers import Adam
from keras.regularizers import l2
from keras.utils import plot_model
from keras.utils.vis_utils import model_to_dot
from keras.callbacks import TensorBoard, EarlyStopping, ModelCheckpoint
from tensorflow.python.keras.utils import Sequence
import h5py

## PARAMETERS ##################################################################

# inputs
dataset_dir = "dataset"
dataset_filename = "dataset.txt"
train_valid_dataset_ratio = 0.05

# outputs
output_dir = "model"

# parameters
param_conv_layers = [
    ('conv', 16, (5,5), (1,1)),
    ('conv', 16, (5,5), (1,1)),
    ('maxpooling', 0, (2,2), (0,0)),
    ('conv', 24, (3,3), (1,1)),
    ('conv', 24, (3,3), (1,1)),
    ('conv', 32, (3,3), (1,1)),
    ('conv', 32, (3,3), (1,1))
] # type, filters, size, stride
param_units_hidden= [32,8,0,0]

# hyperparameters
hyp_batch_size = 128 
hyp_epoch = 1000
hyp_lr = 0.0001 #0.00005
hyp_lr_decay = 0.0
hyp_l2_regularization = 0.0001 
hyp_weight_dropout_1 = 0.50
hyp_weight_dropout_2 = 0.10

## CONSTANTS ###################################################################

# picture
picture_initial_width = 160 
picture_initial_height = 90 
picture_initial_shape = (picture_initial_height, picture_initial_width,3)

picture_final_width = 160 
picture_final_height = 64
picture_final_shape = (picture_final_height, picture_final_width,3)

picture_height_crop = picture_initial_height-picture_final_height

## FUNCTIONS ###################################################################

def load_picture(filename):
    # read track line picture
    image = cv2.imread(filename,cv2.IMREAD_COLOR)
    assert(image.shape == picture_initial_shape)
#    # resize
#    intermediate_size = (160,90) # divided by 8 
#    image_resized = cv2.resize(image, intermediate_size, interpolation = cv2.INTER_AREA)
#    assert(image_resized.shape == (90,160,3))
    # smoothing and gray scale
    image_smoothed= cv2.blur(image,(3,3))
    image_bw = cv2.cvtColor(image_smoothed, cv2.COLOR_RGB2GRAY)
#    assert(image_bw.shape == (90,160))
#    # cut frame
#    frame_size = (32,160)
#    low_frame = image_bw[intermediate_size[1]-frame_size[0]:intermediate_size[1],0:frame_size[1]]
#    middle_frame = image_bw[intermediate_size[1]-2*frame_size[0]:intermediate_size[1]-frame_size[0],0:frame_size[1]]                
#    assert(low_frame.shape == frame_size)
#    assert(middle_frame.shape == frame_size)
#    # normalize
#    low_frame = low_frame/255.0
#    middle_frame = middle_frame/255.0
#    # reshape for conv2D
#    low_frame = low_frame.reshape(32,160,1)
#    middle_frame = middle_frame.reshape(32,160,1)
    # out
    return image_bw

def load_dataset():
    print("Load dataset file...")
    global dataset_dir, dataset_filename
    file = open(dataset_dir+"/"+dataset_filename, "r")
    content = file.read()
    file.close()
    print("Done.")
    print("Parse dataset file...")
    lines = content.splitlines()
    X = []
    Y = []
    filtered_dir = 0.0
    alpha = 0.1
    beta = 1.0-alpha
    filtered_counter = 0
    for l in lines:
        fields = l.split(';')
        filename = fields[0]
        x = load_picture(filename)
        y = []
        # EWMA on DIR (bias correction)
        raw_dir = float(fields[1])/255.0*2.0-1.0
        filtered_dir = alpha * raw_dir + beta*filtered_dir
        filtered_counter += 1
        corrected_dir = filtered_dir / (1.0 - pow(beta, float(filtered_counter)))
        y.append( corrected_dir ) # DIR
        # TODO : EWMA on THR
        y.append( float(fields[2])/255.0*2.0-1.0) # THR
        X.append(x)
        Y.append(y)
        if True:
            print(str(y))
            plt.clf()
            plt.imshow(x, cmap = 'gray' )
            plt.axvline(x=(y[0]+1.0)/2.0*picture_initial_width,linewidth=2, label='DIR')
            plt.axhline(y=picture_height_crop,linewidth=2, label='CROP', marker="v", linestyle='--')
            plt.axhline(y=picture_initial_height/2,linewidth=2)
            plt.pause(0.001)        
    print("Done.")
    print("Dataset, size: "+str(len(X))+"/"+str(len(Y)))
    return X,Y

#def augment_track_line_picture(batch_x,batch_y):
#    global shadow_pictures_list
#    counter = 0
#    ## never pass trhu original pictures, train and validate CNN on augmented pictures
#    for (x,y) in zip(batch_x,batch_y):
#        # reshaphe for picture processing
#        x = x.reshape(32,160)
#        # random flip orignial picture
#        flip = random.randint(0, 3)
#        if flip == 1:
#            #flip vertically
#            x = cv2.flip(x,0)
#        elif flip == 2:
#            #flip horizontaly
#            x = cv2.flip(x,1)
#            y = -y
#        elif flip == 3:
#            # flip both h and v
#            x = cv2.flip(x,-1)
#            y = -y
#        # randow pick a shadow picture and apply to example
#        shadow_picture_filename = random.choice(shadow_pictures_list)
#        shadow_image = cv2.imread(shadow_picture_filename,cv2.IMREAD_COLOR)
#        shadow_image_gray = cv2.cvtColor(shadow_image, cv2.COLOR_RGB2GRAY)
#        alpha = random.uniform(0.6,0.9)
#        beta = 1.0-alpha
#        gamma = random.uniform(-0.4,0.4)
#        x = cv2.addWeighted(x, alpha, shadow_image_gray/255.0, beta, 1.0+gamma)
#        # TODO : random motion blur, dust, 
#        if False:
#            plt.clf()
#            plt.imshow( x, cmap = 'gray' )
#            plt.axvline(x=(y+1.0)/2.0*160,linewidth=2)
#            plt.pause(0.01)
#        # debug : save to file
#        if False:
#            skimage.io.imsave(output_dir+"/batch_x/"+"save_"+str(counter)+".jpeg", x) #skimage.util.img_as_ubyte(x))
#            counter += 1
#        # reshape for conv2D
#        x = x.reshape(32,160,1)
#    # output
#    return batch_x, batch_y

# keras data generator (batch)
#class data_generator(keras.utils.Sequence):
#    def __init__(self, x_set, y_set, batch_size):
#        self.x, self.y = x_set, y_set
#        self.batch_size = batch_size
#
#    def __len__(self):
#        return int(np.ceil(len(self.x) / float(self.batch_size)))
#    
#    def __getitem__(self, idx):
#        batch_x = self.x[idx * self.batch_size:(idx + 1) * self.batch_size]
#        batch_y = self.y[idx * self.batch_size:(idx + 1) * self.batch_size]
#        # data augmentation (original track line picture ==> augmented pictures
#        batch_x, batch_y = augment_track_line_picture(batch_x,batch_y)
#        return np.array(batch_x), np.array(batch_y)
#
#    def on_epoch_end(self):
#        'Updates dataset after each epoch'
#        self.x,self.y = shuffle(self.x,self.y)

## MAIN ########################################################################

#init
random.seed(datetime.now())
# build track line picture dataset
X,Y = load_dataset()
## build shadow pictures list
#shadow_pictures_list = build_shadow_picture_list()
## split dataset
#Xtrain,Xvalid,Ytrain,Yvalid = train_test_split(X,Y,test_size=train_valid_dataset_radio,shuffle=True)
#print("Xtrain, size: "+str(len(Xtrain)))
#print("Ytrain, size: "+str(len(Ytrain)))
#print("Xvalid, size: "+str(len(Xvalid)))
#print("Yvalid, size: "+str(len(Yvalid)))
## generators
#training_generator = data_generator(Xtrain,Ytrain,hyp_batch_size)
#validation_generator = data_generator(Xvalid,Yvalid,hyp_batch_size)
## design model
#model = Sequential()
## design model : CONV Layers
#conv_layer_count = 0
#for c in param_conv_layers:
#    if c[0] == 'conv':
#        if conv_layer_count == 0:
#            model.add(Conv2D(filters=c[1], kernel_size=c[2], strides=c[3], dilation_rate = (1,1), padding='valid', activation='relu', input_shape=picture_shape))
#        else:
#            model.add(Conv2D(filters=c[1], kernel_size=c[2], strides=c[3], dilation_rate = (1,1), padding='valid', activation='relu'))
#    if c[0] == 'maxpooling':
#        model.add(MaxPooling2D(pool_size=c[2]))
#    if c[0] == 'avgpooling':
#        model.add(AveragePooling2D(pool_size=c[2]))
#    conv_layer_count += 1
## design model : CONV -> FC
#model.add(Flatten())
## design model : FCN layers
#for h in param_units_hidden:
#    if h != 0:    
#        model.add(Dropout(hyp_weight_dropout_1))
#        model.add(Dense(h,activity_regularizer=l2(hyp_l2_regularization)))
#        model.add(Activation("relu"))
## design model : last layer
#model.add(Dropout(hyp_weight_dropout_2))
#model.add(Dense(1,activity_regularizer=l2(hyp_l2_regularization)))
#model.add(Activation("linear"))
#print("Done.")
## summarize model.
#model.summary()
## compile model
#opt = Adam(lr=hyp_lr,decay=hyp_lr_decay)
#model.compile(loss='mean_squared_error', optimizer=opt, metrics=['mse'])
## tensorboard
#tensorboard = TensorBoard(log_dir=output_dir+"/{}".format(time.time()), batch_size=hyp_batch_size)
## checkpoint
#filepath="weights-improvement-{epoch:03d}-{val_mean_squared_error:.4f}.hdf5"
#mc = ModelCheckpoint(output_dir+"/"+filepath, monitor='val_mean_squared_error', mode='min', save_best_only=True) #, verbose=1)
## early stopping
#es = EarlyStopping(monitor='val_mean_squared_error', mode='min', min_delta=0.0003, verbose=1, patience=30)
## fit the model
#history = model.fit_generator(epochs=hyp_epoch,generator=training_generator,validation_data=validation_generator,workers=8,callbacks=[tensorboard, mc, es],verbose=2)
## save model and architecture to single file
#model.save(output_dir+"/"+"model.h5")
#print("Saved model to disk")
## list all data in history
####print(history.history.keys())
## plot history
#plt.plot(history.history['mean_squared_error'], label='train')
#plt.plot(history.history['val_mean_squared_error'], label='test')
#plt.yscale("log")
#plt.ylabel('mean_squared_error')
#plt.xlabel('epoch')
#plt.legend(['train', 'test'], loc='upper right')
#plt.show()


