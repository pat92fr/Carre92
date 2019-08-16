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
dataset_dir = "."
dataset_filename = "predictiondata.txt"
train_valid_dataset_radio = 0.05

# outputs
output_dir = "01 - ANN"

# parameters
param_units_hidden= [256,128,0,0]

# hyperparameters
hyp_batch_size = 128 
hyp_epoch = 3000
hyp_lr = 0.00002 #0.0002
hyp_lr_decay = 0.0
hyp_l2_regularization = 0.0001 #0.0001 
hyp_weight_dropout_2 = 0.10

## CONSTANTS ###################################################################

# picture
input_size = 10

## FUNCTIONS ###################################################################

def build_dataset():
    print("Load dataset file...")
    global dataset_dir, dataset_filename
    file = open(dataset_dir+"\\"+dataset_filename, "r")
    content = file.read()
    file.close()
    print("Done.")
    print("Parse track line dataset file...")
    lines = content.splitlines()
    X = []
    Y = []
    for l in lines:
        fields = l.split(';')
        X.append([float(i) for i in fields[0:input_size]])
        Y.append(float(fields[input_size]))
    X = np.array(X)
    Y = np.array(Y)
    # normalize
    X = X/100.0
    Y = Y/100.0
    ###print(str(X[0]))
    ###print(str(Y[0]))
    X,Y = shuffle(X,Y)
    print("Done.")
    print("Dataset, size: "+str(len(X))+"/"+str(len(Y)))
    return X,Y

##def augment_track_line_picture(batch_x,batch_y):
##    global shadow_pictures_list
##    counter = 0
##    ## never pass trhu original pictures, train and validate CNN on augmented pictures
##    for (x,y) in zip(batch_x,batch_y):
##        # reshaphe for picture processing
##        x = x.reshape(32,160)
##        # random flip orignial picture
##        flip = random.randint(0, 3)
##        if flip == 1:
##            #flip vertically
##            x = cv2.flip(x,0)
##        elif flip == 2:
##            #flip horizontaly
##            x = cv2.flip(x,1)
##            y = -y
##        elif flip == 3:
##            # flip both h and v
##            x = cv2.flip(x,-1)
##            y = -y
##        # randow pick a shadow picture and apply to example
##        shadow_picture_filename = random.choice(shadow_pictures_list)
##        shadow_image = cv2.imread(shadow_picture_filename,cv2.IMREAD_COLOR)
##        shadow_image_gray = cv2.cvtColor(shadow_image, cv2.COLOR_RGB2GRAY)
##        alpha = random.uniform(0.6,0.9)
##        beta = 1.0-alpha
##        gamma = random.uniform(-0.4,0.4)
##        x = cv2.addWeighted(x, alpha, shadow_image_gray/255.0, beta, 1.0+gamma)
##        # TODO : random motion blur, dust, 
##        if False:
##            plt.clf()
##            plt.imshow( x, cmap = 'gray' )
##            plt.axvline(x=(y+1.0)/2.0*160,linewidth=2)
##            plt.pause(0.01)
##        # debug : save to file
##        if False:
##            skimage.io.imsave(output_dir+"/batch_x/"+"save_"+str(counter)+".jpeg", x) #skimage.util.img_as_ubyte(x))
##            counter += 1
##        # reshape for conv2D
##        x = x.reshape(32,160,1)
##    # output
##    return batch_x, batch_y
##
### keras data generator (batch)
##class data_generator(keras.utils.Sequence):
##    def __init__(self, x_set, y_set, batch_size):
##        self.x, self.y = x_set, y_set
##        self.batch_size = batch_size
##
##    def __len__(self):
##        return int(np.ceil(len(self.x) / float(self.batch_size)))
##    
##    def __getitem__(self, idx):
##        batch_x = self.x[idx * self.batch_size:(idx + 1) * self.batch_size]
##        batch_y = self.y[idx * self.batch_size:(idx + 1) * self.batch_size]
##        # data augmentation (original track line picture ==> augmented pictures
##        batch_x, batch_y = augment_track_line_picture(batch_x,batch_y)
##        return np.array(batch_x), np.array(batch_y)
##
##    def on_epoch_end(self):
##        'Updates dataset after each epoch'
##        self.x,self.y = shuffle(self.x,self.y)

## MAIN ########################################################################

#init
random.seed(datetime.now())
# build track line picture dataset
X,Y = build_dataset()
# split dataset
Xtrain,Xvalid,Ytrain,Yvalid = train_test_split(X,Y,test_size=train_valid_dataset_radio,shuffle=True)
print("Xtrain, size: "+str(len(Xtrain)))
print("Ytrain, size: "+str(len(Ytrain)))
print("Xvalid, size: "+str(len(Xvalid)))
print("Yvalid, size: "+str(len(Yvalid)))
### generators
##training_generator = data_generator(Xtrain,Ytrain,hyp_batch_size)
##validation_generator = data_generator(Xvalid,Yvalid,hyp_batch_size)
# design model
model = Sequential()
layer_count = 0
for h in param_units_hidden:
    if h != 0:
        if layer_count == 0:
            model.add(Dense(h,activity_regularizer=l2(hyp_l2_regularization),input_dim=input_size))
        else:
            model.add(Dense(h,activity_regularizer=l2(hyp_l2_regularization)))
        ##model.add(BatchNormalization())
        model.add(Activation("relu"))
        model.add(Dropout(hyp_weight_dropout_2))
        layer_count += 1
# design model : last layer
model.add(Dense(1,activity_regularizer=l2(hyp_l2_regularization)))
model.add(Activation("linear"))
print("Done.")
# summarize model.
model.summary()
# compile model
opt = Adam(lr=hyp_lr,decay=hyp_lr_decay)
model.compile(loss='mean_squared_error', optimizer=opt, metrics=['mse'])
# tensorboard
tensorboard = TensorBoard(log_dir=output_dir+"/{}".format(time.time()), batch_size=hyp_batch_size)
# checkpoint
filepath="weights-improvement-{val_mean_squared_error:.5f}-{epoch:04d}.hdf5"
mc = ModelCheckpoint(output_dir+"/"+filepath, monitor='val_mean_squared_error', mode='min', save_best_only=True) #, verbose=1)
# early stopping
es = EarlyStopping(monitor='val_mean_squared_error', mode='min', min_delta=0.000001, verbose=1, patience=100)
# fit the model
history = model.fit(
    x = Xtrain,
    y = Ytrain,
    validation_data = (Xvalid,Yvalid),
    batch_size = hyp_batch_size,
    epochs = hyp_epoch,
    callbacks= [tensorboard, mc, es],
    verbose = 2,
    shuffle=True
    )
# save model and architecture to single file
model.save(output_dir+"/"+"model.h5")
print("Saved model to disk")
# evaluate
score = model.evaluate(x=Xvalid, y=Yvalid, verbose=0)
print('loss mean_squared_error:', score[0])
print('metric mean_squared_error:', score[1])
# list all data in history
###print(history.history.keys())
# plot history
plt.plot(history.history['mean_squared_error'], label='train')
plt.plot(history.history['val_mean_squared_error'], label='test')
plt.yscale("log")
plt.ylabel('mean_squared_error')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper right')
plt.show()
# predictions
Yp = model.predict(Xvalid)
for (y,yp) in zip(Yvalid[0:10],Yp[0:10]):
    print(str(yp[0]) + " " + str(y) + " error:" + str(abs(yp[0]-y)))


