import numpy as np
import matplotlib.pyplot as plt
import random
import math
import cv2
from datetime import datetime

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
from keras.callbacks import TensorBoard
import h5py

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
param_units_hidden= [64,64,64,0]
#param_units_hidden= [64,64,64,0] #mean_squared_error: 0.0008 @ 300

# hyperparameters
hyp_epoch = 30 #300
hyp_bs = 128 #128
hyp_lr = 0.0001 #0.00005
hyp_lr_decay = 0.0 #0.0
hyp_l2_regularization = 0.0001 #0.0001
hyp_weight_dropout_1 = 0.50 #0.50
hyp_weight_dropout_2 = 0.20 #0.20

# constants
width = 160 
height = 32 
inmage_shape = (height, width,1)

#init
random.seed(datetime.now())

# load dataset
print("Loading dataset...")
X = np.transpose(np.loadtxt("03 - Dataset/X.txt", dtype=float))
Y = np.transpose(np.loadtxt('03 - Dataset/Y.txt', dtype=float))
m = X.shape[0]
print(str(m) + " examples loaded.")
print("Reshaping & Shuffling...")
X = X.reshape(m,height,width,1)
X_shuffled, Y_shuffled = shuffle(X, Y)
print("Done.")

print("Building model...")
# create model
model = Sequential()

# CONV Layers
conv_layer_count = 0
for c in param_conv_layers:
    if c[0] == 'conv':
        if conv_layer_count == 0:
            model.add(Conv2D(filters=c[1], kernel_size=c[2], strides=c[3], dilation_rate = (1,1), padding='valid', activation='relu', input_shape=inmage_shape))
        else:
            model.add(Conv2D(filters=c[1], kernel_size=c[2], strides=c[3], dilation_rate = (1,1), padding='valid', activation='relu'))
    if c[0] == 'maxpooling':
        model.add(MaxPooling2D(pool_size=c[2]))
    if c[0] == 'avgpooling':
        model.add(AveragePooling2D(pool_size=c[2]))
    conv_layer_count += 1
    
# CONV 100epochs 0.0008
#32x160
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(2, 2), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(24, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu'))
##model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu'))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu'))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu'))

# CONV 100epochs 0.0007
#32x160
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu'))
##model.add(MaxPooling2D(pool_size=(4, 4)))

# CONV 100epochs 0.0007
###32x160
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu'))
##model.add(MaxPooling2D(pool_size=(4, 4)))

# CONV 30epochs 16 18
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 2)))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 2)))

# CONV 30epochs 18 19
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 2)))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 2)))

# CONV -> FC
model.add(Flatten())

# FCN layers
for h in param_units_hidden:
    if h != 0:    
        model.add(Dropout(hyp_weight_dropout_1))
        model.add(Dense(h,activity_regularizer=l2(hyp_l2_regularization)))
        model.add(Activation("relu"))

# last layer (1x output) : LINEAR
model.add(Dropout(hyp_weight_dropout_2))
model.add(Dense(1,activity_regularizer=l2(hyp_l2_regularization)))
model.add(Activation("linear"))
print("Done.")

# summarize model.
model.summary()

# Compile model
opt = Adam(lr=hyp_lr,decay=hyp_lr_decay)
model.compile(loss='mean_squared_error', optimizer=opt, metrics=['mse'])
# Tensorboard
tensorboard = TensorBoard(log_dir='./04 - Tensorboard Logs/{}'.format(time.time()), batch_size=hyp_bs)
# Fit the model
history = model.fit(X_shuffled, Y_shuffled, shuffle=True,validation_split=0.1, epochs=hyp_epoch, batch_size=hyp_bs, verbose=2,callbacks=[tensorboard])
# evaluate the model
scores = model.evaluate(X_shuffled, Y_shuffled, verbose=0)
print("global \n%s: %.4f" % (model.metrics_names[1], scores[1]))

# save model and architecture to single file
model.save("04 - CNN\\model.h5")
print("Saved model to disk")

# list all data in history
###print(history.history.keys())

# plot history
#plt.semilogy(history.history['acc'], label='train')
#plt.semilogy(history.history['val_acc'], label='test')
plt.plot(history.history['mean_squared_error'], label='train')
plt.plot(history.history['val_mean_squared_error'], label='test')
plt.ylabel('mean_squared_error')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper right')
plt.show()

# summarize model.
plot_model(model, to_file="04 - CNN\\model.png")
SVG(model_to_dot(model).create(prog='dot', format='svg'))
model_json = model.to_json()
with open("04 - CNN\\model.json", "w") as json_file:
    json_file.write(model_json)
