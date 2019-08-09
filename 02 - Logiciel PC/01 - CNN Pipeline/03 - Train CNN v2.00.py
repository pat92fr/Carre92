import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
import random
from datetime import datetime
import h5py
from sklearn.utils import shuffle
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, BatchNormalization, Activation
from keras.layers import Conv2D, MaxPooling2D, AveragePooling2D
from keras.layers import SpatialDropout2D
from keras.optimizers import Adam
from keras.regularizers import l2
from keras.utils import plot_model
from IPython.display import SVG
from keras.utils.vis_utils import model_to_dot

# parameters
width = 160 # 1280 / 8
height = 32 #
inmage_shape = (height, width,1)
param_units_hidden_1 = 512 #128 ***
param_units_hidden_2 = 128 #128
param_units_hidden_3 = 32 #128
param_units_hidden_4 = 8 #128

# hyperparameters
hyp_epoch = 100 #300
hyp_bs = 128 #128
hyp_lr = 0.0001 #0.00005
hyp_lr_decay = 0.0 #0.0
hyp_l2_regularization = 0.0001 #0.0001
hyp_weight_dropout_1 = 0.50 #0.50
hyp_weight_dropout_2 = 0.20 #0.20

random.seed(datetime.now())

# load dataset
X = np.transpose(np.loadtxt("X.txt", dtype=float))
Y = np.transpose(np.loadtxt('Y.txt', dtype=float))
m = X.shape[0]
print(str(m))
X = X.reshape(m,height,width,1)
# shuffle dataset (mini batch)
X_shuffled, Y_shuffled = shuffle(X, Y)

# create model
model = Sequential()

# CONV 100epochs 0.0025
#32x160
model.add(Conv2D(24, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Conv2D(4, kernel_size=(1, 1), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))

# CONV 100epochs 0.0025
#32x160
##model.add(Conv2D(24, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 2)))
##model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 2)))
##model.add(Conv2D(4, kernel_size=(1, 1), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))

# CONV 100epochs 0.0029
#32x160
##model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 2)))
##model.add(Conv2D(64, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(4, 4)))

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

# CONV 100epochs 
###32x160
##model.add(Conv2D(24, kernel_size=(5, 5), strides=(2, 2), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 2)))
###14x78
##model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))


# CONV 100epochs 30 40
###32x160
##model.add(Conv2D(24, kernel_size=(5, 5), strides=(2, 2), dilation_rate = (1,1), border_mode='valid', activation='relu', input_shape=inmage_shape))
###14x78
##model.add(Conv2D(36, kernel_size=(5, 5), strides=(2, 2), dilation_rate = (1,1), border_mode='valid', activation='relu'))
###5x37
##model.add(Conv2D(64, kernel_size=(3,3), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu'))
###3x35
##model.add(Conv2D(64, kernel_size=(3,3), strides=(1, 1), dilation_rate = (1,1), border_mode='valid', activation='relu'))
###1x33

# CONV 100epochs 20 26
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 2)))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))
##model.add(Conv2D(1, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(1, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))

# CONV 100epochs 18 25
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))
##model.add(Conv2D(1, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))

# CONV 100epochs 10 26
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))
##model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))

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


# CONV 30epochs 22 22
##model.add(Conv2D(8, kernel_size=(3, 3), strides=(1, 2), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(8, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(16, kernel_size=(5, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 2)))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 2)))

# CONV 30epochs 31 37
##model.add(Conv2D(8, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(8, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
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

# CONV 30epochs 16 30
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 1)))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(AveragePooling2D(pool_size=(2, 2)))

# CONV 24 30
##model.add(Conv2D(8, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(Conv2D(8, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))

# CONV 31 39
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (2,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 2)))

# CONV 29 / 40
##model.add(Conv2D(8, kernel_size=(7, 7), strides=(1, 1), dilation_rate = (2,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(7, 7), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(8, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 1)))

# CONV 32 36 / 40 43
##model.add(Conv2D(8, kernel_size=(7, 7), strides=(1, 1), dilation_rate = (2,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(7, 7), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(4, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 1)))

# CONV 32 36
##model.add(Conv2D(8, kernel_size=(7, 7), strides=(1, 1), dilation_rate = (2,1), border_mode='same', activation='relu', input_shape=inmage_shape))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(16, kernel_size=(7, 7), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 1)))
##model.add(Conv2D(4, kernel_size=(5, 5), strides=(1, 1), dilation_rate = (1,1), border_mode='same', activation='relu'))
##model.add(MaxPooling2D(pool_size=(2, 1)))

# CONV -> FC
model.add(Flatten())

# FCN
model.add(Dropout(hyp_weight_dropout_1))
model.add(Dense(param_units_hidden_1,activity_regularizer=l2(hyp_l2_regularization)))
model.add(Activation("relu"))

model.add(Dropout(hyp_weight_dropout_2))
model.add(Dense(param_units_hidden_2,activity_regularizer=l2(hyp_l2_regularization)))
model.add(Activation("relu"))

model.add(Dropout(hyp_weight_dropout_2))
model.add(Dense(param_units_hidden_3,activity_regularizer=l2(hyp_l2_regularization)))
model.add(Activation("relu"))

model.add(Dropout(hyp_weight_dropout_2))
model.add(Dense(param_units_hidden_4,activity_regularizer=l2(hyp_l2_regularization)))
model.add(Activation("relu"))

model.add(Dropout(hyp_weight_dropout_2))
model.add(Dense(1,activity_regularizer=l2(hyp_l2_regularization))) # 1 output
model.add(Activation("linear"))

# summarize model.
model.summary()

# Compile model
opt = Adam(lr=hyp_lr,decay=hyp_lr_decay)
model.compile(loss='mean_squared_error', optimizer=opt, metrics=['mse'])
# Fit the model
history = model.fit(X_shuffled, Y_shuffled, shuffle=True,validation_split=0.1, epochs=hyp_epoch, batch_size=hyp_bs, verbose=2)
# evaluate the model
scores = model.evaluate(X_shuffled, Y_shuffled, verbose=0)
print("global \n%s: %.4f" % (model.metrics_names[1], scores[1]))

# list all data in history
print(history.history.keys())
# plot history
#plt.semilogy(history.history['acc'], label='train')
#plt.semilogy(history.history['val_acc'], label='test')
plt.plot(history.history['mean_squared_error'], label='train')
plt.plot(history.history['val_mean_squared_error'], label='test')
plt.ylabel('mean_squared_error')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper right')
plt.show()
##plt.plot(history.history['loss'], label='train')
##plt.plot(history.history['val_loss'], label='test')
##plt.ylabel('loss')
##plt.xlabel('epoch')
##plt.legend(['train', 'test'], loc='upper right')
##plt.show()

# save model and architecture to single file
model.save("model.h5")
print("Saved model to disk")

# summarize model.
plot_model(model, to_file="model.png")
SVG(model_to_dot(model).create(prog='dot', format='svg'))
model_json = model.to_json()
with open("model.json", "w") as json_file:
    json_file.write(model_json)
