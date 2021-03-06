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
from keras.layers import Conv2D, MaxPooling2D
from keras.optimizers import Adam
from keras.regularizers import l2

# parameters
width = 320 # 1280 x 25%
height = 30 # 720p x 25% x 30pix (lower)
inmage_shape = (height, width, 3)
param_units_hidden_1 = 128
param_units_hidden_2 = 8

# hyperparameters
##Epoch 5000/5000
## - 2s - loss: 0.0498 - mean_squared_error: 0.0340 - val_loss: 0.0249 - val_mean_squared_error: 0.0113
##train 
##mean_squared_error: 0.007
##test 
##mean_squared_error: 0.011
##Epoch 30/30 3 CONV 8 3x3 1x1 + 32x1
## - 37s - loss: 0.1277 - mean_squared_error: 0.1255 - val_loss: 0.0746 - val_mean_squared_error: 0.0725
##train 
##mean_squared_error: 0.069
##test 
##mean_squared_error: 0.073
##Epoch 30/30
## - 31s - loss: 0.1315 - mean_squared_error: 0.1293 - val_loss: 0.0769 - val_mean_squared_error: 0.0747
##train 
##mean_squared_error: 0.070
##test 
##mean_squared_error: 0.075
##Epoch 100/100
## - 30s - loss: 0.1353 - mean_squared_error: 0.1330 - val_loss: 0.0773 - val_mean_squared_error: 0.0750
##train 
##mean_squared_error: 0.073
##test 
##mean_squared_error: 0.075
hyp_epoch = 10 #30
hyp_bs = 128 #128
hyp_lr = 0.0001 #0.001
hyp_lr_decay = 0.0 #0.0
hyp_l2_regularization = 0.0001 #0.001
hyp_weight_dropout_1 = 0.50 #0.50
hyp_weight_dropout_2 = 0.20 #0.20

random.seed(datetime.now())

# load dataset
X = np.transpose(np.loadtxt("Xtrain.txt", dtype=float))
Y = np.transpose(np.loadtxt('Ytrain.txt', dtype=float))
m = X.shape[0]
print(str(m))

# shuffle dataset (mini batch)
X_shuffled, Y_shuffled = shuffle(X, Y)

# split dataset
m_train = math.floor(m*0.80)
print(str(m_train))
#X_train = X[:m_train,:]
#Y_train = Y[:m_train]
#X_test = X[m_train:,:]
#Y_test = Y[m_train:]
X_train = X_shuffled[:m_train,:].reshape(m_train,height,width,3)
Y_train = Y_shuffled[:m_train]
X_test = X_shuffled[m_train:,:].reshape(m-m_train,height,width,3)
Y_test = Y_shuffled[m_train:]

# input picture
width = 320 # 1280 x 25% 
height = 30 # 720p x 25% x 30pixels (lower)

# create model
model = Sequential()
# CONV
model.add(Conv2D(8, kernel_size=(3, 3), strides=(1, 1), activation='relu', input_shape=inmage_shape))
model.add(Conv2D(8, kernel_size=(3, 3), strides=(1, 1), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), activation='relu'))
model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
model.add(Flatten())
model.add(Dropout(hyp_weight_dropout_1))
model.add(Dense(param_units_hidden_1,activity_regularizer=l2(hyp_l2_regularization)))
model.add(Activation("relu"))
model.add(Dropout(hyp_weight_dropout_2))
model.add(Dense(param_units_hidden_2,activity_regularizer=l2(hyp_l2_regularization)))
model.add(Activation("relu"))
model.add(Dropout(hyp_weight_dropout_2))
model.add(Dense(1,activity_regularizer=l2(hyp_l2_regularization))) # 1 output
model.add(Activation("linear"))

#model.add(Conv2D(8, kernel_size=(3, 3), strides=(1, 1), activation='relu', input_shape=inmage_shape))
#model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))
#model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), activation='relu'))
#model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))
#model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), activation='relu'))
#model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))

#model.add(Dropout(hyp_weight_dropout_1))
#model.add(Flatten())

# FCN
#model.add(Dropout(hyp_weight_dropout, input_shape=(X_train.shape[1],)))
#model.add(Dense(param_units_hidden_1,use_bias=False,activity_regularizer=l2(hyp_l2_regularization))) #input_dim=X_train.shape[1],activity_regularizer=l2(hyp_l2_regularization)
#model.add(BatchNormalization())
#model.add(Activation("relu"))
#model.add(Dropout(hyp_weight_dropout))
#model.add(Dense(param_units_hidden_2,activity_regularizer=l2(hyp_l2_regularization)))
#model.add(Dense(param_units_hidden_2,use_bias=False,activity_regularizer=l2(hyp_l2_regularization)))
#model.add(BatchNormalization())
#model.add(Activation("relu"))
#model.add(Dropout(hyp_weight_dropout_2))


model.add(Dropout(hyp_weight_dropout_1, input_shape=(X_train.shape[1],)))
#model.add(Dense(param_units_hidden_1,activity_regularizer=l2(hyp_l2_regularization),input_dim=X_train.shape[1]))
##model.add(Dense(param_units_hidden_1,activity_regularizer=l2(hyp_l2_regularization)))
##model.add(BatchNormalization())
##model.add(Activation("relu"))
##model.add(Dropout(hyp_weight_dropout_2))
##model.add(Dense(param_units_hidden_2,activity_regularizer=l2(hyp_l2_regularization)))
##model.add(BatchNormalization())
##model.add(Activation("relu"))
##model.add(Dropout(hyp_weight_dropout_2))
##model.add(Dense(1,activity_regularizer=l2(hyp_l2_regularization))) # 1 output
##model.add(Activation("linear"))

# summarize model.
model.summary()

# Compile model
opt = Adam(lr=hyp_lr,decay=hyp_lr_decay)
model.compile(loss='mean_squared_error', optimizer=opt, metrics=['mse'])
# Fit the model
history = model.fit(X_train, Y_train, validation_data=(X_test, Y_test), epochs=hyp_epoch, batch_size=hyp_bs, verbose=2)
# evaluate the model
scores = model.evaluate(X_train, Y_train, verbose=0)
print("train \n%s: %.3f" % (model.metrics_names[1], scores[1]))
# evaluate the model
scores = model.evaluate(X_test, Y_test, verbose=0)
print("test \n%s: %.3f" % (model.metrics_names[1], scores[1]))

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
plt.plot(history.history['loss'], label='train')
plt.plot(history.history['val_loss'], label='test')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper right')
plt.show()

# save model and architecture to single file
model.save("model.h5")
print("Saved model to disk")





