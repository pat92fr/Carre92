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
from keras.utils import plot_model
from IPython.display import SVG
from keras.utils.vis_utils import model_to_dot

# parameters
width = 160 # 1280 / 8
height = 32 #
inmage_shape = (height, width, 3)
param_units_hidden_1 = 32 #32
param_units_hidden_2 = 32 #32

# hyperparameters
hyp_epoch = 100 #300
hyp_bs = 128 #128
hyp_lr = 0.0001 #0.0001
hyp_lr_decay = 0.0 #0.0
hyp_l2_regularization = 0.0001 #0.001
hyp_weight_dropout_1 = 0.50 #0.50
hyp_weight_dropout_2 = 0.20 #0.20

##Epoch 300/300
## - 6s - loss: 0.0096 - mean_squared_error: 0.0054 - val_loss: 0.0047 - val_mean_squared_error: 9.5809e-04
##train 
##mean_squared_error: 0.0007
##test 
##mean_squared_error: 0.0010

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

# create model
model = Sequential()
# CONV
model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), border_mode='same', activation='relu', input_shape=inmage_shape))
model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), border_mode='same', activation='relu'))
model.add(Conv2D(16, kernel_size=(3, 3), strides=(1, 1), border_mode='same', activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), border_mode='same', activation='relu'))
model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), border_mode='same', activation='relu'))
model.add(Conv2D(32, kernel_size=(3, 3), strides=(1, 1), border_mode='same', activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
model.add(Conv2D(64, kernel_size=(3, 3), strides=(1, 1), border_mode='same', activation='relu'))
model.add(Conv2D(64, kernel_size=(3, 3), strides=(1, 1), border_mode='same', activation='relu'))
model.add(Conv2D(64, kernel_size=(3, 3), strides=(1, 1), border_mode='same', activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
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

##model.add(Dropout(hyp_weight_dropout_1, input_shape=(X_train.shape[1],)))
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
print("train \n%s: %.4f" % (model.metrics_names[1], scores[1]))
# evaluate the model
scores = model.evaluate(X_test, Y_test, verbose=0)
print("test \n%s: %.4f" % (model.metrics_names[1], scores[1]))

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

# summarize model.
plot_model(model, to_file="model.png")
SVG(model_to_dot(model).create(prog='dot', format='svg'))

