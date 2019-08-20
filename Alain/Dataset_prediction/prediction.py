import numpy as np
import random

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.layers import Conv2D, MaxPooling2D, AveragePooling2D
from keras.optimizers import SGD, Adam
from keras.utils import np_utils

SampleSize=10
TestRatio=0.1 # pourcentage de la partie du dataset reservee au test

# Data, valeurs X sur chaque ligne
# Values, 1 valeur de y=f(x) sur chaque ligne

# chargement du dataset
DataSetFileName="predictiondata.txt"
RawData=np.loadtxt(DataSetFileName, dtype=float, delimiter=";")
Data=RawData[:,:SampleSize]
Values=RawData[:,SampleSize]

print("X : {} values".format(Data.shape))
print("Y : {} values".format(Values.shape))


X_train, X_test, Y_train, Y_test = train_test_split(Data, Values, test_size=TestRatio)

model=Sequential()
model.add(Dense(5, input_dim=SampleSize, activation='relu'))
model.add(Dense(5, activation='relu'))
model.add(Dense(1, activation='linear'))
model.summary()

learning_rate = 1e-3
sgd = SGD(lr=0.01)
adam=Adam(lr=learning_rate)
model.compile(loss='mean_squared_error',optimizer=adam)

batch_size = 100
nb_epoch = 10
model.fit(X_train, Y_train,batch_size=batch_size, epochs=nb_epoch,verbose=1)

scores = model.evaluate(X_test, Y_test, verbose=1)
print("%s: %.2f%%" % (model.metrics_names, scores))

