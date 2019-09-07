import numpy as np
import matplotlib.pyplot as plt
import random
import time
from datetime import datetime

#from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

from keras.optimizers import Adam
from keras.callbacks import TensorBoard, EarlyStopping, ModelCheckpoint
#from tensorflow.python.keras.utils import Sequence

import my_contants as const
import my_parameters as params
import my_datasettools as dtools
import my_modeltools as mtools

## GLOBALS ###################################################################

picture_sequence_shape = (params.depth, const.picture_initial_height, const.picture_initial_width, 1)

## FUNCTIONS ###################################################################

def build_sequence_dataset(X, Y, depth, skip):
    print("Build sequence dataset ...")
    m = len(X)
    print(" m=" + str(m) + " input examples")
    Xsequence = []
    Ysequence = []
    
    for i in range(0, m-depth*skip):
        x = []
        for j in range(0, depth):
            x.append(X[j*skip+i])
        x = np.array(x)
        assert(x.shape == picture_sequence_shape)
        y = Y[i+depth*skip]
        assert(y.shape == (2, ))
        Xsequence.append(x)
        Ysequence.append(y)
    # repeat and flip hor
    for i in range(0, m-depth*skip):
        x = []
        for j in range(0, depth):
            x.append(np.flip(X[j*skip+i],1))
        x = np.array(x)
        assert(x.shape == picture_sequence_shape)
        y = -Y[i+depth*skip]
        assert(y.shape == (2, ))
        Xsequence.append(x)
        Ysequence.append(y)
    print("Done.")
    print("Sequence dataset, size: "+str(len(Xsequence))+"/"+str(len(Ysequence)))
    Xsequence = np.array(Xsequence)
    Ysequence = np.array(Ysequence)
    print("Xsequence:" + str(Xsequence.shape))
    print("Ysequence:" + str(Ysequence.shape))
    assert( Xsequence.shape == (2*(m-depth*skip),params.depth, const.picture_initial_height, const.picture_initial_width, 1))
    assert( Ysequence.shape == (2*(m-depth*skip),2))
    return Xsequence,Ysequence

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
X = np.array([])
Y = np.array([])
for directory in params.dataset_dir:
    print(directory)
    # load dataset file
    Xpictures,Ypictures = dtools.load_dataset(directory,  params.dataset_filename, display = False)
    # build sequence dataset file
    Xsequence,Ysequence = build_sequence_dataset(Xpictures, Ypictures, params.depth, params.skip)
    # concate
    if X.size == 0:
        X = Xsequence
        Y = Ysequence
    else:
        X = np.concatenate((X,Xsequence), axis=0)
        Y = np.concatenate((Y,Ysequence), axis=0)
## build shadow pictures list
#shadow_pictures_list = build_shadow_picture_list()
## split dataset
print("Split squence dataset into train and valid datasets...")
Xtrain,Xvalid,Ytrain,Yvalid = train_test_split(X,Y,test_size=params.hyp_train_valid_dataset_ratio,shuffle=True)
print("Xtrain, size: "+str(len(Xtrain)))
print("Ytrain, size: "+str(len(Ytrain)))
print("Xvalid, size: "+str(len(Xvalid)))
print("Yvalid, size: "+str(len(Yvalid)))
## generators
#training_generator = data_generator(Xtrain,Ytrain,hyp_batch_size)
#validation_generator = data_generator(Xvalid,Yvalid,hyp_batch_size)
# design model
model = mtools.build_model(picture_sequence_shape, params.conv_layers, params.full_connected_hidden_layers, 2, params.hyp_l2_regularization)
# compile model
opt = Adam(lr=params.hyp_lr,decay=params.hyp_lr_decay)
model.compile(loss='mean_squared_error', optimizer=opt, metrics=['mse'])
# tensorboard
tensorboard = TensorBoard(log_dir=params.model_dir+"/{}".format(time.time()), batch_size=params.hyp_batch_size)
# checkpoint
filepath="weights-improvement-{epoch:04d}-{val_mean_squared_error:.5f}.hdf5"
mc = ModelCheckpoint(params.model_dir+"/"+filepath, monitor='val_mean_squared_error', mode='min', save_best_only=True) #, verbose=1)
# early stopping
es = EarlyStopping(monitor='val_mean_squared_error', mode='min', min_delta=params.hyp_min_delta, verbose=1, patience=params.hyp_patience)
## fit the model
history = model.fit(
    x=Xtrain, 
    y=Ytrain, 
    batch_size=params.hyp_batch_size, 
    epochs=params.hyp_epoch,
    validation_data=(Xvalid, Yvalid),
    shuffle=True,
    callbacks=[tensorboard, mc, es],
    verbose=2
)
#history = model.fit_generator(epochs=hyp_epoch,generator=training_generator,validation_data=validation_generator,workers=8,callbacks=[tensorboard, mc, es],verbose=2)
## save model and architecture to single file
model.save(params.model_dir+"/"+params.model_filename + ".h5")
print("Saved model to disk")
## list all data in history
####print(history.history.keys())
## plot history
plt.plot(history.history['mean_squared_error'], label='train')
plt.plot(history.history['val_mean_squared_error'], label='test')
plt.yscale("log")
plt.ylabel('mean_squared_error')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper right')
plt.show()

