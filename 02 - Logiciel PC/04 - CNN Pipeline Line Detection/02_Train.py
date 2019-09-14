## GLOBALS ###################################################################

import numpy as np
import matplotlib.pyplot as plt
import time
import os
import shutil
#from datetime import datetime
import random 

from sklearn.utils import shuffle

from keras.optimizers import Adam
from keras.callbacks import TensorBoard, EarlyStopping, ModelCheckpoint, Callback
#from tensorflow.python.keras.utils import Sequence
from keras.utils import Sequence
from keras.utils import multi_gpu_model

import my_constants as consts
import my_parameters as params
import my_modeltools as mtools

## FUNCTIONS ###################################################################

# updatable plot
class PlotLosses(Callback):
    def on_train_begin(self, logs={}):
        self.i = 0
        self.x = []
        self.losses = []
        self.val_losses = []
        
        self.fig = plt.figure()
        
        self.logs = []

    def on_epoch_end(self, epoch, logs={}):
        
        self.logs.append(logs)
        self.x.append(self.i)
        self.losses.append(logs.get('loss'))
        self.val_losses.append(logs.get('val_loss'))
        self.i += 1
        
        #clear_output(wait=True)
        plt.clf()
        plt.plot(self.x, self.losses, label="loss")
        plt.plot(self.x, self.val_losses, label="val_loss")
        plt.legend()
        plt.pause(0.001)
        #plt.show(block=False);

def augment_picture(batch_x,batch_y):
    #global shadow_pictures_list
    counter = 0
    ## never pass trhu original pictures, train and validate CNN on augmented pictures
    for (x,y) in zip(batch_x,batch_y):
        # # reshaphe for picture processing
        # x = x.reshape(90,160)
        # # random flip orignial picture
        # flip = random.randint(0, 1)
        # if flip == 2:
        #     #flip horizontaly
        #     x = cv2.flip(x,1)
        #     y = -y
        # randow pick a shadow picture and apply to example
##        shadow_picture_filename = random.choice(shadow_pictures_list)
##        shadow_image = cv2.imread(shadow_picture_filename,cv2.IMREAD_COLOR)
##        shadow_image_gray = cv2.cvtColor(shadow_image, cv2.COLOR_RGB2GRAY)
##        alpha = random.uniform(0.6,0.9)
##        beta = 1.0-alpha
##        gamma = random.uniform(-0.4,0.4)
##        x = cv2.addWeighted(x, alpha, shadow_image_gray/255.0, beta, 1.0+gamma)
        # TODO : random motion blur, dust, 
        if False:
            plt.clf()
            plt.imshow( x, cmap = 'gray' )
            plt.axvline(x=(y+1.0)/2.0*160,linewidth=2)
            plt.pause(0.01)
        # debug : save to file
        if False:
            skimage.io.imsave(output_dir+"/batch_x/"+"save_"+str(counter)+".jpeg", x) #skimage.util.img_as_ubyte(x))
            counter += 1
        # reshape for conv2D
        ###x = x.reshape(90,160,1)
    # output
    return batch_x, batch_y

# keras data generator (batch)
class data_generator(Sequence):
    def __init__(self, x_set, y_set, batch_size,valid):
        self.x, self.y = x_set, y_set
        self.batch_size = batch_size
        self.valid = valid

    def __len__(self):
        return int(np.ceil(len(self.x) / float(self.batch_size)))
    
    def __getitem__(self, idx):
        batch_x = self.x[idx * self.batch_size:(idx + 1) * self.batch_size]
        batch_y = self.y[idx * self.batch_size:(idx + 1) * self.batch_size]
        # data augmentation for training set only
        if not self.valid:
            batch_x, batch_y = augment_picture(batch_x,batch_y)
        return np.array(batch_x), np.array(batch_y)

    def on_epoch_end(self):
        'Updates dataset after each epoch'
        self.x,self.y = shuffle(self.x,self.y)

## MAIN ########################################################################

print("Loading and decompressing dataset file from disk...")
file = params.base_dataset_directory + consts.train_valid_dataset_directory + '/' + consts.train_valid_dataset_filename
dataset = np.load(file)
print("Done.")

print("Retreiving train and valid datasets...")
Xtrain = dataset['xt']
Ytrain = dataset['yt']
Xvalid = dataset['xv']
Yvalid = dataset['yv']
dataset.close()
print("Done.")

print("Xtrain, shape: "+str(Xtrain.shape))
print("Ytrain, shape: "+str(Ytrain.shape))
print("Xvalid, shape: "+str(Xvalid.shape))
print("Yvalid, shape: "+str(Yvalid.shape))

# mkdir
if not os.path.exists(consts.model_directory):
        os.makedirs(consts.model_directory)

## generators
training_generator = data_generator(Xtrain,Ytrain,params.hyp_batch_size,False)
validation_generator = data_generator(Xvalid,Yvalid,params.hyp_batch_size,True)

# optimizer
opt = Adam(lr=params.hyp_lr,decay=params.hyp_lr_decay)

# history plot
plot_losses = PlotLosses()

# early stopping
es = EarlyStopping(monitor='val_mean_squared_error', mode='min', min_delta=params.hyp_min_delta, verbose=1, patience=params.hyp_patience)

# tensorboard
tensorboard = TensorBoard(log_dir=consts.model_directory+"/{}".format(time.time()), batch_size=params.hyp_batch_size)

# checkpoint
filepath="weights-improvement-{epoch:04d}-{val_mean_squared_error:.5f}.hdf5"
mc = ModelCheckpoint(consts.model_directory+"/"+filepath, monitor='val_mean_squared_error', mode='min', save_best_only=True) #, verbose=1)

# compile model
model = mtools.build_model(consts.picture_final_shape, params.layers)
print("Training model...")
if consts.gpu_count > 1:
    parallel_model = multi_gpu_model(model, gpus=consts.gpu_count)
    parallel_model.compile(loss='mean_squared_error',optimizer=opt, metrics=['mse'])
    history = parallel_model.fit_generator(
        epochs=params.hyp_epoch,
        generator=training_generator,
        validation_data=validation_generator,
        workers=8,
    #    callbacks=[tensorboard, mc, es, plot_losses],
        callbacks=[es, plot_losses],
        verbose=2
    )
else:
    model.compile(loss='mean_squared_error', optimizer=opt, metrics=['mse'])
    history = model.fit_generator(
        epochs=params.hyp_epoch,
        generator=training_generator,
        validation_data=validation_generator,
        workers=8,
    #    callbacks=[tensorboard, mc, es, plot_losses],
        callbacks=[es, plot_losses],
        verbose=2
    )
print("Done.")

# backup previous model
file = consts.model_directory + '/' + consts.model_filename
if os.path.isfile(file):
    shutil.copy(file,file +'.bak')

## save model and architecture to single file
print("Saving model to disk...")
model.save(consts.model_directory + '/' + consts.model_filename + '_' + time.asctime().replace(' ', '_').replace(':', '-') + '.h5')
model.save(file)
print("Done.")

## list all data in history
####print(history.history.keys())
print("Saving history to disk...")
## plot history
plt.clf()
plt.plot(history.history['mean_squared_error'], label='train')
plt.plot(history.history['val_mean_squared_error'], label='test')
plt.yscale("log")
plt.ylabel('mean_squared_error')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper right')
plt.savefig(consts.model_directory + '/' + 'history' + '_' + time.asctime().replace(' ', '_').replace(':', '-') + '.png')
#plt.show()
#plt.close(fig)    # close the figure
print("Done.")

# evaluate on tests sets : TODO