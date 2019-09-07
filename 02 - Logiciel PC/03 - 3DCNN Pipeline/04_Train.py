import numpy as np
import matplotlib.pyplot as plt
import time
#from datetime import datetime

from keras.optimizers import Adam
from keras.callbacks import TensorBoard, EarlyStopping, ModelCheckpoint
#from tensorflow.python.keras.utils import Sequence

import my_constants as const
import my_parameters as params
import my_modeltools as mtools

## GLOBALS ###################################################################

picture_sequence_shape = (params.depth, const.picture_initial_height, const.picture_initial_width, 1)

## FUNCTIONS ###################################################################

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
print("Load train and valid picture sequence dataset...")
Xtrain = np.load(params.dataset_train_valid_3D_dir + '/Xtrain.bin.npy', )
Ytrain = np.load(params.dataset_train_valid_3D_dir + '/Ytrain.bin.npy')
Xvalid = np.load(params.dataset_train_valid_3D_dir + '/Xvalid.bin.npy')
Yvalid = np.load(params.dataset_train_valid_3D_dir + '/Yvalid.bin.npy')
print("Done.")
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
model.save(params.model_dir+"/"+params.model_filename + '_' + time.asctime().replace(' ', '_').replace(':', '-') + ".h5")
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
plt.savefig(params.model_dir+"/"+"history" + '_' + time.asctime().replace(' ', '_').replace(':', '-') + '.png')
print("Saved graph to disk")
