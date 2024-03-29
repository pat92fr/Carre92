## PARAMETERS ##################################################################

# inputs/outputs "dataset_002", "dataset_003", "dataset_004",
dataset_dir = [  "dataset_005" ]     ## add training dataset here
testset_dir = [  "dataset_006" ]     ## add test dataset here (en attendant, on remet les training sets)
dataset_filename = "dataset.txt"    ## do not change
video_dir = "video"    ## do not change
dataset_preview_filename = "dataset_preview"    ## do not change
dataset_train_valid_3D_dir = "dataset_train_valid_3D"    ## do not change
model_dir = "model"    ## do not change
model_filename = "model"    ## do not change
dataset_prediction_filename = "dataset_prediction"    ## do not change

# picture
picture_height_crop = 26    ## default 26 pixels (top of picture to crop)
picture_final_height = 64    ##default 64 pixels (part of picture processed by CNN

# picture pre-processing
blur_kernel = (3,3)         ## from (1,1) to (5,5)

# direction and throttle filter
direction_alpha = 0.1     ## default 0.01

# picture sequence for CNN
depth = 3           ## default 6 frames processed by CNN
skip = 5            ## default 3 frames skiped between each pair of frames processed by CNN

# CNN parameters
### see build_3d_cnn https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/keras.py
### Credit: https://github.com/jessecha/DNRacing/blob/master/3D_CNN_Model/model.py
conv_layers = [
    ('crop3D', picture_height_crop),         ## do not change
    ('norm', 0),                                  ## do not change
    
    ('conv3D', 8, (3,5,5), (1,2,2)),           ## filters, kernel_size, stride
    ('maxpooling3D', (1,2,2), (1,2,2)),     ## size, stride
    ('dropout',  0.2),
    
    ('conv3D', 12, (1,3,3), (1,1,1)),
    ('maxpooling3D', (1,2,2), (1,2,2)),
    ('dropout',  0.2),

    ('conv3D', 16, (1,3,3), (1,1,1)),
    ('maxpooling3D', (1,2,2), (1,2,2)),
    ('dropout',  0.2),

   
]
full_connected_hidden_layers= [
    (32, 'relu',  0.2),         ## default 256 units in 1srt hidden layer, 10% dropout
    (16, 'relu',  0.2)       ## default 128 units in 2nd hidden layer, 10% dropout
]      

# hyperparameters
hyp_train_valid_dataset_ratio = 0.05
hyp_batch_size = 128 
hyp_epoch = 30
hyp_lr = 0.0001 
hyp_lr_decay = 0.0
hyp_l2_regularization = 0.0001 
hyp_min_delta=0.0001
hyp_patience=20

## RESULTS ##################################################################

# parameters for dataset 004 + 003 figure5
##depth = 6           ## default 6 frames processed by CNN
##skip = 3            ## default 3 frames skiped between each pair of frames processed by CNN
### see build_3d_cnn https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/keras.py
### Credit: https://github.com/jessecha/DNRacing/blob/master/3D_CNN_Model/model.py
##conv_layers = [
##    ('crop3D', picture_height_crop),         ## do not change
##    ('norm', 0),                                  ## do not change
##    
##    ('conv3D', 16, (3,5,5), (1,3,3)),           ## filters, kernel_size, stride
##    ('maxpooling3D', (1,2,2), (1,2,2)),     ## size, stride
##    
##    ('conv3D', 32, (3,3,3), (1,1,1)),
##    ('maxpooling3D', (1,2,2), (1,2,2)),
##
##    ##('conv3D', 64, (3,3,3), (1,1,1)),
##    ##('maxpooling3D', (1,2,2), (1,2,2)),
##
##    ##('conv3D', 128, (3,3,3), (1,1,1)),
##    ##('maxpooling3D', (1,2,2), (1,2,2))
##    
##    ##('dropout',  0.1),
##    ##('batchnorm', 0),
##]
##full_connected_hidden_layers= [
##    (256, 'relu',  0.1),         ## default 256 units in 1srt hidden layer, 10% dropout
##    (128, 'relu',  0.1)       ## default 128 units in 2nd hidden layer, 10% dropout
##]
##Epoch 82/100
## - 54s - loss: 0.0011 - mean_squared_error: 4.4452e-04 - val_loss: 7.8541e-04 - val_mean_squared_error: 1.5991e-04
##Epoch 83/100
## - 54s - loss: 0.0011 - mean_squared_error: 4.4821e-04 - val_loss: 7.7953e-04 - val_mean_squared_error: 1.6096e-04
##Epoch 00083: early stopping



# parameters for dataset 004 + 004 figure history_Tue_Sep_10_02-10-28_2019.png
##batchnorm
##depth = 6           ## default 6 frames processed by CNN
##skip = 3            ## default 3 frames skiped between each pair of frames processed by CNN
# CNN parameters
### see build_3d_cnn https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/keras.py
### Credit: https://github.com/jessecha/DNRacing/blob/master/3D_CNN_Model/model.py
##conv_layers = [
##    ('crop3D', picture_height_crop),         ## do not change
##    ('norm', 0),                                  ## do not change
##    
##    ('conv3D', 8, (3,5,5), (1,2,2)),           ## filters, kernel_size, stride
##    ('maxpooling3D', (1,2,2), (1,2,2)),     ## size, stride
##    ('dropout',  0.2),
##    
##    ('conv3D', 12, (3,3,3), (1,1,1)),
##    ('maxpooling3D', (1,2,2), (1,2,2)),
##    ('dropout',  0.2),
##
##    ('conv3D', 16, (2,3,3), (1,1,1)),
##    ('maxpooling3D', (1,2,2), (1,2,2)),
##    ('dropout',  0.2),
##
##   
##]
##full_connected_hidden_layers= [
##    (64, 'relu',  0.2),         ## default 256 units in 1srt hidden layer, 10% dropout
##    (32, 'relu',  0.2)       ## default 128 units in 2nd hidden layer, 10% dropout
##]  
##Epoch 106/120
## - 91s - loss: 0.0039 - mean_squared_error: 0.0028 - val_loss: 0.0154 - val_mean_squared_error: 0.0149
##Epoch 107/120
## - 91s - loss: 0.0039 - mean_squared_error: 0.0028 - val_loss: 0.0020 - val_mean_squared_error: 9.6055e-04
##



##
### picture sequence for CNN
##depth = 3           ## default 6 frames processed by CNN
##skip = 5            ## default 3 frames skiped between each pair of frames processed by CNN
##
### CNN parameters
##### see build_3d_cnn https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/keras.py
##### Credit: https://github.com/jessecha/DNRacing/blob/master/3D_CNN_Model/model.py
##conv_layers = [
##    ('crop3D', picture_height_crop),         ## do not change
##    ('norm', 0),                                  ## do not change
##    
##    ('conv3D', 8, (3,5,5), (1,2,2)),           ## filters, kernel_size, stride
##    ('maxpooling3D', (1,2,2), (1,2,2)),     ## size, stride
##    ('dropout',  0.2),
##    
##    ('conv3D', 12, (1,3,3), (1,1,1)),
##    ('maxpooling3D', (1,2,2), (1,2,2)),
##    ('dropout',  0.2),
##
##    ('conv3D', 16, (1,3,3), (1,1,1)),
##    ('maxpooling3D', (1,2,2), (1,2,2)),
##    ('dropout',  0.2),
##
##   
##]
##full_connected_hidden_layers= [
##    (32, 'relu',  0.2),         ## default 256 units in 1srt hidden layer, 10% dropout
##    (16, 'relu',  0.2)       ## default 128 units in 2nd hidden layer, 10% dropout
##]   
##Epoch 30/30
## - 37s - loss: 0.0050 - mean_squared_error: 0.0042 - val_loss: 0.0033 - val_mean_squared_error: 0.0028
 
