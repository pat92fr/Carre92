## PARAMETERS ##################################################################

# inputs/outputs "dataset_002", "dataset_003", "dataset_004",
dataset_dir = [  "dataset_003", "dataset_004", "dataset_007", "dataset_008" ]     ## add training dataset here
testset_dir = [  "dataset_007" ]     ## add test dataset here (en attendant, on remet les training sets)
dataset_filename = "dataset.txt"    ## do not change
label_filename = "label.txt"     ## do not change
video_dir = "video"    ## do not change
dataset_preview_filename = "dataset_preview"    ## do not change
dataset_train_valid_3D_dir = "dataset_train_valid_3D"    ## do not change
dataset_train_valid_2D_dir = "dataset_train_valid_2D"    ## do not change
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
    ('crop2D', picture_height_crop),         ## do not change
    ('norm', 0),                                  ## do not change
    
    ('conv2D', 8, (5,5), (2,2)),           ## filters, kernel_size, stride
    ('conv2D', 8, (5,5), (2,2)),           ## filters, kernel_size, stride
#    ('conv2D', 16, (3,3), (1,1)),           ## filters, kernel_size, stride
    ('maxpooling2D', (2,2), (2,2)),     ## size, stride
    ('dropout',  0.2),

]
full_connected_hidden_layers= [
    (8, 'relu',  0.2),         ## default 256 units in 1srt hidden layer, 10% dropout
#    (8, 'tanh',  0.2),         ## default 256 units in 1srt hidden layer, 10% dropout
#    (16, 'relu',  0.2)       ## default 128 units in 2nd hidden layer, 10% dropout
]      

# hyperparameters
hyp_train_valid_dataset_ratio = 0.05
hyp_batch_size = 256 
hyp_epoch = 60
hyp_lr = 0.0001 
hyp_lr_decay = 0.0
hyp_l2_regularization = 0.0001 
hyp_min_delta=0.0001
hyp_patience=20

## RESULTS ##################################################################

##conv_layers = [
##    ('crop2D', picture_height_crop),         ## do not change
##    ('norm', 0),                                  ## do not change
##    
##    ('conv2D', 8, (5,5), (2,2)),           ## filters, kernel_size, stride
##    ('maxpooling2D', (2,2), (2,2)),     ## size, stride
##    ('dropout',  0.2),
##    
##]
##full_connected_hidden_layers= [
##    (8, 'relu',  0.2),         ## default 256 units in 1srt hidden layer, 10% dropout
###    (16, 'relu',  0.2)       ## default 128 units in 2nd hidden layer, 10% dropout
##]    
##Epoch 20/20
## - 3s - loss: 0.0660 - mean_squared_error: 0.0436 - val_loss: 0.0253 - val_mean_squared_error: 0.0060


# CNN parameters
### see build_3d_cnn https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/keras.py
### Credit: https://github.com/jessecha/DNRacing/blob/master/3D_CNN_Model/model.py
##conv_layers = [
##    ('crop2D', picture_height_crop),         ## do not change
##    ('norm', 0),                                  ## do not change
##    
##    ('conv2D', 8, (5,5), (2,2)),           ## filters, kernel_size, stride
##    ('maxpooling2D', (2,2), (2,2)),     ## size, stride
##    ('dropout',  0.2),
##
##]
##full_connected_hidden_layers= [
##    (8, 'relu',  0.2),         ## default 256 units in 1srt hidden layer, 10% dropout
###    (16, 'relu',  0.2)       ## default 128 units in 2nd hidden layer, 10% dropout
##]  
##Epoch 60/60
## - 8s - val_mean_squared_error: 0.0030


##essais gilles
### CNN parameters
##### see build_3d_cnn https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/keras.py
##### Credit: https://github.com/jessecha/DNRacing/blob/master/3D_CNN_Model/model.py
##conv_layers = [
##    ('crop2D', picture_height_crop),         ## do not change
##    ('norm', 0),                                  ## do not change
##    
##    ('conv2D', 8, (5,5), (2,2)),           ## filters, kernel_size, stride
##    ('conv2D', 8, (3,15), (2,5)),           ## filters, kernel_size, stride
##    ('maxpooling2D', (2,2), (2,2)),     ## size, stride
##    ('dropout',  0.2),
##
##]
##full_connected_hidden_layers= [
##    (8, 'relu',  0.2),         ## default 256 units in 1srt hidden layer, 10% dropout
###    (16, 'relu',  0.2)       ## default 128 units in 2nd hidden layer, 10% dropout
##] 
