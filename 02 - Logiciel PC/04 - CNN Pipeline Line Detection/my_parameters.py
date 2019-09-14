## PARAMETERS ##################################################################

# inputs/outputs
base_dataset_directory = "c:/GitHub/datasets/" # change to your local directory where datasets are decompressed
train_and_valid_dataset_list = [  "dataset_003", "dataset_004", "dataset_007", "dataset_008" ]
test_dataset_list = [  "dataset_007" ]

# CNN parameters
### see build_3d_cnn https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/keras.py
### Credit: https://github.com/jessecha/DNRacing/blob/master/3D_CNN_Model/model.py
conv_layers = [
    ('crop2D', 26),         ## do not change
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
