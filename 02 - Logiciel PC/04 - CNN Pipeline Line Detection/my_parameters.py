## PARAMETERS ##################################################################

# inputs/outputs
base_dataset_directory = "c:/GitHub/datasets/" # change to your local directory where datasets are decompressed
train_and_valid_dataset_list = [  "dataset_003", "dataset_004", "dataset_007", "dataset_008" ]
test_dataset_list = [  "dataset_007" ]

# CNN parameters
layers = [
    ('crop2D', 26), ## do not change
    ('norm', 0), ## do not change
    ('conv2D', 8, (5,5), (2,2),'relu'), ## filters, kernel_size, stride, activation
    ('conv2D', 8, (5,5), (2,2),'relu'), ## filters, kernel_size, stride, activation
    ('maxpooling2D', (2,2), (2,2)),     ## size, stride
    ('dropout',  0.2), ## %
    ('flatten',  0),  ## do not change ## CONV to FC transition
    ('fc', 8, 'tanh', 0.0001), ## units, activation, l2
    ('dropout',  0.2), ## %
    ('fc', 1, 'linear', 0.0001), ## units, activation, l2        
    # do not change unit and activation of the last layer (one output, no activation = linear)
]      
#    (8, 'tanh',  0.2),         ## default 256 units in 1srt hidden layer, 10% dropout

# hyperparameters
hyp_train_valid_dataset_ratio = 0.05
hyp_batch_size = 256 
hyp_epoch = 100
hyp_lr = 0.0001 
hyp_lr_decay = 0.0
hyp_min_delta=0.0002
hyp_patience=12

## RESULTS ##################################################################

###Paul OK 19-09-13, 60 epoch, datasets : 003 004 007 008
# layers = [
#     ('crop2D', 26), ## do not change
#     ('norm', 0), ## do not change
#     ('conv2D', 8, (5,5), (2,2),'relu'), ## filters, kernel_size, stride, activation
#     ('conv2D', 8, (5,5), (2,2),'relu'), ## filters, kernel_size, stride, activation
#     ('maxpooling2D', (2,2), (2,2)),     ## size, stride
#     ('dropout',  0.2), ## %
#     ('flatten',  0),  ## do not change ## CONV to FC transition
#     ('fc', 8, 'relu', 0.0001), ## units, activation, l2
#     ('dropout',  0.2), ## %
#     ('fc', 1, 'linear', 0.0001), ## units, activation, l2         
# ]  

