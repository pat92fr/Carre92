## PARAMETERS ##################################################################

# inputs/outputs 
dataset_dir = [ "dataset_002", "dataset_003", "dataset_004", "dataset_005", "dataset_006" ]     ## add training dataset here
testset_dir = [ "dataset_006" ]     ## add test dataset here (en attendant, on remet les training sets)
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

# picture sequence for CNN
depth = 3           ## default 6 frames processed by CNN
skip = 5            ## default 3 frames skiped between each pair of frames processed by CNN
