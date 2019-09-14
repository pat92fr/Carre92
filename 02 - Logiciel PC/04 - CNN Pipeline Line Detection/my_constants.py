## CONSTANTS ###################################################################

## do not change## do not change## do not change
## do not change## do not change## do not change
## do not change## do not change## do not change

# file names
dataset_filename = "dataset.txt"    
label_filename = "label.txt"
model_filename = "model.h5"    
dataset_preview_filename = "dataset_preview.avi"
dataset_prediction_filename = "dataset_prediction.avi" 
train_valid_dataset_filename = "train_and_valid_dataset.npz"

# directory file names
video_directory = "video"    
model_directory = "model"        
train_valid_dataset_directory = "train_valid_dataset"

# picture
picture_initial_width = 160 
picture_initial_height = 90 
picture_initial_shape = (picture_initial_height, picture_initial_width, 3)
picture_height_crop = 26    ## default 26 pixels (top of picture to crop)
picture_final_height = 64    ##default 64 pixels (part of picture processed by CNN
picture_final_shape = (picture_initial_height, picture_initial_width, 1)

# picture pre-processing
blur_kernel_size = (3,3)         ## from (1,1) to (5,5)

# training
gpu_count = 1
###export TF_MIN_GPU_MULTIPROCESSOR_COUNT=5