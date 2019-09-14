## GLOBALS ###################################################################

import numpy as np
import cv2
import os
import shutil
from sklearn.model_selection import train_test_split
import my_constants as consts
import my_parameters as params
import my_datasettools as dtools

## MAIN ########################################################################

#init
X = []
Y = []

# for each dataset in the list
for dataset_dir in params.train_and_valid_dataset_list:
    print(dataset_dir)

    # load label.txt 
    print("Loading label file...")
    label_file = open(params.base_dataset_directory+dataset_dir+'/'+consts.label_filename, "r")
    content = label_file.read()
    label_file.close()
    print("Done.")

    # parse label.txt 
    print("Parsing label file...")
    lines = content.splitlines()
    m = len(lines)
    print(" m=" + str(m) + " examples")
    print("Done.")

    # for each example (picture,linepos) of current dataset
    print("Building dataset...")
    for l in lines:
        fields = l.split(';')
        filename = fields[0]
        y = float(fields[1])
        # load picture
        x = dtools.load_and_preprocess_picture(params.base_dataset_directory+filename)
        # accumulate
        X.append(x)
        Y.append(y)
        # augmentation & accumulate
        #flip horizontaly
        xf = cv2.flip(x.reshape(consts.picture_initial_height,consts.picture_initial_width),1).reshape(consts.picture_initial_height,consts.picture_initial_width,1)
        X.append(xf)
        Y.append(-y)        
    print("Done.")

# to numpy
X = np.array(X)
Y = np.array(Y)
print(str(X.shape))
print(str(Y.shape))
print("Done.")

# split dataset into train and cross validation sets
print("Spliting whole dataset into train and cross-validation datasets...")
Xtrain,Xvalid,Ytrain,Yvalid = train_test_split(X,Y,test_size=params.hyp_train_valid_dataset_ratio,shuffle=True)
print("Xtrain, size: "+str(len(Xtrain)))
print("Ytrain, size: "+str(len(Ytrain)))
print("Xvalid, size: "+str(len(Xvalid)))
print("Yvalid, size: "+str(len(Yvalid)))
print("Done.")

# mkdir
train_valid_dataset_directory = params.base_dataset_directory+consts.train_valid_dataset_directory
if not os.path.exists(train_valid_dataset_directory):
        os.makedirs(train_valid_dataset_directory)

# backup previous datasets
file = train_valid_dataset_directory + '/' + consts.train_valid_dataset_filename
if os.path.isfile(file):
    shutil.copy(file,file +'.bak')

print("Compressing and writing file to disk...")
###np.savez_compressed(file, xt=Xtrain, yt=Ytrain, xv=Xvalid, yv=Yvalid) ##compression gain too low
np.savez(file, xt=Xtrain, yt=Ytrain, xv=Xvalid, yv=Yvalid) ## quicker
print("Done.")
