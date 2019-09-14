import numpy as np
from sklearn.model_selection import train_test_split

import my_constants as consts
import my_parameters as params
import my_datasettools as dtools

## GLOBALS ###################################################################

picture_sequence_shape = (params.depth, consts.picture_initial_height, consts.picture_initial_width, 1)

## MAIN ########################################################################

#init
X = []
Y = []
for directory in params.dataset_dir:
    print(directory)
    # load label.txt 
    print("Load label file...")
    label_file = open(directory+"/"+params.label_filename, "r")
    content = label_file.read()
    label_file.close()
    print("Done.")
    # parse label.txt 
    print("Parse label file...")
    lines = content.splitlines()
    m = len(lines)
    print(" m=" + str(m) + " examples")
    # display
    frame_counter = 0
    for l in lines:
        fields = l.split(';')
        filename = fields[0]
        y = float(fields[1])
        # load picture
        x = dtools.load_and_preprocess_picture(filename)
        # build dataset
        X.append(x)
        Y.append(y)
X = np.array(X)
Y = np.array(Y)
print(str(X.shape))
print(str(Y.shape))
# split dataset
print("Split squence dataset into train and valid datasets...")
Xtrain,Xvalid,Ytrain,Yvalid = train_test_split(X,Y,test_size=params.hyp_train_valid_dataset_ratio,shuffle=True)
print("Xtrain, size: "+str(len(Xtrain)))
print("Ytrain, size: "+str(len(Ytrain)))
print("Xvalid, size: "+str(len(Xvalid)))
print("Yvalid, size: "+str(len(Yvalid)))
np.save(params.dataset_train_valid_2D_dir + '/Xtrain.bin', Xtrain)
np.save(params.dataset_train_valid_2D_dir + '/Ytrain.bin', Ytrain)
np.save(params.dataset_train_valid_2D_dir + '/Xvalid.bin', Xvalid)
np.save(params.dataset_train_valid_2D_dir + '/Yvalid.bin', Yvalid)

