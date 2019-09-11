import numpy as np
from sklearn.model_selection import train_test_split

import my_constants as consts
import my_parameters as params
import my_datasettools as dtools

## GLOBALS ###################################################################

picture_sequence_shape = (params.depth, consts.picture_initial_height, consts.picture_initial_width, 1)

## FUNCTIONS ###################################################################

def build_sequence_dataset(X, Y, depth, skip):
    print("Build sequence dataset ...")
    m = len(X)
    print(" m=" + str(m) + " input examples")
    Xsequence = []
    Ysequence = []
    
    for i in range(0, m-depth*skip):
        x = []
        for j in range(0, depth):
            x.append(X[j*skip+i])
        x = np.array(x)
        assert(x.shape == picture_sequence_shape)
        y = Y[i+depth*skip]
        Xsequence.append(x)
        Ysequence.append(y)
    # repeat and flip hor
    for i in range(0, m-depth*skip):
        x = []
        for j in range(0, depth):
            x.append(np.flip(X[j*skip+i],1))
        x = np.array(x)
        assert(x.shape == picture_sequence_shape)
        y = -Y[i+depth*skip]
        Xsequence.append(x)
        Ysequence.append(y)
    print("Done.")
    print("Sequence dataset, size: "+str(len(Xsequence))+"/"+str(len(Ysequence)))
    Xsequence = np.array(Xsequence)
    Ysequence = np.array(Ysequence).T
    print("Xsequence:" + str(Xsequence.shape))
    print("Ysequence:" + str(Ysequence.shape))
    assert( Xsequence.shape == (2*(m-depth*skip),params.depth, consts.picture_initial_height, consts.picture_initial_width, 1))
    assert( Ysequence.shape == (2*(m-depth*skip),))
    return Xsequence,Ysequence

## MAIN ########################################################################

#init
X = np.array([])
Y = np.array([])
for directory in params.dataset_dir:
    print(directory)
    # load dataset file
    Xpictures,Ypictures = dtools.load_dataset(directory,  params.dataset_filename, display = False)
    # build sequence dataset file
    Xsequence,Ysequence = build_sequence_dataset(Xpictures, Ypictures, params.depth, params.skip)
    # concate
    if X.size == 0:
        X = Xsequence
        Y = Ysequence
    else:
        X = np.concatenate((X,Xsequence), axis=0)
        Y = np.concatenate((Y,Ysequence), axis=0)
# split dataset
print("Split squence dataset into train and valid datasets...")
Xtrain,Xvalid,Ytrain,Yvalid = train_test_split(X,Y,test_size=params.hyp_train_valid_dataset_ratio,shuffle=True)
print("Xtrain, size: "+str(len(Xtrain)))
print("Ytrain, size: "+str(len(Ytrain)))
print("Xvalid, size: "+str(len(Xvalid)))
print("Yvalid, size: "+str(len(Yvalid)))
np.save(params.dataset_train_valid_3D_dir + '/Xtrain.bin', Xtrain)
np.save(params.dataset_train_valid_3D_dir + '/Ytrain.bin', Ytrain)
np.save(params.dataset_train_valid_3D_dir + '/Xvalid.bin', Xvalid)
np.save(params.dataset_train_valid_3D_dir + '/Yvalid.bin', Yvalid)

