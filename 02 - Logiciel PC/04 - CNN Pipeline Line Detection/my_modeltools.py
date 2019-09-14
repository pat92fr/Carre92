from keras.models import Sequential
from keras.layers import Lambda,  Dense, Flatten, Dropout, BatchNormalization, Activation
from keras.layers import Conv2D, MaxPooling2D, AveragePooling2D,  Cropping2D
from keras.layers import Conv3D, MaxPooling3D, AveragePooling3D,  Cropping3D
from keras.regularizers import l2

## FUNCTIONS ###################################################################

def build_model(input_shape, layers):
    print("Build model ...")
    model = Sequential()
    # design model : CONV Layers
    for c in layers:
        if c[0] == 'crop3D':
            model.add(Cropping3D(cropping=( (0, 0),  (c[1], 0), (0, 0) ), input_shape=input_shape ) )
        if c[0] == 'crop2D':
            model.add(Cropping2D(cropping=( (c[1], 0), (0, 0) ), input_shape=input_shape ) )
        if c[0] == 'norm':
            model.add(Lambda(lambda x: x / 255.0 - 0.5))
        if c[0] == 'conv2D':
                model.add(Conv2D(filters=c[1], kernel_size=c[2], strides=c[3], padding='valid'))
                model.add(BatchNormalization())
                model.add(Activation(c[4]))
        if c[0] == 'conv3D':
                model.add(Conv3D(filters=c[1], kernel_size=c[2], strides=c[3], padding='valid'))
                model.add(BatchNormalization())
                model.add(Activation(c[4]))
        if c[0] == 'maxpooling2D':
            model.add(MaxPooling2D(pool_size=c[1],strides=c[2]))
        if c[0] == 'avgpooling2D':
            model.add(AveragePooling2D(pool_size=c[1],strides=c[2]))
        if c[0] == 'maxpooling3D':
            model.add(MaxPooling3D(pool_size=c[1],strides=c[2]))
        if c[0] == 'avgpooling3D':
            model.add(AveragePooling3D(pool_size=c[1],strides=c[2]))
        if c[0] == 'dropout':
            model.add(Dropout(c[1]))
        if c[0] == 'batchnorm':
            model.add(BatchNormalization())
        if c[0] == 'flatten':
            model.add(Flatten())
        if c[0] == 'fc':
            model.add(Dense(c[1],activity_regularizer=l2(c[3])))
            model.add(BatchNormalization())
            model.add(Activation(c[2]))
        if c[0] == 'fc_wo_bn':
            model.add(Dense(c[1],activity_regularizer=l2(c[3])))
            model.add(Activation(c[2]))
    print("Done.")
    # summarize model.
    model.summary()
    # out
    return model
