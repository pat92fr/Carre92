from keras.models import Sequential
from keras.layers import Lambda,  Dense, Flatten, Dropout, BatchNormalization, Activation
from keras.layers import Conv2D, MaxPooling2D, AveragePooling2D,  Cropping2D
from keras.layers import Conv3D, MaxPooling3D, AveragePooling3D,  Cropping3D
from keras.regularizers import l2

## FUNCTIONS ###################################################################

def build_model(input_shape, conv_layers, full_connected_hidden_layers, output_shape, l2_regularization):
    print("Build model ...")
    model = Sequential()
    # design model : CONV Layers
    conv_layer_count = 0
    for c in conv_layers:
        if c[0] == 'crop3D':
            model.add(Cropping3D(cropping=( (0, 0),  (c[1], 0), (0, 0) ), input_shape=input_shape ) )
        if c[0] == 'crop2D':
            model.add(Cropping2D(cropping=( (c[1], 0), (0, 0) ), input_shape=input_shape ) )
        if c[0] == 'norm':
            model.add(Lambda(lambda x: x / 255.0 - 0.5))
        if c[0] == 'conv2D':
            if conv_layer_count == 0:
                model.add(Conv2D(filters=c[1], kernel_size=c[2], strides=c[3], dilation_rate = (1, 1), padding='valid', activation='relu', input_shape=input_shape))
            else:
                model.add(Conv2D(filters=c[1], kernel_size=c[2], strides=c[3], dilation_rate = (1, 1), padding='valid', activation='relu'))
        if c[0] == 'conv3D':
            if conv_layer_count == 0:
                model.add(Conv3D(filters=c[1], kernel_size=c[2], strides=c[3], dilation_rate = (1, 1, 1), padding='valid', activation='relu', input_shape=input_shape))
            else:
                model.add(Conv3D(filters=c[1], kernel_size=c[2], strides=c[3], dilation_rate = (1, 1, 1), padding='valid', activation='relu'))
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
        conv_layer_count += 1
    # design model : CONV -> FC
    model.add(Flatten())
    # design model : FCN layers
    for h in full_connected_hidden_layers:    
        model.add(Dense(h[0],activity_regularizer=l2(l2_regularization)))
        model.add(BatchNormalization())
        model.add(Activation(h[1]))
        model.add(Dropout(h[2]))
    # design model : last layer with 2 outputs
    model.add(Dense(output_shape,activity_regularizer=l2(l2_regularization)))
    model.add(Activation("linear"))
    print("Done.")
    # summarize model.
    model.summary()
    # out
    return model
