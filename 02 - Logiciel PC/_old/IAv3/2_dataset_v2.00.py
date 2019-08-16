import numpy as np
import math
import cv2
import matplotlib.pyplot as plt

def image2vector(image):
    """
    Argument:
    image -- a numpy array of shape (length, height, depth)
    
    Returns:
    v -- a vector of shape (length*height*depth, 1)
    """
    size = image.shape[0] * image.shape[1] #* image.shape[2]
    v = image.reshape(size, 1)
    return v

# input parameters
dataset_dir = "dataset_720"
dataset_filename = "dataSet_infos.txt"

#output parameters
intermediate_size = (160,90) # divided by 8 
frame_size = (160,32) # divided by 8 

# read input text file "picture_fullpathname";"x";"y"\n[...]
text_file = open(dataset_dir+"\\"+dataset_filename, "r")
content = text_file.read()
text_file.close()
print("Dataset file loaded...")

# parse data set
dataset = []
lines = content.splitlines()
for l in lines:
    # parse one example
    example = l.split(';')
    dataset.append(example)
print("Dataset file parsed, m: "+str(len(dataset)))

# init data set
m = len(dataset)* 6 ##data augmentation factor
X = np.zeros((frame_size[0]*frame_size[1]*1,m))
Y = np.zeros((1,m))

# load pictures and build data set (X,Y)
i = 0
for example in dataset:
    # read 1280x720x3 image jpeg
    image = cv2.imread(example[0],cv2.IMREAD_COLOR)
    assert(image.shape == (720,1280,3))
    # resize
    image_resized = cv2.resize(image, intermediate_size, interpolation = cv2.INTER_AREA)
    # trace line
    line_position_middle = float(int(example[2])/100.0)
    line_position_low = float(int(example[1])/100.0)
    #print(str(lile_position_middle) +" "+ str(lile_position_low))
    cv2.line(image_resized,( int(intermediate_size[0]*(1.0+line_position_middle)/2.0), int(intermediate_size[1]*(1.0-0.46))),(int(intermediate_size[0]*(1.0+line_position_low)/2.0),int(intermediate_size[1]*(1.0-0.18))),(255,0,0),5)
    cv2.imshow('Imgage', image_resized) 
    ###cv2.waitKey(20)
    cv2.waitKey(200)
    # cut lower frame 
    lower_frame_image = image_resized[intermediate_size[1]-frame_size[1]:intermediate_size[1],0:frame_size[0]]
    ###print(str(lower_frame_image.shape))
    assert(lower_frame_image.shape == (frame_size[1],frame_size[0],3))
    #cut middle frame
    middle_frame_image = image_resized[intermediate_size[1]-2*frame_size[1]:intermediate_size[1]-frame_size[1],0:frame_size[0]]
    assert(middle_frame_image.shape == (frame_size[1],frame_size[0],3))
    # convert to greyscale
    lower_frame_image_bw = cv2.cvtColor(lower_frame_image, cv2.COLOR_RGB2GRAY)
    middle_frame_image_bw = cv2.cvtColor(middle_frame_image, cv2.COLOR_RGB2GRAY)
    # data augmentation : flip H and V
    lower_frame_image_bw_flipped_ver = cv2.flip(lower_frame_image_bw,0)
    lower_frame_image_bw_flipped_hor = cv2.flip(lower_frame_image_bw,1)
    middle_frame_image_bw_flipped_ver = cv2.flip(middle_frame_image_bw,0)
    middle_frame_image_bw_flipped_hor = cv2.flip(middle_frame_image_bw,1)
    # fill X,Y
    X[:,i] = np.transpose(image2vector(lower_frame_image_bw))/255
    Y[0:i] = line_position_low
    i += 1
    X[:,i] = np.transpose(image2vector(lower_frame_image_bw_flipped_ver))/255
    Y[0:i] = line_position_low
    i += 1    
    X[:,i] = np.transpose(image2vector(lower_frame_image_bw_flipped_hor))/255
    Y[0:i] = -line_position_low
    i += 1    
    X[:,i] = np.transpose(image2vector(middle_frame_image_bw))/255
    Y[0:i] = line_position_middle
    i += 1
    X[:,i] = np.transpose(image2vector(middle_frame_image_bw_flipped_ver))/255
    Y[0:i] = line_position_middle
    i += 1    
    X[:,i] = np.transpose(image2vector(middle_frame_image_bw_flipped_hor))/255
    Y[0:i] = -line_position_middle
    i += 1
    #debug
    ###plt.clf()
    ###x = X[:,i-1]
    ###x = x.reshape( (frame_size[1], frame_size[0]) )
    ###plt.imshow( x, cmap = 'gray' )
    ###plt.axvline(x=(Y[0,i-1]+1.0)/2.0*frame_size[0],linewidth=3)
    ###plt.pause(0.002)

print(str(i)+"/"+str(m))
np.savetxt("X.txt",X,fmt="%f")
np.savetxt("Y.txt",Y,fmt="%f")

    
