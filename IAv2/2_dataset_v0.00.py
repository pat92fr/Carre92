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
    size = image.shape[0] * image.shape[1] * image.shape[2]
    v = image.reshape(size, 1)
    return v

# parameters

# read input text file "picture";"x";"y";[...]
text_file = open("result.txt", "r")
content = text_file.read()
text_file.close()

# split fields into list
lines = content.split(';')
#print(lines)

# input picture
width = 320 # 1280 x 25% 
height_start = 30 # 720p x 25% x 30pix (lower)
height_end = 60 # 720p x 25% x 30pix (lower)
height = 30 # 720p x 25% x 30pix (lower)

# build stats

# build data set
m = math.floor(len(lines)/3)
print("number of examples:" + str(m))
X_train = np.zeros((width*height*3,m*3))
Y_train = np.zeros((1,m*3))
for i in range(0,m):
    fullpathname = lines[i*3].split('.')[0]+".jpg"
    print(fullpathname)
    print(lines[i*3+1])
    # read 1280x720x3 image jpeg
    image = cv2.imread(fullpathname,cv2.IMREAD_COLOR)
    assert(image.shape == (720,1280,3))
    # resize (25%)
    dim = (width,180)
    im_resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    assert(im_resized.shape == (180,width,3))
    #cv2.imshow('Img', im_resized) 
    #cv2.waitKey(0)
    # crop to heigth start-end
    im_croped = im_resized[180-height_end:180-height_start,:width]
    assert(im_croped.shape == (height,width,3))
    #cv2.imshow('Img', im_croped) 
    #cv2.waitKey(0)
    #
    im_flipped_ver = cv2.flip(im_croped,0)
    im_flipped_hor = cv2.flip(im_croped,1)
    # list of images
    images = [im_croped, im_flipped_ver, im_flipped_hor]
    # read position
    position = int( lines[i*3+1] ) #0..1280
    if position == -1:
        y = 0 # middle
    else:
        #scale Y to -1:+1
        y = float(position) / 1280.0 * 2.0 - 1.0
    # list of positions
    ys = [y, y, -y]
    # append into data set with data set augmentation (+3)
    for j in range(0,3):
        current_image = images[j]
        current_y = ys[j]
        # build x
        x = image2vector(current_image)
        assert(x.shape == (height*width*3,1))
        #plt.imshow(x.reshape((height, width, 3)))
        #plt.show()
        X_train [:,3*i+j]=np.transpose(x/255.0) # from integer to float RGB
        #plt.imshow(X[:,i+j].reshape((height, width, 3)))
        #plt.show()
        # build y
        Y_train [0,3*i+j] = current_y
        #debug
        #plt.imshow(X_train[:,3*i+j].reshape((height, width, 3)))
        #plt.axvline(x=(Y_train[0,3*i+j]+1.0)/2.0*width,linewidth=3)
        #plt.show()

    
np.savetxt("Xtrain.txt",X_train,fmt="%f")
np.savetxt("Ytrain.txt",Y_train,fmt="%f")

