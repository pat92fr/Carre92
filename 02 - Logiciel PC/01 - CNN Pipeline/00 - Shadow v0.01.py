import os
import sys
import skimage
import random
from skimage.util import img_as_ubyte
from skimage.color import rgb2gray

# parameters
input_dir = "00 - Shadow Photos\\"
output_dir = "01 - Shadow Pictures\\"
random_seed = 999
number_of_pictures = 500

# constants
margin = 10
w_target_size=160
h_target_size=32

random.seed(random_seed)

for input_file in os.listdir(input_dir):

  original=skimage.io.imread(input_dir+input_file)
  (h_source_size, w_source_size, z)=original.shape
  grayscale = rgb2gray(original)

  for i in range(0, number_of_pictures):

    xc=random.randint(margin,w_source_size-2*w_target_size-margin)
    yc=random.randint(margin,h_source_size-2*w_target_size-margin)
    cropImage=grayscale[yc:yc+2*w_target_size, xc:xc+2*w_target_size]

    angle=random.randint(0,360)
    rotImage=skimage.transform.rotate(cropImage, angle)

    xc2=int(w_target_size/4.0)
    yc2=int(w_target_size/2.0)
    finalImage=rotImage[yc2:yc2+h_target_size, xc2:xc2+w_target_size]
    finalImage2=skimage.transform.rotate(finalImage,180)

    skimage.io.imsave(output_dir+input_file+"{:03d}rot.jpeg".format(i), finalImage)
    skimage.io.imsave(output_dir+input_file+"{:03d}flp.jpeg".format(i), finalImage2)
  
