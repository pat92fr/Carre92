import os
import sys
import skimage
import random
from skimage.util import img_as_ubyte

# parameters
input_dir = "00 - Background Photos\\"
output_dir = "01 - Background Pictures\\"
random_seed = 666
number_of_pictures = 350

# constants
margin = 150
w_target_size=1280
h_target_size=720

random.seed(random_seed)
for input_file in os.listdir(input_dir):
  baseImage=skimage.io.imread(input_dir+input_file)
  for i in range(0, number_of_pictures):
    angle=random.randint(-25,+25)
    rotImage=skimage.transform.rotate(baseImage, angle)
    #print(str(rotImage.shape))
    resizeImage=skimage.transform.rescale(rotImage, 0.5)
    (h_source_size, w_source_size, z)=resizeImage.shape
    xc=random.randint(margin,w_source_size-w_target_size-margin)
    yc=random.randint(margin,h_source_size-h_target_size-margin)
    cropImage=resizeImage[yc:yc+h_target_size, xc:xc+w_target_size]
    rotImage2=skimage.transform.rotate(cropImage, 180)
    skimage.io.imsave(output_dir+input_file+"{:03d}rot.jpeg".format(i), cropImage)
    skimage.io.imsave(output_dir+input_file+"{:03d}flp.jpeg".format(i), rotImage2)
  
