import os
import sys
import skimage
import random
from skimage.util import img_as_ubyte
import multiprocessing
import subprocess

# parameters
input_dir = "00 - Background Photos\\"
output_dir = "01 - Background Pictures\\"
random_seed = 666
number_of_pictures = 8
batch_size=4

# constants
margin = 150
w_target_size=1280
h_target_size=720

ResizeFactor=1./8.
output_dir_resized=output_dir[:-1]+" {}\\".format(int(h_target_size*ResizeFactor))

def Generation(params):
  img=params[0]
  input_file=params[1]
  i=params[2]
  print("Inputfile : {} {}".format(input_file, i))
  angle=random.randint(-25,+25)
  rotImage=skimage.transform.rotate(img, angle)
  #print(str(rotImage.shape))
  if rotImage.shape[0] > 2*(w_target_size+2*margin) and (rotImage.shape[1] > 2*(h_target_size+2*margin)):
    resizeImage=skimage.transform.rescale(rotImage, 0.5)
  else:
    resizeImage=rotImage
  (h_source_size, w_source_size, z)=resizeImage.shape
  print("src {} target {} margin {}".format(w_source_size, w_target_size, margin))
  xc=random.randint(margin,w_source_size-w_target_size-margin)
  yc=random.randint(margin,h_source_size-h_target_size-margin)
  cropImage=resizeImage[yc:yc+h_target_size, xc:xc+w_target_size]
  rotImage2=skimage.transform.rotate(cropImage, 180)
  skimage.io.imsave(output_dir+input_file+"{:03d}rot.jpeg".format(i), cropImage)
  skimage.io.imsave(output_dir+input_file+"{:03d}flp.jpeg".format(i), rotImage2)
  cropImageResized=skimage.transform.rescale(cropImage, 1/8)
  rotImage2Resized=skimage.transform.rescale(rotImage2, 1/8)
  skimage.io.imsave(output_dir_resized+input_file+"{:03d}rot.jpeg".format(i), cropImageResized)
  skimage.io.imsave(output_dir_resized+input_file+"{:03d}flp.jpeg".format(i), rotImage2Resized)

if __name__ == '__main__':
  random.seed(random_seed)
  nbOfCPU=int(subprocess.check_output(['wmic', 'cpu', 'get', 'numberoflogicalprocessors']).split()[1])

  print("Nb of CPU : {}".format(nbOfCPU))

  for batch_number in range(0,number_of_pictures//batch_size, batch_size):
    for input_file in os.listdir(input_dir):
      baseImage=skimage.io.imread(input_dir+input_file)
      set_of_params=[]
      for i in range(0, batch_size):
        params=[baseImage, input_file, i+batch_number*batch_size]
        set_of_params.append(params)
    #    Generation(params)
      print("Starting pool, {} params in set".format(len(set_of_params)))
      p = multiprocessing.Pool(nbOfCPU)
      p.map(Generation, set_of_params)
      p.close()
      p.join()
