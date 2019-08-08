import sys
import skimage
import random
from skimage.util import img_as_ubyte

margin = 150
w_target_size=1280
h_target_size=720

if len(sys.argv) <= 2:
  print(sys.argv[0]+" image nombre [seed]")
  print("  Génération d'images extraites")
  sys.exit(0)
  
baseFileName=sys.argv[1]
nbAGenerer=int(sys.argv[2])
if len(sys.argv) > 3:
  random.seed(int(sys.argv[3]))
else:
  random.seed(0)
  
print("Extraction de {} images à partir de {}".format(nbAGenerer, baseFileName))
baseImage=skimage.io.imread(baseFileName)

for i in range(0, nbAGenerer):
  angle=random.randint(-25,+25)
  rotImage=skimage.transform.rotate(baseImage, angle)
  #print(str(rotImage.shape))
  resizeImage=skimage.transform.rescale(rotImage, 0.5)
  (h_source_size, w_source_size, z)=resizeImage.shape
  xc=random.randint(margin,w_source_size-w_target_size-margin)
  yc=random.randint(margin,h_source_size-h_target_size-margin)
  cropImage=resizeImage[yc:yc+h_target_size, xc:xc+w_target_size]
  skimage.io.imsave("out/"+baseFileName+"rot{:03d}.jpeg".format(i), cropImage)
  rotImage2=skimage.transform.rotate(cropImage, 180)
  skimage.io.imsave("out/"+baseFileName+"flp{:03d}.jpeg".format(i), rotImage2)
  
