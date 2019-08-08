import sys
import skimage
import random

xSize=1280
ySize=720
size=max(xSize, ySize)

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
(yImage, xImage, z)=baseImage.shape

if xImage <= size or yImage <= size :
  print("Image trop petite...")
  sys.exit(0)
  
for i in range(0, nbAGenerer):
  angle=random.randint(0,360)
  xc=random.randint(int(size/2),xImage-size-int(size/2))
  yc=random.randint(int(size/2),yImage-size-int(size/2))
  rotImage=skimage.transform.rotate(baseImage, angle)
  img=rotImage[yc:yc+ySize, xc:xc+xSize]
  skimage.io.imsave("out/"+baseFileName+"rot{:03d}.jpeg".format(i), img)
