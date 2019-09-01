import os
import sys
import math
import numpy as np
import skimage
import random
import hashlib
import platform
import multiprocessing
import subprocess
from skimage import img_as_float
import time

datasetDir="02 - Track Line Pictures"
imgfondDir="01 - Background Pictures"

def GenereImagePerturbee(params):
  img, cutoff, ombre, lumiere, k, angle, posMin, largeurRelat, xBandeBasse, largeurRelatBas, xBandeMilieu, largeurRelatMilieu, i=params
  alphaLum=0.4
  alphaOmb=0.3
  img6=img+(1-img)*lumiere*alphaLum-ombre*alphaOmb
  name=k+"_a{:+04d}_pai{:+04.0f}_pri{:+04.0f}_pab{:+04.0f}_prb{:+04.0f}_pam{:+04.0f}_prm{:+04.0f}_{:04d}.jpeg".format(angle, posMin, largeurRelat, xBandeBasse, largeurRelatBas*100., xBandeMilieu, largeurRelatMilieu*100., i)
  destdirname=datasetDir #"{}/{}_a{:+04d}".format(datasetDir, k, angle) # cible avec rep
  filename=destdirname+"/"+name
  skimage.io.imsave(filename, skimage.util.img_as_ubyte(img6))
  #f=open(datasetDir+"/"+k+"_dataSet_infos.txt", "a")
  f=open(datasetDir+"/"+"dataSet_infos.txt", "a")
  print(filename+";{:+04.0f};{:+04.0f}".format(largeurRelatBas*100.,largeurRelatMilieu*100), file=f)
  f.close()
  print(name)


class Camera:
  sizeX=1280
  sizeY=720
  def __init__(self, hauteur, pointMilieuDroite, pointProcheDroite):
    """ Initialisation de la camera
      Paramètres :
      hauteur : hauteur de la camera en cm
      pointMilieuDroite : coordonnées réelles du point x max, y milieu
      pointProcheDroite : coordonnées réelles du point x max, y min
      distanceMilieu : distance du milieu de l'image
      Operations :
      - azimut : angle d'inclinaison de la camera
      - distanceMin : distance du bas de l'image
      - largeurMin : largeur de l'image en bas
      - yHorizon : hauteur de l'horizon sur l'image, en px
      - horizonVisible : flag indiquant si l'horizon est sur l'image ou au dessus
      - focale, en pixels
      - Matrice de transformation perspective
    """
    self.hauteur=hauteur
    self.distanceMilieu=pointMilieuDroite[1]
    self.demilargeurMilieu=pointMilieuDroite[0]
    self.distanceProche=pointProcheDroite[1]
    self.demilargeurProche=pointProcheDroite[0]
    # azimut, négatif
    self.azimut=-np.rad2deg(math.atan(hauteur/self.distanceMilieu))
    # distance entre camera et point de visée au centre
    self.hypothMilieuMilieu=math.sqrt(self.hauteur*self.hauteur+self.distanceMilieu*self.distanceMilieu)
    # distance entre camera et point milieu droite
    self.hypothMilieuDroite=math.sqrt(self.hypothMilieuMilieu*self.hypothMilieuMilieu+self.demilargeurMilieu*self.demilargeurMilieu)
    # distance entre camera et point proche milieu
    self.hypothProcheMilieu=math.sqrt(self.hauteur*self.hauteur+self.distanceProche*self.distanceProche)
    # distance entre camera et point de visée proche droite
    self.hypothProcheDroite=math.sqrt(self.hypothProcheMilieu*self.hypothProcheMilieu+self.demilargeurProche*self.demilargeurProche)
    # champ X : moyenne sur le champ calculé pour les points proches et milieu
    cXM=np.rad2deg(math.acos(self.hypothMilieuMilieu/self.hypothMilieuDroite))*2.
    cXP=np.rad2deg(math.acos(self.hypothProcheMilieu/self.hypothProcheDroite))*2.
    self.champX=(cXM+cXP)/2.
    self.champY=self.champX*self.sizeY/self.sizeX
    self.angleParPixelY=self.champY/self.sizeY
    # position de l'horizon sur l'image
    self.yHorizon=int(self.sizeY*(1.+math.sin(np.deg2rad(-self.azimut))/math.sin(np.deg2rad(self.champY/2.)))/2.)
    # Horizon visible ?
    print("Az={}, champY={}".format(self.azimut, self.champY/2.))
    if -self.azimut < self.champY/2.:
      self.HorizonVisible = True
    else:
      self.HorizonVisible = False    
    self.focale=0.4 
    # Distance du point de fuite de la bande
    # on utilise la résolution angulaire de la camera
    # le pixel sous l'horizon est vu avec un angle angleParPixelY
    # la distance du point sous l'horizon est a d=hauteur/tan(angleParPixelY)
    self.distPtFuite=self.hauteur/math.tan(np.deg2rad(self.angleParPixelY))
    # taille de l'image réelle
    self.realImgSizeX=int(self.distPtFuite*1.1*math.tan(np.deg2rad(self.champX/2.))*2.)
    self.realImgSizeY=int(self.distPtFuite*1.1)
    ### Matrice de transformation 
    self.matProjTransform=skimage.transform.ProjectiveTransform()
    pMD=np.array([self.demilargeurMilieu, self.distanceMilieu])
    pMG=np.array([-self.demilargeurMilieu, self.distanceMilieu])
    pPD=np.array([self.demilargeurProche, self.distanceProche])
    pPG=np.array([-self.demilargeurProche, self.distanceProche])
    srcPts=[pMG, pMD, pPG, pPD]
    dstPts=[np.array([0., self.sizeY/2.]), np.array([self.sizeX, self.sizeY/2.]), np.array([0., self.sizeY]), np.array([self.sizeX, self.sizeY])]
    print("Point Milieu droite = "+str(pointMilieuDroite))
    print("Point proche droite = "+str(pointProcheDroite))
    
    if self.matProjTransform.estimate(np.array(srcPts), np.array(dstPts)):
      print("Estimation OK")
      print(str(self.matProjTransform.params))
    else :
      print("Estimation KO !!!!!!!!!!!!")
  def Projection(self, x,y):
    """ Calcul de la position d'un point réel de coordonnées (x,y) 
        sur l'image. Utilise la matrice de transformation
    """
    point=np.zeros(3, dtype=float)
    point[0]=x
    point[1]=y
    point[2]=1.
    res=np.dot(self.matProjTransform.params, point)
    res[0]=res[0]/res[2]
    res[1]=res[1]/res[2]
    return [int(res[0]), int(res[1])]
    
class DataSet:
  # LargeurRel = position x réelle relative à la demi-largeur bas image (%)
  minLargeurRel=-96
  maxLargeurRel=96
  stepLargeurRel=2
  # angle = orientation de la ligne par rapport à l'axe de vue
  stepAngle=2  
  minAngle=-30
  maxAngle=30
  colorBlack=(30,30,30) # couleur ligne
  colorWhite=(230,230,230)
  bandeHauteurBas=0.18
  bandeHauteurMilieu=0.53
  bandeHauteurHaute=0.75 
  # line with, in cm
  lineWidth=5.
  def __init__(self, camera):
    self.images={}
    self.back_ground_picture_list = []
    self.back_ground_picture_count = 0
    self.camera=camera
    ombrevide=np.zeros((self.camera.sizeY, self.camera.sizeX, 3))
    self.imagesOmbres=[ombrevide]
    self.imagesLumières=[ombrevide]
    self.infotext=[]
    # analyse du CPU
    if platform.uname()[0] == 'Windows':
      self.nbOfCPU=int(subprocess.check_output(['wmic', 'cpu', 'get', 'numberoflogicalprocessors']).split()[1])
    else:
      self.nbOfCPU=1

  def AddImageFond(self, imgpath):
    # change : do not copy pictures in RAM
    self.back_ground_picture_list.append(imgpath) #save full path name of background pictures in a list
    self.back_ground_picture_count += 1 #save number of background pictures 
    #print("Image ajoutée : "+imgpath)

  def GenereOmbresEtLumières(self, n):
    for i in range(0,n):
      points=np.zeros((self.camera.sizeY, self.camera.sizeX, 3))
      for el in range(0, random.randint(100,300)):
        elx=random.randint(0, self.camera.sizeX)
        ely=random.randint(0, self.camera.sizeY)
        elrx=random.randint(1, 30)
        elry=random.randint(1, 30)
        ela=random.uniform(-math.pi, math.pi)
        [rr,cc]=skimage.draw.ellipse(ely, elx, elry, elrx, [self.camera.sizeY, self.camera.sizeX], ela)
        points[rr, cc]=(1.,1.,1.)
      ombres=skimage.filters.gaussian(points, sigma=10., multichannel=False)
      self.imagesOmbres.append(ombres)
    for i in range(0,n):
      points=np.zeros((self.camera.sizeY, self.camera.sizeX, 3))
      for el in range(0, random.randint(100,300)):
        elx=random.randint(0, self.camera.sizeX)
        ely=random.randint(0, self.camera.sizeY)
        elrx=random.randint(1, 30)
        elry=random.randint(1, 30)
        ela=random.uniform(-math.pi, math.pi)
        [rr,cc]=skimage.draw.ellipse(ely, elx, elry, elrx, [self.camera.sizeY, self.camera.sizeX], ela)
        points[rr, cc]=(1.,1.,1.)
      ombres=skimage.filters.gaussian(points, sigma=10., multichannel=False)
      self.imagesLumières.append(ombres)
  def AddLumiere(self, ombre, img, alpha):
    res=img+(1-img)*ombre*alpha
    return res
  def AddOmbres(self, ombre, img, alpha):
    res=img-ombre*alpha
    return res
  def AddLignes(self, redFactor=1.):
    """ Ajoute des lignes sur les images de fond
    """
    # Image size
    hImage=int(self.camera.sizeY*redFactor)
    wImage=int(self.camera.sizeX*redFactor)
    shape_image=[wImage, hImage]
    # directories
    source_dir=imgfondDir+" {}".format(hImage)
    dest_dir=datasetDir+" {}".format(hImage)
    print(dest_dir)
    if not os.path.isdir(source_dir):
      print("Source dir {} not found".format(src_dir))
      return
    if not os.path.isdir(dest_dir):
      print("Destination dir {} not found, will be created".format(dest_dir))
      return
    for root, dirs, files in os.walk(source_dir):
      for fname in files:
        ##print(root+"/"+fname)
        self.back_ground_picture_list.append(root+"/"+fname) #save full path name of background pictures in a list
        self.back_ground_picture_count += 1 #save number of background pictures 
    
    # shuffle background pictures
    picture_count = 0
    random.shuffle(self.back_ground_picture_list)
    ### reference lines y values
    yNear=hImage-1
    yLowBand=int(hImage*(1.-self.bandeHauteurBas))
    yMediumBand=int(hImage*(1.-self.bandeHauteurMilieu))
    yHighBand=int(hImage*(1.-self.bandeHauteurHaute))
    yHorizon=int((self.camera.sizeY-self.camera.yHorizon)*redFactor)
    yTop=0
    # line withs in Px at low and medium position
    lineHalfWidthPxNear=self.lineWidth/(self.camera.demilargeurProche*4)*wImage
    lineHalfWidthPxMedium=lineHalfWidthPxNear/self.camera.yHorizon*self.bandeHauteurMilieu
    lineHalfWidthPxHigh=lineHalfWidthPxNear/self.camera.yHorizon*self.bandeHauteurHaute
    ### generate lines for heach low, medium and horizon position
    nbStep=50
    step=int(100/nbStep)
    for pNear in range(0, 100+step, step):
      for pMedium in range(0,100+step, step):
        for pHorizon in range(0,100+step, step):
          for control in np.arange(0.6, 1.8, 0.2):
            for yControl in [yMediumBand, yHighBand]:
              # generate image
              imgname = self.back_ground_picture_list[picture_count]
              img=skimage.io.imread(imgname)
              picture_count += 1 # inc
              if(picture_count>=self.back_ground_picture_count):
                print("No more background picture available!!!")
                picture_count=0
                
              #### Generate Bezier curve with middle band height for control point
              print("Positions : {} {} {}".format(pNear, pMedium, pHorizon))
              xNear=int(pNear*wImage/100.)
              xMedium=int(pMedium*wImage/100.)
              xHorizon=int(pHorizon*wImage/100.)
              xNear0=xNear
              xMedium0=xMedium
              counter=0
              # generate middle Bezier curve
              # find low, middle and high bands x position
              # check if learning values are get, else changhe parameters
              while True:
                notFound=False
                [xBzC, yBzC]=skimage.draw.bezier_curve(xNear, yNear, xMedium, yControl, xHorizon, yHorizon, control, shape_image)
                if yLowBand in yBzC.tolist():
                  xLowBand=xBzC[yBzC.tolist().index(yLowBand)]
                else:
                  if pNear==100:
                    xLowBand=wImage-1
                  else:
                    print("##### bande basse non visible")
                    xLowBand=None
                    notFound=True
                if yMediumBand in yBzC.tolist():
                  xMediumBand=xBzC[yBzC.tolist().index(yMediumBand)]
                else:
                  if pMedium==100:
                    xMediumBand=wImage-1
                  else:
                    print("##### bande milieu non visible")
                    xMediumBand=None
                    notFound=True
                if yHighBand in yBzC.tolist():
                  xHighBand=xBzC[yBzC.tolist().index(yHighBand)]
                else:
                  if pHorizon==100:
                    xHighBand=wImage-1
                  else:
                    print("##### bande haute non visible")
                    xHighBand=None
                    notFound=True
                if notFound :
                  if control>20:
                    break
                  xNear=xNear0+random.randint(0, int(lineHalfWidthPxNear*0.2))
                  xMedium=xMedium0+random.randint(0, int(lineHalfWidthPxNear*0.2))
                  control+=1
                else:
                  break
              # generate white Bezier curves
              for dxNear in range(0,int(lineHalfWidthPxNear)+1):
                dxMedium=int(dxNear*lineHalfWidthPxMedium/lineHalfWidthPxNear)
                [xBzCt, yBzCt]=skimage.draw.bezier_curve(xNear+dxNear, yNear, xMedium+dxMedium, yControl, xHorizon, yHorizon,control, shape_image)
                xBzC=np.append(xBzC, xBzCt)
                yBzC=np.append(yBzC, yBzCt)
                [xBzCt, yBzCt]=skimage.draw.bezier_curve(xNear-dxNear, yNear, xMedium-dxMedium, yControl, xHorizon, yHorizon, control, shape_image)
                xBzC=np.append(xBzC, xBzCt)
                yBzC=np.append(yBzC, yBzCt)
              img[yBzC, xBzC]=np.add( np.multiply(img[yBzC, xBzC],0.05), np.multiply(self.colorWhite,0.95)) # blend
              # generate black Bezier curves
              if random.randint(0, 1) == 1:
                [xBzC, yBzC]=skimage.draw.bezier_curve(xNear+int(lineHalfWidthPxNear), yNear, xMedium+int(lineHalfWidthPxMedium), yControl, xHorizon, yHorizon, control, shape_image)
                for dxNear in range(0,int(lineHalfWidthPxNear)+1):
                  dxMedium=int(dxNear*lineHalfWidthPxMedium/lineHalfWidthPxNear)
                  [xBzCt, yBzCt]=skimage.draw.bezier_curve(xNear+dxNear+int(lineHalfWidthPxNear), yNear, xMedium+dxMedium+int(lineHalfWidthPxMedium), yControl, xHorizon, yHorizon,control, shape_image)
                  xBzC=np.append(xBzC, xBzCt)
                  yBzC=np.append(yBzC, yBzCt)
                  [xBzCt, yBzCt]=skimage.draw.bezier_curve(xNear-dxNear-int(lineHalfWidthPxNear), yNear, xMedium-dxMedium-int(lineHalfWidthPxMedium), yControl, xHorizon, yHorizon, control, shape_image)
                  xBzC=np.append(xBzC, xBzCt)
                  yBzC=np.append(yBzC, yBzCt)
                img[yBzC, xBzC]=np.add( np.multiply(img[yBzC, xBzC],0.05), np.multiply(self.colorWhite,0.05)) # blend
              name=dest_dir+"/{}h".format(picture_count)+imgname[len(source_dir)+1:]+"_{:+04d}_{:+04d}_{:+04d}_{:1.1f}.jpeg".format(picture_count, xNear, xMedium, xHorizon, control)
              skimage.io.imsave(name, skimage.util.img_as_ubyte(img))
              f=open(dest_dir+"/"+"dataSet_infos.txt", "a")
              print(name+";{:+.3f};{:+.3f};{:+.3f}".format((xLowBand-wImage/2.)/wImage*2.,(xMediumBand-wImage/2.)/wImage*2., (xHighBand-wImage/2.)/wImage*2.), file=f)
              f.close()


if __name__ == '__main__':
  # initialisation de la camera : 
  # hauteur
  # point à droite à mi hauteur
  # point en bs à droite
  # Caméra sur TT01 : h=16, côté milieu (25, 51), côté bas (13.5, 24)
#  camera=Camera(16., [25., 51.], [13.5, 24.]) 
  camera=Camera(16., [62., 60.], [22.5, 17.]) 
  print(" champ X = {:.0f}°".format(camera.champX))
  print(" champ Y = {:.0f}°".format(camera.champY))
  print("  Hauteur = {:.0f} cm".format(camera.hauteur))
  print(" D milieu = {:.0f} cm".format(camera.distanceMilieu))
  print("Larg. mil = {:.0f} cm".format(camera.demilargeurMilieu))
  print("   azimut = {:.2f}°".format(camera.azimut))
  print("D minimum = {:.0f} cm".format(camera.distanceProche))
  print("Pos horiz = {:.0f}".format(camera.yHorizon))
  print("Larg. min = {:.1f} cm".format(camera.demilargeurProche))
  print("   focale = {:.0f}".format(camera.focale))
  print(" pt fuite = {:.0f} cm".format(camera.distPtFuite))
  print(" Horizon visible : "+str(camera.HorizonVisible))

  if not os.path.isdir(datasetDir):
    os.mkdir(datasetDir)

  random.seed(0)
  dataSet=DataSet(camera)
  # Génération du dataset en 720p
#  dataSet.AddLignes(redFactor=1.)
  # Génération du dataset en 90p
  dataSet.AddLignes(redFactor=1./8.)
