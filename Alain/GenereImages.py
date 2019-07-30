import os
import math
import numpy as np
import skimage
import random
import hashlib

datasetDir="dataset_720"
imgfondDir="Photos fond"

class Camera:
  champX=46.8
  sizeX=1280
  sizeY=720
  champY=sizeY/sizeX*champX
  angleParPixelY=champY/sizeY
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
      - positionHorizon : hauteur de l'horizon sur l'image, en px
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
    self.positionHorizon=self.sizeY*(1.+math.sin(np.deg2rad(-self.azimut))/math.sin(np.deg2rad(self.champY/2.)))/2.
    # Horizon visible ?
    if self.azimut < self.champY/2.:
      self.HorizonVisible = True
    else:
      self.HorizonVisible = False    
    self.focale=0.4 
    # Distance du point de fuite de la bande
    # on utilise la résolution angilaire de la camera
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
  # LargeurRel = position x réelle relative à la demi-largeur bas image
  minLargeurRel=-200
  maxLargeurRel=200 
  stepLargeurRel=20
  # angle = orientation de la ligne par rapport à l'axe de vue
  stepAngle=5  
  minAngle=-45 
  maxAngle=45  
  colorBlack=(64,64,64)
  colorWhite=(192,192,192)
  bandeHauteurBas=0.28
  bandeHauteurHaut=0.65
  def __init__(self, camera):
    self.images={}
    self.imagesFond={}
    self.camera=camera
    ombrevide=np.zeros((self.camera.sizeY, self.camera.sizeX, 3))
    self.imagesOmbres=[ombrevide]
    self.imagesLumières=[ombrevide]
    self.infotext=[]
  def AddImageFond(self, imgpath):
    name=hashlib.sha256(imgpath.encode('utf-8')).hexdigest()[0:8]
    if not os.path.isdir(datasetDir):
      os.mkdir(datasetDir)
    # dataset deja fait pour cette image ?
    subdirs=[]
    for root, dir, file in os.walk(datasetDir):
      if root == datasetDir:
        subdirs.append(dir)
    if name in subdirs:
        print("Image déjà traitée : "+imgpath)
        return
    else:
      os.mkdir(datasetDir+"/"+name)
      img=skimage.io.imread(imgpath)
      img2=skimage.transform.resize(img, [self.camera.sizeY, self.camera.sizeX])
      self.imagesFond[name]=img2
      print("Image ajoutée : "+name+" "+imgpath)
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
  def AddLignes(self):
    """ Ajoute des lignes sur les images de fond
    """
    # On verifie qu'il y a des images de fond
    if not self.imagesFond:
      return
    # Création des sous-répertoires du dataset
    for k in self.imagesFond.keys():
      for angle in range(self.minAngle, self.maxAngle+self.stepAngle, self.stepAngle):
        destdirname="{}/{}_a{:+04d}".format(datasetDir, k, k, angle)
        if not os.path.isdir(destdirname):
          os.mkdir(destdirname)
    # calcul de la distance à la caméra pour les bandes basses et hautes
    ecartBandeBas=(0.5-self.bandeHauteurBas)*2
    thetaBas=np.rad2deg(math.asin(math.sin(np.deg2rad(self.camera.champY/2.))*ecartBandeBas))
    distanceBandeBas=self.camera.hauteur/math.tan(np.deg2rad(-self.camera.azimut+thetaBas))
    [x, yBandeBasse]=self.camera.Projection(0, distanceBandeBas)
    ecartBandeMilieu=(0.5-self.bandeHauteurHaut)*2
    thetaMilieu=np.rad2deg(math.asin(math.sin(np.deg2rad(self.camera.champY/2.))*ecartBandeMilieu))
    distanceBandeMilieu=self.camera.hauteur/math.tan(np.deg2rad(-self.camera.azimut+thetaMilieu))
    [x, yBandeMilieu]=self.camera.Projection(0, distanceBandeMilieu)
    for angle in range(self.minAngle, self.maxAngle+self.stepAngle, self.stepAngle):
      for largeurRelat in range(self.minLargeurRel, self.maxLargeurRel+self.stepLargeurRel, self.stepLargeurRel):
        # position réelle de la bande sur la transversale au bas de l'image
        posMin=largeurRelat/100.*self.camera.demilargeurProche
        cosangle=math.cos(np.deg2rad(angle))
        posBR=posMin+7.5/cosangle # pos bande noire droite
        posWR=posMin+2.5/cosangle # pos bande blanche droite
        posWL=posMin-2.5/cosangle # pos bande blanche gauche
        posBL=posMin-7.5/cosangle # pos bande noire gauche
        posPtFuite=posMin+self.camera.distPtFuite*math.tan(np.deg2rad(angle))
        # Calcul des points sur l'image
        imgMin=self.camera.Projection(posMin, self.camera.distanceProche)
        imgBR=self.camera.Projection(posBR, self.camera.distanceProche)
        imgWR=self.camera.Projection(posWR, self.camera.distanceProche)
        imgWL=self.camera.Projection(posWL, self.camera.distanceProche)
        imgBL=self.camera.Projection(posBL, self.camera.distanceProche)
        imgPF=self.camera.Projection(posPtFuite, self.camera.distPtFuite)
        # creation des bandes
        # ligne noire gauche
        r=np.array([imgBL[1], imgWL[1], imgPF[1]]) # y
        c=np.array([imgBL[0], imgWL[0], imgPF[0]])
        [rrBL,ccBL]=skimage.draw.polygon(c,r, [self.camera.sizeX, self.camera.sizeY])
        # ligne blanche
        r=np.array([imgWL[1], imgWR[1], imgPF[1]]) # y
        c=np.array([imgWL[0], imgWR[0], imgPF[0]])
        [rrW,ccW]=skimage.draw.polygon(c,r, [self.camera.sizeX, self.camera.sizeY])
        # ligne noire droite
        r=np.array([imgWR[1], imgBR[1], imgPF[1]]) # y
        c=np.array([imgWR[0], imgBR[0], imgPF[0]])
        [rrBR,ccBR]=skimage.draw.polygon(c,r, [self.camera.sizeX, self.camera.sizeY])
        # Ligne de référence
        r=np.array([imgMin[1], imgPF[1]]) # y
        c=np.array([imgMin[0], imgPF[0]])
        [rrWu,ccWu]=skimage.draw.line(imgMin[0], imgMin[1], imgPF[0], imgPF[1])
        index=ccWu.tolist().index(yBandeBasse)
        xBandeBasse=rrWu[index]
        largeurRelatBas=(xBandeBasse-self.camera.sizeX/2)/self.camera.sizeX*2
        index=ccWu.tolist().index(yBandeMilieu)
        xBandeMilieu=rrWu[index]
        largeurRelatMilieu=(xBandeMilieu-self.camera.sizeX/2)/self.camera.sizeX*2
        [rrBas, ccBas]=skimage.draw.line(0,yBandeBasse, self.camera.sizeX-1, yBandeBasse)
        [rrMil, ccMil]=skimage.draw.line(0,yBandeMilieu, self.camera.sizeX-1, yBandeMilieu)
        
        if rrW.size > 0 : # bande blanche visible sur l'image
          for k in self.imagesFond.keys():
            img=np.copy(self.imagesFond[k])
            img2=img*255
            img2[ccW, rrW]=self.colorWhite
            img2[ccBL, rrBL]=self.colorBlack
            img2[ccBR, rrBR]=self.colorBlack
            img3=img2.astype(np.uint8)
            # Metadonnées :
            # a angle de la ligne
            # pai position réelle de la ligne au bas de l'image
            # pri position relative de la ligne au bas de l'image (% de la demi largeur)
            # pab position réelle de la ligne sur la bande basse
            # prb position relative de la ligne sur la bande basse (% de la demi largeur)
            # pah position réelle de la ligne sur la bande haute
            # prh position relative de la ligne sur la bande haute (% de la demi largeur)
            print("Angle={:+03.0f}, pos réelle bas={:+03.0f}, pos relat bas={:+03.0f}".format(angle, posMin, largeurRelat))
            print("Bande bas : réel={:+03.0f} relatif={:03.0f}".format(xBandeBasse, largeurRelatBas*100.))
            print("Bande Mil : réel={:+03.0f} relatif={:03.0f}".format(xBandeMilieu, largeurRelatMilieu*100.))
            i=0
            # contraste
            for cutoff in np.arange(0.3,0.75,0.05):
              img4=skimage.util.img_as_float(skimage.exposure.adjust_sigmoid(img3, cutoff, 10))
              # ombres et lumières
              for l in self.imagesLumières:
                for o in self.imagesOmbres:
                  img5=self.AddLumiere(l, img4, 0.4)
                  img6=self.AddOmbres(o, img5, 0.3)
                  name=k+"_a{:+04d}_pai{:+04.0f}_pri{:+04.0f}_pab{:+04.0f}_prb{:+04.0f}_pam{:+04.0f}_prm{:+04.0f}_{:04d}.jpeg".format(angle, posMin, largeurRelat, xBandeBasse, largeurRelatBas*100., xBandeMilieu, largeurRelatMilieu*100, i)
                  destdirname="{}/{}_a{:+04d}".format(datasetDir, k, k, angle)
                  filename=destdirname+"/"+name
                  skimage.io.imsave(filename, skimage.util.img_as_ubyte(img6))
                  print(name)
                  f=open(datasetDir+"/"+k+"_dataSet_infos.txt", "a")
                  print(filename+";{:+04.0f};{:+04.0f}".format(largeurRelatBas*100.,largeurRelatMilieu*100), file=f)
                  f.close()
                  i=i+1
  

# initialisation de la camera : 
# hauteur
# point à droite à i hauteur
# point en bs à droite
camera=Camera(20., [50., 80], [22.39, 33.23])
print(" champ X = {:.0f}°".format(camera.champX))
print(" champ Y = {:.0f}°".format(camera.champY))
print("  Hauteur = {:.0f} cm".format(camera.hauteur))
print(" D milieu = {:.0f} cm".format(camera.distanceMilieu))
print("Larg. mil = {:.0f} cm".format(camera.demilargeurMilieu))
print("   azimut = {:.2f}°".format(camera.azimut))
print("D minimum = {:.0f} cm".format(camera.distanceProche))
print("Pos horiz = {:.0f}".format(camera.positionHorizon))
print("Larg. min = {:.0f} cm".format(camera.demilargeurProche))
print("   focale = {:.0f}".format(camera.focale))
print(" pt fuite = {:.0f} cm".format(camera.distPtFuite))


random.seed(0)
dataSet=DataSet(camera)

for root, dirs, files in os.walk(imgfondDir):
  for fname in files:
    print(root+"/"+fname)
    dataSet.AddImageFond(root+"/"+fname)

# Génération des masques pour les ombres et lumières
dataSet.GenereOmbresEtLumières(4)
# Génération du dataset
dataSet.AddLignes()
