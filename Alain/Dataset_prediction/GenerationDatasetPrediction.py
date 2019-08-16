# -*- coding: utf-8 -*-
import numpy as np

# Nombre de valeur à générer
SampleSet=10

# orientation de la ligne avec l'axe
# en degrés
AngleMin=-20.
AngleMax=+20.
AngleStep=1.

# Décalage de la ligne au démarrage
# en % de la demi-largeur d'image
PosMin=-100.
PosMax=+100.
PosStep=1.

# Vitesse de la camera
# equivalent au pas en y
SpeedMin=1.
SpeedMax=20.
SpeedStep=1

# variance des perturbations aléatoires
Variance=0

DataFileName="predictiondata.txt"

f=open(DataFileName, "w")
for angle in np.arange(AngleMin, AngleMax+AngleStep, AngleStep) :
  for pos in np.arange(PosMin, PosMax+PosStep, PosStep) :
    values=np.zeros(SampleSet+1, dtype=float)
    speed=1.
    sortie=""
    for i in range(0, SampleSet+1):
      val=pos+i*speed*np.tan(np.deg2rad(angle))
      sortie=sortie+"{:.2f};".format(val)
    sortie=sortie+"{:03d};{:03d};".format(int(angle), int(pos))
    print(sortie, file=f)
    
f.close()
