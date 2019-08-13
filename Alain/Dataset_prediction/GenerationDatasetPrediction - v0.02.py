# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

# Nombre de valeur à générer
SampleSet=10

# orientation de la ligne avec l'axe
# en degrés
AngleMin=-20
AngleMax=+20
AngleStep=1.

# Décalage de la ligne au démarrage
# en % de la demi-largeur d'image
PosMin=-50.
PosMax=+50.
PosStep=10.

DataFileName="predictiondata.txt"

f=open(DataFileName, "w")
for angle in np.arange(AngleMin, AngleMax+AngleStep, AngleStep) :
  for pos in np.arange(PosMin, PosMax+PosStep, PosStep) :
    #################################
    # ligne droite pour courbure nulle
    # le virage est visible au moins sur les deux derniers points 
    for d in range(0, SampleSet-1): # nb de points de ligne droite avant la courbe
      for rCourbure in np.arange(-200, 200, 10): # rayon de courbure
        for angleParStep in np.arange(1, 90/SampleSet, 9/SampleSet): # vitesse d'avance le long de la courbe
          valeurs=np.zeros(SampleSet+1, dtype=float)
          # ligne droite
          for i in range(0, d):
            valeurs[i]=pos+i*np.tan(np.deg2rad(angle))
          # courbe à droite
          for i in range(d, SampleSet+1):
            valeurs[i]=pos+i*np.tan(np.deg2rad(angle))+(1-np.cos(np.deg2rad((i-d+1)*angleParStep)))*rCourbure
          # ecriture des resultats
          sortie=""
          for i in range(0, SampleSet+1):
            sortie=sortie+"{:.2f};".format(valeurs[i])
          sortie=sortie+"{:03d};{:03d};".format(int(angle), int(pos))
          print(sortie, file=f)
          # plt.axis((0,SampleSet, -150, 150))
          # plt.plot(range(0, SampleSet+1), valeurs)
          # plt.show()
f.close()