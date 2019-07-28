Principes des images générées :
On génère une ligne tournée d'un certain angle et décalée par rapport au milieu de la caméra,
l'image résultante est assombrie et modifiée avec des zones claires et des zones sombres pour
simuler le soleil à travers un feuillage.
Un dataset est composé de lignes orientées de -45° à +45° par pas de 5°, 
et décalées de -200% à +200 par pas de 20% de la largeur du bas de l'image, 
puis l'image est modifiée par les effets d'éclairages.
Les valeurs des angles et du décalages, ainsi que les pas sont modifiables
dans le script (variables de la classe DataSet).
On donne la position latérale de la ligne à 28% et 65% (soit 200 px et 470px environ) 
de la hauteur de l'image.

Le nom des images du dataset est de la forme :
- angle d'orientation de la ligne
- position de la ligne en bas de l'image : absolue en cm (au sol) et relative en %
- position de la ligne à 200px de haut : absolue en cm (au sol) et relative en %
- position de la ligne à 470px de haut : absolue en cm (au sol) et relative en %
Le nom de l'image et la position de la ligne à 200px et 470px est dans un fichier texte.

Mode d'emploi du générateur de dataset :
- créer les dossiers "Photos fond/" et "dataset_720/"
- mettre les images de fond dans le dossier "Photos fond"
- lancer le script
Le dataset sera créé dans le répertoire "dataset_720/"

Quelques infos complémentaires
Les images du dataset sont nommées à partir du hash (8 permiers caractères) du nom de l'image de fond.
Si une image dans le répertorie "dataset_720/" a un nom commençant par le même haché que celui d'une
image de fond, celle-ci sera ignorée.
Le séquencement de la génération est :
- angles
  - décalage horizontal de la ligne
    - images de fond
      - éclaircissement ou assombrissement
        - ajout des ombres et limières
  

