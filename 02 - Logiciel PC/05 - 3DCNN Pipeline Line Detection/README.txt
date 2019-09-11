Pb : End-to-end (3D)CNN overtfitting / no robustesness / no generalization

3DCNN entrainé sur ligne : donne position de la ligne ==> PID Steering

Training
	- virtuelle dynamique type Toulouse (hand-labeling  à partir dataset)
	- réelle dynamique a CRISTAL (hand-labeling  à partir dataset)
	- virtuel statique (labeling  scripté)

Outils : 
	01 - Line picture (picture+label)
	01 - Video Labeling (sequence+label)
	01 - Dataset Labeling (sequence+label)
	02 - Labeling to 3D dataset (+flip)
	03 - Training + Data augmentation (+shadow/light/color +neural style transfer)
	04 - Predict dataset
	05 - Simulateur autopilot + PID
	06 - Biggy autopilot + PID



