/*
 * TFminiPlus.h
 *
 *  Created on: 30/07/2019
 *      Author: Nicolas
 */

#ifndef TFMINIPLUS_H_
#define TFMINIPLUS_H_

// Liste des formats de sortie possible.
// Permet d'avoir la résolution (mm ou cm)
typedef enum
{
	standard_cm,
	pixhawk,
	standard_mm
} eLidarOutputFormat;

// Les index sur les capteurs (à passer dans le champ a_numCapteur des fonctions)
#define MINILIDAR_DROIT		1
#define MINILIDAR_GAUCHE 	2

// Définition des fonctions d'interface
int tfminiplus_getLastAcquisition(int a_numCapteur, int *a_pDistance, int *a_pStrength, int *a_pTemperature);
int tfminiplus_setOutputFormat(int a_numCapteur, eLidarOutputFormat *a_pFormat);
int tfminiplus_getFramerate(int a_numCapteur, int *a_pFramerate);
int tfminiplus_getBaudrate(int a_numCapteur, int *a_pBaudrate);
int tfminiplus_getVersion(int a_numCapteur, int *a_pVersion);
int tfminiplus_init();
void tfminiplusIrq(int a_numCapteur);

#endif /* TFMINIPLUS_H_ */
