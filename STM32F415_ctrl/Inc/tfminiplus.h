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
typedef enum {
	MINILIDAR_DROIT,
	MINILIDAR_GAUCHE,
	MINILIDAR_HAUT
} LIDAR_ID;

// Définition des fonctions d'interface
int tfminiplus_getLastAcquisition(LIDAR_ID a_numCapteur, int32_t *a_pDistance, int32_t *a_pStrength, int32_t *a_pTemperature);
int tfminiplus_setOutputFormat(LIDAR_ID a_numCapteur, eLidarOutputFormat *a_pFormat);
int tfminiplus_getFramerate(LIDAR_ID a_numCapteur, int32_t *a_pFramerate);
int tfminiplus_getBaudrate(LIDAR_ID a_numCapteur, int32_t *a_pBaudrate);
int tfminiplus_getVersion(LIDAR_ID a_numCapteur, int32_t *a_pVersion);
int tfminiplus_init();
void tfminiplusIrq(LIDAR_ID a_numCapteur);

#endif /* TFMINIPLUS_H_ */
