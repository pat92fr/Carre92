/*
 * TF Mini Lidar.c
 *
 *  Created on: 18/07/2018
 *      Author: Nicolas
 */

#include "stm32f4xx_hal.h"
#include "tfminiplus.h"

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

// Timeout de 1000 ms
#define TIMEOUT 1000

// Définition d'une structure qui liste tous les éléments d'un capteur.
// Ceci permet d'instancier plusieurs capteurs dans un même logiciel facilement
typedef struct
{
	// Pointeur vers l'uart utilisé
	UART_HandleTypeDef *pHuart;

	// La taille max d'une trame, dans tout le protocole, est de 8 octets
	uint8_t serialBuffer[10];
	// Dernière distance en cm
	int distance;
	// Intensité du rayon reçu
	int strength;
	// Temperature du sensor en centigrade
	int temperature;

	// Format : 00.version.revision.edition
	int32_t version;

	// Frequence d'acquisition
	int framerate;
	// Frequence de la liaison serie
	int baudrate;
	// Type de format de données
	eLidarOutputFormat outputFormat;

	// Semaphore pour attendre la réponse du lidar à une commande
	int semaphore;

} stMiniLidar;

// Instanciation des capteurs
stMiniLidar miniLidarDroit;
stMiniLidar miniLidarGauche;

// Dans la routine d'IRQ du DMA, on positionne le numéro du capteur rattaché au DMA
// Remarque générale, plutot que de typer numCapteur en int, on pourrait mettre un enum
void tfminiplusIrq(int a_numCapteur)
{
	int distance, strength, temp;
	uint32_t checksum, checksum_ref;
	stMiniLidar *pLidar;

	if(a_numCapteur == MINILIDAR_DROIT)
		pLidar = &miniLidarDroit;
	else if(a_numCapteur == MINILIDAR_GAUCHE)
		pLidar = &miniLidarGauche;
	else
		pLidar = 0;

	if(pLidar != 0)
	{
		// On réarme le DMA
		HAL_UART_Receive_DMA(pLidar->pHuart, pLidar->serialBuffer, 9);

		// On vérifie l'entête
		if(pLidar->serialBuffer[0] == 0x59)
		{
			// C'est une trame de donnée

			// On vérifie que le deuxième octet est correct
			// Ce driver ne gère que le format standard mais pas le format Pixhawk
			if(pLidar->serialBuffer[1] == 0x59)
			{
				// On vérifie le checksum
				checksum = 0;
				for (int i=0; i<8;i++) checksum += pLidar->serialBuffer[i];
				checksum &= 0xFF;
				checksum_ref = pLidar->serialBuffer[8];
				if(checksum == checksum_ref)
				{
					// La trame est correcte, on traite les données
					// On constitue les valeurs réelles
					distance = pLidar->serialBuffer[2] + (pLidar->serialBuffer[3] << 8);
					strength = pLidar->serialBuffer[4] + (pLidar->serialBuffer[5] << 8);
					temp =     pLidar->serialBuffer[6] + (pLidar->serialBuffer[7] << 8);
					// Si la force du signal de retour est suffisante, la donnée de distance est valable
					if((strength>100) && (strength!=65535))
					{
						pLidar->distance = distance;
						pLidar->strength = strength;
					}
					// Sinon, on laisse les valeurs de distance et d'intensité précédentes
					pLidar->temperature = temp;
				}
			}
		} else if (pLidar->serialBuffer[0] == 0x5A)
		{
			// C'est une trame de réponse à une commande
			if((pLidar->serialBuffer[1] == 0x07) &&
			   (pLidar->serialBuffer[2] == 0x01))
			{
				// On vérifie le checksum
				checksum = 0;
				for (int i=0; i<6;i++)
					checksum += pLidar->serialBuffer[i];
				checksum &= 0xFF;
				checksum_ref = pLidar->serialBuffer[6];
				if(checksum == checksum_ref)
				{
					// La trame est correcte, on traite les données
					// On récupère les numéros de version
					// Format 00.V3.V2.V1
					pLidar->version = pLidar->serialBuffer[3] + (pLidar->serialBuffer[4] << 8) + (pLidar->serialBuffer[5] << 16);
					pLidar->semaphore++;
				}
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x02) &&
					(pLidar->serialBuffer[3] == 0x00) &&
					(pLidar->serialBuffer[4] == 0x60))
			{
				// Le capteur va reseter
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x02) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x61))
			{
				// Le capteur refuse de reseter
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x06) &&
					(pLidar->serialBuffer[2] == 0x03))
			{
				// On vérifie le checksum
				checksum = 0;
				for (int i=0; i<5;i++)
					checksum += pLidar->serialBuffer[i];
				checksum &= 0xFF;
				checksum_ref = pLidar->serialBuffer[5];
				if(checksum == checksum_ref)
				{
					// La trame est correcte, on traite les données
					// On récupère le Frame Rate
					pLidar->framerate = pLidar->serialBuffer[3] + (pLidar->serialBuffer[4] << 8);

					pLidar->semaphore++;
				}
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x05) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x65))
			{
				// On récupère le Format des données
				pLidar->outputFormat = standard_cm;
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x05) &&
					(pLidar->serialBuffer[3] == 0x02) &&
					(pLidar->serialBuffer[4] == 0x66))
			{
				// On récupère le Format des données
				pLidar->outputFormat = pixhawk;
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x05) &&
					(pLidar->serialBuffer[3] == 0x03) &&
					(pLidar->serialBuffer[4] == 0x67))
			{
				// On récupère le Format des données
				pLidar->outputFormat = standard_mm;
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x08) &&
					(pLidar->serialBuffer[2] == 0x06))
			{
				// On vérifie le checksum
				checksum = 0;
				for (int i=0; i<7;i++)
					checksum += pLidar->serialBuffer[i];
				checksum &= 0xFF;
				checksum_ref = pLidar->serialBuffer[7];
				if(checksum == checksum_ref)
				{
					// La trame est correcte, on traite les données
					// On récupère le Baud Rate
					pLidar->baudrate = pLidar->serialBuffer[3] + (pLidar->serialBuffer[4] << 8) +
							(pLidar->serialBuffer[5] << 16) + (pLidar->serialBuffer[6] << 24);
					pLidar->semaphore++;
				}
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x07) &&
					(pLidar->serialBuffer[3] == 0x00) &&
					(pLidar->serialBuffer[4] == 0x66))
			{
				// Arrête la génération automatique des distances
				// Les distances ne sont fournies que sur demande
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x07) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x67))
			{
				// Démarre la génération automatique des distances
				// Les distances sont fournies régulièrement. La fréquence est fournie par le Frame Rate
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x10) &&
					(pLidar->serialBuffer[3] == 0x00) &&
					(pLidar->serialBuffer[4] == 0x6E))
			{
				// La demande de restauration des paramètres d'usine est acceptée
				pLidar->baudrate = 115200;
				pLidar->framerate = 100;
				pLidar->outputFormat = standard_cm;
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x10) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x6F))
			{
				// La demande de restauration des paramètres d'usine est refusée
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x22) &&
					(pLidar->serialBuffer[3] == 0x00) &&
					(pLidar->serialBuffer[4] == 0x6F))
			{
				// La demande de sauvegarde des parametres courant est acceptée
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x22) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x70))
			{
				// La demande de sauvegarde des parametres courant est refusée
				pLidar->semaphore++;
			}
		}
		// Sinon, c'est un format inconnu. On jète la trame.
	}
}

int tfminiplus_getLastAcquisition(int a_numCapteur, int *a_pDistance, int *a_pStrength, int *a_pTemperature)
{
	int erreur;

	if(a_numCapteur == MINILIDAR_DROIT)
	{
		// On renvoie la distance mesurée par le premier capteur
		if((miniLidarDroit.strength >= 100) && (miniLidarDroit.strength != 65535))
		{
			*a_pDistance = miniLidarDroit.distance;
			*a_pStrength = miniLidarDroit.strength;
			*a_pTemperature = miniLidarDroit.temperature;
		}
		else
		{
			*a_pDistance = -1;
		}
		erreur = 0;

		// Réinitialisation du strength pour détecter lorsque le lidar arrête d'envoyer des valeurs valides
		// En gros, le lidar envoie des captures toutes les 10 ms.
		// La valeur du strength du rayon de retour permet de savoir si la mesure est valide.
		// On peut avoir strength 250 250 10 10 10 10 10 10
		// Si le logiciel applicatif prend la mesure après le deuxième 250, et qu'il prend la deuxième mesure après le 4ieme 10,
		// alors le driver pourrait renvoyer la dernière valeur valide, mais elle est très ancienne.
		miniLidarDroit.strength = 0;
	}
	else if(a_numCapteur == MINILIDAR_GAUCHE)
	{
		// On renvoie la distance mesurée par le premier capteur
		if((miniLidarGauche.strength >= 100) && (miniLidarGauche.strength != 65535))
		{
			*a_pDistance = miniLidarGauche.distance;
			*a_pStrength = miniLidarGauche.strength;
			*a_pTemperature = miniLidarGauche.temperature;
		}
		else
		{
			*a_pDistance = -1;
		}
		erreur = 0;
		// Réinitialisation du strength pour détecter lorsque le lidar arrête d'envoyer des valeurs valides
		// En gros, le lidar envoie des captures toutes les 10 ms.
		// La valeur du strength du rayon de retour permet de savoir si la mesure est valide.
		// On peut avoir strength 250 250 10 10 10 10 10 10
		// Si le logiciel applicatif prend la mesure après le deuxième 250, et qu'il prend la deuxième mesure après le 4ieme 10,
		// alors le driver pourrait renvoyer la dernière valeur valide, mais elle est très ancienne.
		miniLidarGauche.strength = 0;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_setOutputFormat(int a_numCapteur, eLidarOutputFormat *a_pFormat)
{
	int erreur;
	int attente;
	uint8_t commande[8];

	if(a_numCapteur == 1)
	{
		if(*a_pFormat == standard_cm)
		{
			// Configure le format de sortie
			commande[0] = 0x5A; commande[1] = 0x05;
			commande[2] = 0x05; commande[3] = 0x01;
			commande[4] = 0x65;
		}
		if(*a_pFormat == pixhawk)
		{
			// Configure le format de sortie
			commande[0] = 0x5A; commande[1] = 0x05;
			commande[2] = 0x05; commande[3] = 0x02;
			commande[4] = 0x66;
		}
		if(*a_pFormat == standard_mm)
		{
			// Configure le format de sortie
			commande[0] = 0x5A; commande[1] = 0x05;
			commande[2] = 0x05; commande[3] = 0x06;
			commande[4] = 0x6A;
		}
		HAL_UART_Transmit(miniLidarDroit.pHuart, commande, 5, 100000);

		// Attente de la réponse
		miniLidarDroit.semaphore = 0;
		attente = 0;
		do {
			attente++;
			HAL_Delay(1);
		} while((miniLidarDroit.semaphore==0) && (attente<TIMEOUT));

		if(attente < TIMEOUT)
		{
			// Verification que le nouveau format est celui demandé
			if(*a_pFormat == miniLidarDroit.outputFormat)
				erreur = 0;
			else
			{
				*a_pFormat = miniLidarDroit.outputFormat;
				erreur = -1;
			}
		}
		else
			// Timeout
			erreur = -1;
	}
	else if(a_numCapteur == 2)
	{
		if(*a_pFormat == standard_cm)
		{
			// Configure le format de sortie
			commande[0] = 0x5A; commande[1] = 0x05;
			commande[2] = 0x05; commande[3] = 0x01;
			commande[4] = 0x65;
		}
		if(*a_pFormat == pixhawk)
		{
			// Configure le format de sortie
			commande[0] = 0x5A; commande[1] = 0x05;
			commande[2] = 0x05; commande[3] = 0x02;
			commande[4] = 0x66;
		}
		if(*a_pFormat == standard_mm)
		{
			// Configure le format de sortie
			commande[0] = 0x5A; commande[1] = 0x05;
			commande[2] = 0x05; commande[3] = 0x06;
			commande[4] = 0x6A;
		}
		HAL_UART_Transmit(miniLidarGauche.pHuart, commande, 5, 100000);

		// Attente de la réponse
		miniLidarGauche.semaphore = 0;
		attente = 0;
		do {
			attente++;
			HAL_Delay(1);
		} while((miniLidarGauche.semaphore==0) && (attente<TIMEOUT));

		if(attente < TIMEOUT)
		{
			// Verification que le nouveau format est celui demandé
			if(*a_pFormat == miniLidarGauche.outputFormat)
				erreur = 0;
			else
			{
				*a_pFormat = miniLidarGauche.outputFormat;
				erreur = -1;
			}
		}
		else
			// Timeout
			erreur = -1;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_getFramerate(int a_numCapteur, int *a_pFramerate)
{
	int erreur;

	if(a_numCapteur == 1)
	{
		*a_pFramerate = miniLidarDroit.framerate;
		erreur = 0;
	}
	else if(a_numCapteur == 2)
	{
		*a_pFramerate = miniLidarGauche.framerate;
		erreur = 0;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_getBaudrate(int a_numCapteur, int *a_pBaudrate)
{
	int erreur;

	if(a_numCapteur == 1)
	{
		*a_pBaudrate = miniLidarDroit.baudrate;
		erreur = 0;
	}
	else if(a_numCapteur == 2)
	{
		*a_pBaudrate = miniLidarGauche.baudrate;
		erreur = 0;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_getVersion(int a_numCapteur, int *a_pVersion)
{
	int erreur;
	int attente;
	uint8_t commande[8];

	if(a_numCapteur == 1)
	{
		// Demande de la version au capteur
		commande[0] = 0x5A; commande[1] = 0x04; commande[2] = 0x01; commande[3] = 0x5F;
		HAL_UART_Transmit(miniLidarDroit.pHuart, commande, 4, 100000);

		// Attente de la réponse
		miniLidarDroit.semaphore = 0;
		attente = 0;
		do {
			attente++;
			HAL_Delay(1);
		} while((miniLidarDroit.semaphore==0) && (attente<TIMEOUT));

		if(attente < TIMEOUT)
		{
			// Fourniture de la version
			*a_pVersion = miniLidarDroit.version;

			erreur = 0;
		}
		else
			erreur = -1;
	}
	else if(a_numCapteur == 2)
	{
		// Demande de la version au capteur
		commande[0] = 0x5A; commande[1] = 0x04; commande[2] = 0x01; commande[3] = 0x5F;
		HAL_UART_Transmit(miniLidarGauche.pHuart, commande, 4, 100000);

		// Attente de la réponse
		miniLidarGauche.semaphore = 0;
		attente = 0;
		do {
			attente++;
			HAL_Delay(1);
		} while((miniLidarGauche.semaphore==0) && (attente<TIMEOUT));

		if(attente < TIMEOUT)
		{
			// Fourniture de la version
			*a_pVersion = miniLidarGauche.version;

			erreur = 0;
		}
		else
			erreur = -1;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_init()
{
	HAL_StatusTypeDef retour;

	// Initialisation d'un capteur. Il sera facile d'en initialiser un deuxième

	// Initialisation des parametres
	miniLidarDroit.pHuart = &huart3;
	miniLidarDroit.distance = -1;
	miniLidarDroit.strength = 0;
	miniLidarDroit.temperature = 0;
	miniLidarDroit.baudrate = 115200;
	miniLidarDroit.framerate = 100;
	miniLidarDroit.semaphore = 0;

	miniLidarGauche.pHuart = &huart5;
	miniLidarGauche.distance = -1;
	miniLidarGauche.strength = 0;
	miniLidarGauche.temperature = 0;
	miniLidarGauche.baudrate = 115200;
	miniLidarGauche.framerate = 100;
	miniLidarGauche.semaphore = 0;

	// Début d'écoute
	retour = HAL_UART_Receive_DMA(miniLidarGauche.pHuart, miniLidarGauche.serialBuffer, 9);
	retour = HAL_UART_Receive_DMA(miniLidarDroit.pHuart, miniLidarDroit.serialBuffer, 9);

	return retour;
}


