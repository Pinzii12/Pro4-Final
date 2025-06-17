#ifndef __MAGCALIBRATION_H
#define __MAGCALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file           : magCalibration.h
 * @brief          : Beinhaltetet die Offsets, welche per Python-Skript gewonnen
 * 					 werden.
 ******************************************************************************
 * @author Ralf Pianzola
 * @date 20.04.2025
 * @version 1.0
 *
 ******************************************************************************
 */

/**
 * @brief   Hard-Iron-Offset-Korrektur für den BMM350.
 * @details Der Offset wird mittels Python-Programm ermittelt und nachher als
 * Array abgespeichert.
 * @note    Werte in uT [x,y,z]
 */
static const float magOffset[3] = { 18.0f, 20.0f, -6.0f };

/**
 * @brief   Soft-Iron-Korrekturmatrix für den BMM350.
 * @details Der Offset wird mittels Python-Programm ermittelt und nachher als
 * Array abgespeichert.
 */
static const float magSoftIron[3][3] = { { 1.017984f, 0.0f, 0.0f }, { 0.0f,
		0.977681f, 0.0f }, { 0.0f, 0.0f, 1.005189f } };

#ifdef __cplusplus
}
#endif

#endif /* __MAGCALIBRATION_H */
