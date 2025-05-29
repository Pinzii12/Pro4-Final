#ifndef __motorControl_H
#define __motorControl_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file           : motorControl.h
 * @brief          : Beschreibt die Schnittstellen der Motorsteuerung.
 ******************************************************************************
 * @author Ralf Pianzola
 * @date 06.05.2025
 * @version 1.0
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "AS5048A.h"
#include "PID.h"
#include "PID_Config.h"
#include "cmsis_os.h"
#include "main.h"
#include "quaternion.h"
#include "sensor.h"
#include "sinLookupTable.h"
#include "stm32h7xx_hal.h"

/* Defines für die Motorsteuerung --------------------------------------------*/
#define ENCODER_RESOLUTION (1<<14)
#define NUM_POLEPAIRS 7
#define PWM_MAX_VAL 1200

/* Defines fürs RTOS ---------------------------------------------------------*/
#define flagIntReceived 0x01

/**
 * @brief Initialisiert die Encoder.
 */
void initEncoder(void);

/**
 * @brief Initialisiert die PID-Regler für Roll, Pitch und Yaw.
 *
 * Setzt die Parameter gemäss den Werten in der PID_Config.h
 */
void initPIDController(void);

/**
 * @brief Initialisiert die Motorsteuerung des Gimbal-Systems.
 *
 * Diese Funktion aktiviert:
 * - Die Enable-Pins für den BLDC-Driver,
 * - Die PWM-Kanäle für die BLDC-Ansteuerung
 * - Den FreeRTOS-Timer
 * - Das RTOS-Flag
 * - Die RTOS-Tasks
 */
void initMotorControl(void);

#ifdef __cplusplus
}
#endif

#endif /* __motorControl_H */
