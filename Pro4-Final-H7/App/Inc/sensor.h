#ifndef __sensor_H
#define __sensor_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file           : sensor.h
 * @brief          : Beschreibt die Schnittstellen zur Interaktion mit dem
 * 					 Madgwick-Filter.
 ******************************************************************************
 * @author Ralf Pianzola
 * @date 29.04.2025
 * @version 1.0
 *
 ******************************************************************************
 */

/* Includes -----------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "main.h"
#include "fusion.h"
#include "bmm350.h"
#include "cmsis_os.h"
#include "bmi08.h"
#include <string.h>
#include <stdio.h>
#include "magCalibration.h"

/* Defines ------------------------------------------------------------------*/
#define BMM350_CALIBRATION_ACTIVE 0
#define QUATERNION_OUTPUT_ACTIVE 0
#define USE_MAGNETOEMTER 1

#define BMM350_I2C_ADDR  0x14
#define BMM350_MAG_DATA_LEN 11
#define flagIntReceived 0x01

/* Conversion-Rates ---------------------------------------------------------*/
static const float BMI088aConversion = 6.0f / 32768.0f;
static const float BMI088gConversion = 500.0f / 32768.0f;

/* Structs ------------------------------------------------------------------*/

/**
 * @brief Struktur zum Speichern der pyhsikalischen Messwerte.
 *
 * Enzhält alle Messwerte, welche nachher dem Madgwick-Filter übergeben
 * werden.
 */
typedef struct {
	FusionVector gyro; /**< Gyroskop-Daten [rad/s] */
	FusionVector accel; /**< Beschleunigungsdaten [m/s^2] */
	FusionVector mag; /**< Magnetometer-Daten [µT] */
} SensorData_t;

/**
 * @brief Struktur zum Speichern der SPI-Kommunikation des BMI088s.
 * Enthält den SPI-Port und die Chip-Select vom Accelerometer und Gyroskop.
 * Wird für die Bosch-API benötigt.
 */
typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} bmi088_interface_t;

/* Functions ----------------------------------------------------------------*/

/**
 * @brief   Startet die RTOS-Tasks der ganzen Sensoren und initalisiert den Timer.
 * @return  None
 */
void initSensorFusion(void);

/**
 * @brief   Initialisiert und konfiguriert den BMM350 Magnetometer.
 * @return  None
 */
void userInitBMM350();

/**
 * @brief   Initialisiert und konfiguriert den BMI088 IMU.
 * @return  None
 */
void userInitBMI088();

#ifdef __cplusplus
}
#endif

#endif /* __sensor_H */

