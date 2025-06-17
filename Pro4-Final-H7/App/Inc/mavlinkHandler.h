#ifndef __MAVLINK_HANDLER_H
#define __MAVLINK_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file           : mavlinkHandler.h
 * @brief          : Beschreibt die Schnittstellen der Mavlink-Kommunikation.
 ******************************************************************************
 * @author Ralf Pianzola
 * @date 19.04.2025
 * @version 1.0
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "mavlink.h"

#define MAVLINK_RX_BUFFER_SIZE 512
#define HEARTBEAT_INTERVAL 1000
#define MAVLINK_SYSTEM_ID 2
#define flagIntReceived 0x01

/**
 * @brief Datenstruktur für eine empfangene MAVLink-Nachricht.
 */
typedef struct {
    uint8_t data[MAVLINK_RX_BUFFER_SIZE];
    uint16_t length;
} mavlinkData_t;

/**
 * @brief Struktur fürs Quaternion der gewünschten Ausrichtung.
 */
typedef struct {
    float q[4];
} gimbalAttitude_t;


/* Funktion deklarieren */

/**
 * @brief Startet die Mavlink-Kommunikation
 * Es werden alle RTOS-Task initalisiert und gestartet.
 */
void initMavlink(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_HANDLER_H */
