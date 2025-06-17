#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file           : app_main.h
 * @brief          : Beschreibt die Schnittstellen der Hauptanwendung.
 * 					 Leitet die main.c in die app_main.cpp um, sodass eine
 * 					 Änderung im .ioc-File nicht jedes Mal händisch in eine
 * 					 .cpp Datei geändert werden muss.
 ******************************************************************************
 * @author Ralf Pianzola
 * @date 06.05.2025
 * @version 1.0
 *
 ******************************************************************************
 */

#include "stm32h7xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "sensor.h"
#include <mavlinkHandler.h>
#include "motorControl.h"

void app_main();

#ifdef __cplusplus
}
#endif

#endif /* __APP_MAIN_H */
