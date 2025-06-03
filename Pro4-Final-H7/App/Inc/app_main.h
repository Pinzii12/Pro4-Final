#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

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
