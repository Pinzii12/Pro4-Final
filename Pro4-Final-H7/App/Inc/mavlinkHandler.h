#ifndef __MAVLINK_HANDLER_H
#define __MAVLINK_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "mavlink.h"

#define MAVLINK_RX_BUFFER_SIZE 512
#define HEARTBEAT_INTERVAL 1000
#define MAVLINK_SYSTEM_ID 2
#define flagIntReceived 0x01

typedef struct {
    uint8_t data[MAVLINK_RX_BUFFER_SIZE];
    uint16_t length;
} mavlinkData_t;

typedef struct {
    float q[4];
} gimbalAttitude_t;


/* Funktion deklarieren */
void initMavlink(void);
void sendHeartbeat(void);
int8_t parseMavlinkMessage(mavlinkData_t*, mavlink_message_t*);
void decodeMavlinkMessage(mavlink_message_t*);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_HANDLER_H */
