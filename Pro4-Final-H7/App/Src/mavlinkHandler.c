/**
 ******************************************************************************
 * @file           : mavlinkHandler.c
 * @brief          : Implementiert die Kommunikation mittels MAVLink.
 ******************************************************************************
 * @author Ralf Pianzola
 * @date 19.04.2025
 * @version 1.0
 ******************************************************************************
 */

#include <mavlinkHandler.h>

/* external defines ----------------------------------------------------------*/
extern UART_HandleTypeDef huart6;

/* Private variables ---------------------------------------------------------*/
static uint8_t rxBuffer[MAVLINK_RX_BUFFER_SIZE];

/* Pre-Declarations ----------------------------------------------------------*/
void sendHeartbeat(void);
int8_t parseMavlinkMessage(mavlinkData_t *mavlinkMessage, mavlink_message_t *msg);
void decodeMavlinkMessage(mavlink_message_t *msg);
void handleGimbalAttitudeMsg(mavlink_gimbal_device_set_attitude_t *gimbal_attitude);

/* Free-RTOS -----------------------------------------------------------------*/
osEventFlagsId_t heartbeatFlag;
osEventFlagsId_t mavlinkMessageReceivedFlag;
osTimerId_t heartbeatTimerHandle;
osMessageQueueId_t mavlinkQueue;
osMessageQueueId_t qTargetQueue;

static const osThreadAttr_t sendHeartbeatTask_attributes = { .name =
		"sendHeartbeatTask", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 2048 };

static const osThreadAttr_t processMavlinkMessageTask_attributes = { .name =
		"processMavlinkMessageTask",
		.priority = (osPriority_t) osPriorityNormal, .stack_size = 4096 };

static const osTimerAttr_t heartbeatTimer_attributes = { .name =
		"heartbeatTimer" };

/**
 * @brief Sendet einen Heartbeat
 * Wird alle 1s vom Timer heartbeatTimer aufgerufen.
 */
void sendHeartbeatTask(void *argument) {
	for (;;) {
		osEventFlagsWait(heartbeatFlag, flagIntReceived, osFlagsWaitAny,
		osWaitForever);
		sendHeartbeat();
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

	}
}

/**
 * @brief Verarbeitet eine empfange Nachricht.
 * Sobald eine Nachricht in der Queue empfangen wird, wird diese dekodiert und verarbeitet.
 */
void processMavlinkMessageTask(void *argument) {
	mavlinkData_t msgReceived;
	mavlink_message_t msg;

	for (;;) {
		if (osMessageQueueGet(mavlinkQueue, &msgReceived, NULL, osWaitForever)
				== osOK) {
			if (parseMavlinkMessage(&msgReceived, &msg) == 0) {
				decodeMavlinkMessage(&msg);
			}
		}
	}
}

/**
 * @brief Setzt das Flag zum Senden des Heartbeats
 */
void heartbeatTimerCallback(void *argument) {
	osEventFlagsSet(heartbeatFlag, flagIntReceived);
}

/**
 * @brief Startet die Mavlink-Kommunikation
 * Es werden alle RTOS-Task initalisiert und gestartet.
 */
void initMavlink(void) {
	heartbeatFlag = osEventFlagsNew(NULL);
	mavlinkMessageReceivedFlag = osEventFlagsNew(NULL);

	mavlinkQueue = osMessageQueueNew(10, sizeof(mavlinkData_t), NULL);

	osThreadNew(sendHeartbeatTask, NULL, &sendHeartbeatTask_attributes);
	osThreadNew(processMavlinkMessageTask, NULL,
			&processMavlinkMessageTask_attributes);

	heartbeatTimerHandle = osTimerNew(heartbeatTimerCallback, osTimerPeriodic,
	NULL, &heartbeatTimer_attributes);

	osTimerStart(heartbeatTimerHandle, HEARTBEAT_INTERVAL);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rxBuffer, MAVLINK_RX_BUFFER_SIZE);

	qTargetQueue = osMessageQueueNew(10, sizeof(float[4]), NULL);
}

/* Callbacks -----------------------------------------------------------------*/

/**
 * @brief Sendet die empfangenen Daten in die Queue und startet den DMA-Empfang wieder.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART6) {
		mavlinkData_t msg;
		memcpy(msg.data, rxBuffer, Size);
		msg.length = Size;

		osMessageQueuePut(mavlinkQueue, &msg, 0, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rxBuffer,
		MAVLINK_RX_BUFFER_SIZE);
	}
}

/* Mavlink-Verarbeitung ------------------------------------------------------*/

/**
 * @brief Sendet einen Heartbeat.
 */
void sendHeartbeat(void) {
	mavlink_message_t msg;
	static uint8_t buffer[10];

	mavlink_msg_heartbeat_pack(
	MAVLINK_SYSTEM_ID, MAV_COMP_ID_GIMBAL, &msg, MAV_TYPE_GIMBAL,
			MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);

	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
	HAL_UART_Transmit(&huart6, buffer, len, HAL_MAX_DELAY);
}

/**
 * @brief Verarbeitet die UART-Bytes in eine gültige Mavlink-Message.
 * @param *mavlinkMessage Zeiger auf die empfangen Daten.
 * @retval 0 -> Decodierung erfolgreich
 * @retval -1 -> Fehler
 */
int8_t parseMavlinkMessage(mavlinkData_t *mavlinkMessage,
		mavlink_message_t *msg) {
	mavlink_status_t status;

	for (int i = 0; i < mavlinkMessage->length; i++) {
		if (mavlink_parse_char(MAVLINK_COMM_0, mavlinkMessage->data[i], msg,
				&status)) {
			return 0;
		}
	}
	return -1;
}

/**
 * @brief Verabreitet die Gimba-Attitude Message.
 * Sendet die Ausrichtung per RTOS-Queue ans Ziel-Quaternion.
 * @param gimbal_attitude Zeiger auf die empfangene Mavlink Message.
 */
void handleGimbalAttitudeMsg(
		mavlink_gimbal_device_set_attitude_t *gimbal_attitude) {
	float qTarget[4];

	qTarget[0] = gimbal_attitude->q[0];
	qTarget[1] = gimbal_attitude->q[1];
	qTarget[2] = gimbal_attitude->q[2];
	qTarget[3] = gimbal_attitude->q[3];

	osMessageQueuePut(qTargetQueue, &qTarget, 0, 0);
}

/**
 * @brief Decodiert eine Mavlink-Message.
 * Decodiert eine Mavlink-Message und verarbietet diese entsprechend weiter.
 * @param msg Zeiger auf die empfangene Mavlink Message.
 */
void decodeMavlinkMessage(mavlink_message_t *msg) {
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT: {
		mavlink_heartbeat_t heartbeat;
		mavlink_msg_heartbeat_decode(msg, &heartbeat);
		//TODO: hier noch eine Kom. LED ansteuern
		break;
	}
	case MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE: {
		mavlink_gimbal_device_set_attitude_t gimbal_attitude;
		mavlink_msg_gimbal_device_set_attitude_decode(msg, &gimbal_attitude);
		handleGimbalAttitudeMsg(&gimbal_attitude);
		break;
	}
	default:
		break;
	}

}

