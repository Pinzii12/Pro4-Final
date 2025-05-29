#include "app_main.h"

void app_main() {


	initEncoder();

	osKernelInitialize();

	//userInitBMM350();
	userInitBMI088();
	HAL_GPIO_WritePin(LED_Error3_GPIO_Port, LED_Error3_Pin, GPIO_PIN_SET);
	initSensorFusion();
	HAL_GPIO_WritePin(LED_Error2_GPIO_Port, LED_Error2_Pin, GPIO_PIN_SET);
	initMavlink();
	initPIDController();
	initMotorControl();

	// Starte den FreeRTOS-Scheduler
	osKernelStart();

	// Sollte nie erreicht werden
	while (1) {
	}

}
