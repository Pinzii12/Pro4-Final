#include "app_main.h"

void app_main() {

	initEncoder();
	osKernelInitialize();
	userInitBMM350();
	userInitBMI088();
	initSensorFusion();
	initMavlink();
	initPIDController();
	initMotorControl();

	// Starte den FreeRTOS-Scheduler
	osKernelStart();

	// Sollte nie erreicht werden
	while (1) {
	}

}
