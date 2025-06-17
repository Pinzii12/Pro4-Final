/**
 ******************************************************************************
 * @file           : sensor.c
 * @brief          : Konfiguriert die IMU + Magnetometer und startet den
 * 					 Madgwick-Filter.
 ******************************************************************************
 * @author Ralf Pianzola
 * @date 29.04.2025
 * @version 1.0
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"

/* Externs  ------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;

/* Structs -------------------------------------------------------------------*/
struct bmm350_dev bmm350;
struct bmi08_dev bmi088;
struct bmm350_mag_temp_data rawMag;
struct bmm350_mag_temp_data rawMagData;
struct bmi08_sensor_data rawAcc;
struct bmi08_sensor_data rawAccData;
struct bmi08_sensor_data rawGyro;
struct bmi08_sensor_data rawGyroData;
struct bmi08_accel_int_channel_cfg accel_int_cfg;
struct bmi08_gyro_int_channel_cfg gyro_int_cfg;

bmi088_interface_t acc_iface;
bmi088_interface_t gyro_iface;
SensorData_t sensorData;
FusionAhrs madgwickFilter;

/* Declarations --------------------------------------------------------------*/
static uint8_t bmm350_i2c_addr = BMM350_I2C_ADDR;
static uint32_t previousMadgwickTick = 0;

/* Free-RTOS -----------------------------------------------------------------*/
osEventFlagsId_t bmm350IntFlag;
osEventFlagsId_t bmi088aIntFlag;
osEventFlagsId_t bmi088gIntFlag;
osEventFlagsId_t madgwickTimerFlag;

osMessageQueueId_t bmi088aQueue;
osMessageQueueId_t bmi088gQueue;
osMessageQueueId_t bmm350Queue;
osMessageQueueId_t qCurrentQueue;

static osThreadId_t interruptReadBMI088aTaskHandle;
static const osThreadAttr_t interruptReadBMI088aTask_attributes = { .name =
		"interruptReadBMI088a", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 2048 };

static osThreadId_t interruptReadBMI088gTaskHandle;
static const osThreadAttr_t interruptReadBMI088gTask_attributes = { .name =
		"interruptReadBMI088g", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 2048 };

static osThreadId_t interruptReadBMM350TaskHandle;
static const osThreadAttr_t interruptReadBMM350Task_attributes = { .name =
		"interruptReadBMM350", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 2048 };

static osThreadId_t processBMI088aDataTaskHandle;
static const osThreadAttr_t processBMI088aDataTask_attributes = { .name =
		"processBMI088aData", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 2048 };

static osThreadId_t processBMI088gDataTaskHandle;
static const osThreadAttr_t processBMI088gDataTask_attributes = { .name =
		"processBMI088gData", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 2048 };

static osThreadId_t processBMM350DataTaskHandle;
static const osThreadAttr_t processBMM350DataTask_attributes = { .name =
		"processBMM350Data", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 2048 };

static osThreadId_t madgwickTaskHandle;
static const osThreadAttr_t madgwickTask_attributes = { .name = "madgwickTask",
		.priority = (osPriority_t) osPriorityNormal, .stack_size = 2048 };

#if QUATERNION_OUTPUT_ACTIVE
static osThreadId_t sendQuaternionTaskHandle;
static const osThreadAttr_t sendQuaternionTask_attributes = { .name =
		"sendQuaternionTask", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 4096, };
#endif

#if BMM350_CALIBRATION_ACTIVE
static osThreadId_t sendMagnetometerDataTaskHandle;
static const osThreadAttr_t sendMagnetometerDataTask_attributes = { .name =
		"sendMagnetometerDataTask", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 4096, };
#endif

osTimerId_t madgwickTimerHandle;
static const osTimerAttr_t madgwickTimer_attributes =
		{ .name = "madgwickTimer" };

/* Free-RTOS Accelerometer-Tasks ---------------------------------------------*/

/**
 * @brief   RTOS-Task zum Auslesen des Accerlerometers.
 *          Wird durch ein Flag aktiviert und startet den Datenabruf.
 * @return  None
 */
void interruptReadBMI088a(void *argument) {

	for (;;) {
		osEventFlagsWait(bmi088aIntFlag, flagIntReceived, osFlagsWaitAny,
		osWaitForever);
		if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) {
			bmi08a_get_data(&rawAcc, &bmi088);
			osMessageQueuePut(bmi088aQueue, &rawAcc, 0, 0);
		}

	}

}

/**
 * @brief   RTOS-Task zum Verarbeiten und speichern der Accelerometerdaten.
 *          Wird durch die Queue aktiviert und speichert die Daten in m/s^2 als aktuelle
 *          Beschleunigungswerte.
 * @return  None
 */
void processBMI088aData(void *argument) {

	for (;;) {
		if (osMessageQueueGet(bmi088aQueue, &rawAccData, NULL, osWaitForever)
				== osOK) {
			float ax = (float) rawAccData.x * BMI088aConversion;
			float ay = (float) rawAccData.y * BMI088aConversion;
			float az = (float) rawAccData.z * BMI088aConversion;

			taskENTER_CRITICAL();
			sensorData.accel.axis.x = ax;
			sensorData.accel.axis.y = ay;
			sensorData.accel.axis.z = az;
			taskEXIT_CRITICAL();
		}
	}
}

/* Free-RTOS Gyroskop-Tasks --------------------------------------------------*/

/**
 * @brief   RTOS-Task zum Auslesen des Gyroskops.
 *          Wird durch ein Flag aktiviert und startet den Datenabruf.
 * @return  None
 */
void interruptReadBMI088g(void *argument) {

	for (;;) {
		osEventFlagsWait(bmi088gIntFlag, flagIntReceived, osFlagsWaitAny,
		osWaitForever);
		if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) {
			bmi08g_get_data(&rawGyro, &bmi088);
			osMessageQueuePut(bmi088gQueue, &rawGyro, 0, 0);
		}

	}

}

/**
 * @brief   RTOS-Task zum Verarbeiten und speichern der Daten des Gyroskops.
 *          Wird durch die Queue aktiviert und speichert die Daten in °/s als aktuelle
 *          Winkelbeschleunigungswerte.
 * @return  None
 */
void processBMI088gData(void *argument) {

	for (;;) {
		if (osMessageQueueGet(bmi088gQueue, &rawGyroData, NULL, osWaitForever)
				== osOK) {

			float gx = (float) rawGyroData.x * BMI088gConversion;
			float gy = (float) rawGyroData.y * BMI088gConversion;
			float gz = (float) rawGyroData.z * BMI088gConversion;

			taskENTER_CRITICAL();
			sensorData.gyro.axis.x = gx;
			sensorData.gyro.axis.y = gy;
			sensorData.gyro.axis.z = gz;
			taskEXIT_CRITICAL();
		}
	}
}

/* Free-RTOS Magnetometer-Tasks ----------------------------------------------*/

/**
 * @brief   RTOS-Task zum Auslesen des Magnetometers.
 *          Wird durch ein Flag aktiviert und startet den Datenabruf.
 * @return  None
 */
void interruptReadBMM350(void *argument) {

	for (;;) {
		osEventFlagsWait(bmm350IntFlag, flagIntReceived, osFlagsWaitAny,
		osWaitForever);
		if (HAL_I2C_GetState(&hi2c3) == HAL_I2C_STATE_READY) {
			bmm350_get_compensated_mag_xyz_temp_data(&rawMag, &bmm350);
			osMessageQueuePut(bmm350Queue, &rawMag, 0, 0);
		}

	}

}


/**
 * @brief   RTOS-Task zum Verarbeiten und speichern der Daten des Magnetometers.
 *          Wird durch die Queue aktiviert und speichert die Daten in uT als aktuelles
 *          Magnetfeld.
 * @return  None
 */
void processBMM350Data(void *argument) {

	for (;;) {
		if (osMessageQueueGet(bmm350Queue, &rawMagData, NULL, osWaitForever)
				== osOK) {

#if BMM350_CALIBRATION_ACTIVE
			float x = rawMagData.x;
			float y = rawMagData.y;
			float z = rawMagData.z;
#else
			float mx = rawMagData.x - magOffset[0];
			float my = rawMagData.y - magOffset[1];
			float mz = rawMagData.z - magOffset[2];

			float x = magSoftIron[0][0] * mx;
			float y = magSoftIron[1][1] * my;
			float z = magSoftIron[2][2] * mz;
#endif
			taskENTER_CRITICAL();
			sensorData.mag.axis.x = x;
			sensorData.mag.axis.y = y;
			sensorData.mag.axis.z = z;
			taskEXIT_CRITICAL();
		}
	}
}

/* Free-RTOS Madgwick Task ---------------------------------------------------*/

/**
 * @brief   RTOS-Task zum Auslösen des MAdgwick-Filters.
 * @return  None
 */
void madgwickTimerCallback(void *argument) {
	osEventFlagsSet(madgwickTimerFlag, flagIntReceived);
}

/**
 * @brief   RTOS-Task des Madgwick-Filters. Wird durch einen Timer periodisch ausgelöst.
 * 			Abhängig davo, ob Magnetometer genutzt wird oder nicht, wird eine andere Funktion aufgerufen.
 * 			Das erhaltene Quaternion wird an die Motorsteuerung (motorControl) weitergegeben.
 * @return  None
 */
void madgwickTask(void *argument) {
	for (;;) {
		osEventFlagsWait(madgwickTimerFlag, flagIntReceived, osFlagsWaitAny,
		osWaitForever);

		FusionVector accelData;
		FusionVector gyroData;
		FusionVector magData;

		taskENTER_CRITICAL();
		accelData = sensorData.accel;
		gyroData = sensorData.gyro;
		magData = sensorData.mag;
		taskEXIT_CRITICAL();

		uint32_t currentTick = osKernelGetTickCount();
		float deltaTime = (currentTick - previousMadgwickTick) / 1000.0f;
		previousMadgwickTick = currentTick;

#if USE_MAGNETOEMTER
		FusionAhrsUpdate(&madgwickFilter, gyroData, accelData, magData,
				deltaTime);
#else
		 FusionAhrsUpdateNoMagnetometer(&madgwickFilter, gyroData, accelData,
		 deltaTime);
#endif

		 FusionQuaternion quaternion;
		 float qCurrent[4];
		 taskENTER_CRITICAL();
		 quaternion = FusionAhrsGetQuaternion(&madgwickFilter);
		 taskEXIT_CRITICAL();

		 qCurrent[0] = quaternion.element.w;
		 qCurrent[1] = quaternion.element.x;
		 qCurrent[2] = quaternion.element.y;
		 qCurrent[3] = quaternion.element.z;

		 osMessageQueuePut(qCurrentQueue, &qCurrent, 0, 0);

	}
}

/**
 * @brief   RTOS-Task, welcher ein Quaternion an die UART-Schnitstelle sendet. Kann z.B. für die Visualisierung genutzt werden.
 * @note	https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/webserial-visualizer
 * @return  None
 */
void sendQuaternionTask(void *argument) {
	for (;;) {
		FusionQuaternion quaternion;

		taskENTER_CRITICAL();
		quaternion = FusionAhrsGetQuaternion(&madgwickFilter);
		taskEXIT_CRITICAL();

		char buffer[128];
		int length = snprintf(buffer, sizeof(buffer),
				"Quaternion: %.2f, %.2f, %.2f, %.2f\r\n", quaternion.element.w,
				quaternion.element.x, quaternion.element.y,
				quaternion.element.z);

		if (length > 0) {
			HAL_UART_Transmit(&huart3, (uint8_t*) buffer, length,
			HAL_MAX_DELAY);

		}
		osDelay(10);
	}
}

/**
 * @brief   RTOS-Task, welcher die Daten des Magnetometers an die UART-Schnittstelle sendet. Wird für die Kalibrierung genutzt.
 * @return  None
 */
void sendMagnetometerDataTask(void *argument) {
	for (;;) {
		FusionVector magData;

		taskENTER_CRITICAL();
		magData = sensorData.mag;
		taskEXIT_CRITICAL();

		char buffer[64];
		int length = snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f\r\n",
				magData.axis.x, magData.axis.y, magData.axis.z);

		if (length > 0) {
			HAL_UART_Transmit(&huart3, (uint8_t*) buffer, length,
			HAL_MAX_DELAY);
		}

		osDelay(4);
	}
}

/**
 * @brief   Startet die RTOS-Tasks der ganzen Sensoren und initalisiert den Timer.
 * @return  None
 */
void initSensorFusion(void) {

	bmm350IntFlag = osEventFlagsNew(NULL);
	bmi088aIntFlag = osEventFlagsNew(NULL);
	bmi088gIntFlag = osEventFlagsNew(NULL);
	madgwickTimerFlag = osEventFlagsNew(NULL);

	bmi088aQueue = osMessageQueueNew(10, sizeof(struct bmi08_sensor_data),
	NULL);
	bmi088gQueue = osMessageQueueNew(10, sizeof(struct bmi08_sensor_data),
	NULL);
	bmm350Queue = osMessageQueueNew(10, sizeof(struct bmm350_mag_temp_data),
	NULL);
	qCurrentQueue = osMessageQueueNew(10, sizeof(float[4]), NULL);

	interruptReadBMI088aTaskHandle = osThreadNew(interruptReadBMI088a, NULL,
			&interruptReadBMI088aTask_attributes);
	interruptReadBMI088gTaskHandle = osThreadNew(interruptReadBMI088g, NULL,
			&interruptReadBMI088gTask_attributes);
	interruptReadBMM350TaskHandle = osThreadNew(interruptReadBMM350, NULL,
			&interruptReadBMM350Task_attributes);

	processBMI088aDataTaskHandle = osThreadNew(processBMI088aData, NULL,
			&processBMI088aDataTask_attributes);
	processBMI088gDataTaskHandle = osThreadNew(processBMI088gData, NULL,
			&processBMI088gDataTask_attributes);
	processBMM350DataTaskHandle = osThreadNew(processBMM350Data, NULL,
			&processBMM350DataTask_attributes);
	madgwickTaskHandle = osThreadNew(madgwickTask, NULL,
			&madgwickTask_attributes);

#if QUATERNION_OUTPUT_ACTIVE
	sendQuaternionTaskHandle = osThreadNew(sendQuaternionTask, NULL,
			&sendQuaternionTask_attributes);
#endif

#if BMM350_CALIBRATION_ACTIVE
	sendMagnetometerDataTaskHandle = osThreadNew(sendMagnetometerDataTask,
	NULL, &sendMagnetometerDataTask_attributes);
#endif

	madgwickTimerHandle = osTimerNew(madgwickTimerCallback, osTimerPeriodic,
	NULL, &madgwickTimer_attributes);

	FusionAhrsInitialise(&madgwickFilter);
	osTimerStart(madgwickTimerHandle, 3);

}

/* Interrupts / Callbacks ----------------------------------------------------*/

/**
 * @brief   Callback der Interrupts an den GPIO-Pins. Sendet ein Flag an den entsprechenden Task.
 * @param   GPIO_Pin: Der Pin, der den Interrupt ausgelöst hat.
 * @return  None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case INT1_ACC_Pin:
		osEventFlagsSet(bmi088aIntFlag, flagIntReceived);
		break;

	case INT4_GYRO_Pin:
		osEventFlagsSet(bmi088gIntFlag, flagIntReceived);
		break;

	case INT_MAG_Pin:
		osEventFlagsSet(bmm350IntFlag, flagIntReceived);
		break;

	default:
		return;
	}

}

/* Delay Config --------------------------------------------------------------*/

/**
 * @brief   Für die API wird ein us-Delay benötigt. Rundet die Befehle auf die nächste ms auf.
 * @param   period: Verzögerung in Mikrosekunden.
 * @param   intf_ptr: Nicht verwendet.
 * @return  None
 */
void user_delay_us(uint32_t period, void *intf_ptr) {
	HAL_Delay((period + 999) / 1000);
}

/* BMM350 Config -------------------------------------------------------------*/

/**
 * @brief   I2C-Read-Funktion für BMM350.
 * @param   reg_addr: Registeradresse.
 * @param   reg_data: Zeiger auf Zielpuffer.
 * @param   len: Anzahl der zu lesenden Bytes.
 * @param   intf_ptr: Zeiger auf I2C-Adresse.
 * @return  0 bei Erfolg, -1 bei Fehler.
 */
int8_t bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
		void *intf_ptr) {
	uint8_t dev_addr = *((uint8_t*) intf_ptr);
	if (HAL_I2C_Mem_Read(&hi2c3, dev_addr << 1, reg_addr,
	I2C_MEMADD_SIZE_8BIT, reg_data, len, HAL_MAX_DELAY) == HAL_OK)
		return 0;
	else
		return -1;
}

/**
 * @brief   I2C-Write-Funktion für BMM350.
 * @param   reg_addr: Registeradresse.
 * @param   reg_data: Zeiger auf Datenpuffer.
 * @param   len: Anzahl der zu schreibenden Bytes.
 * @param   intf_ptr: Zeiger auf I2C-Adresse.
 * @return  0 bei Erfolg, -1 bei Fehler.
 */
int8_t bmm350_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
		void *intf_ptr) {
	uint8_t dev_addr = *((uint8_t*) intf_ptr);
	if (HAL_I2C_Mem_Write(&hi2c3, dev_addr << 1, reg_addr,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) reg_data, len, HAL_MAX_DELAY) == HAL_OK)
		return 0;
	else
		return -1;
}

/**
 * @brief   Initialisiert und konfiguriert den BMM350 Magnetometer.
 * @return  None
 */
void userInitBMM350() {

	bmm350.read = bmm350_i2c_read;
	bmm350.write = bmm350_i2c_write;
	bmm350.delay_us = user_delay_us;
	bmm350.intf_ptr = &bmm350_i2c_addr;
	bmm350_init(&bmm350);
	bmm350_set_powermode(BMM350_PMU_CMD_NM, &bmm350);

#if BMM350_CALIBRATION_ACTIVE
	bmm350_set_odr_performance(BMM350_DATA_RATE_400HZ, BMM350_AVG_NO_AVG,
			&bmm350);
#else
	bmm350_set_odr_performance(BMM350_DATA_RATE_50HZ, BMM350_AVG_4, &bmm350);
#endif

	bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &bmm350);
	bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &bmm350);
	bmm350_configure_interrupt(BMM350_PULSED, BMM350_ACTIVE_LOW,
			BMM350_INTR_PUSH_PULL, BMM350_MAP_TO_PIN, &bmm350);
}

/* BMI088 Config -------------------------------------------------------------*/

/**
 * @brief   SPI-Read-Funktion für BMI088 (ACC und GYRO).
 * @param   reg_addr: Registeradresse.
 * @param   reg_data: Zeiger auf Zielpuffer.
 * @param   len: Anzahl der zu lesenden Bytes.
 * @param   intf_ptr: Zeiger auf Interface-Struktur.
 * @return  0 bei Erfolg, -1 bei Fehler.
 */
int8_t bmi088_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
		void *intf_ptr) {
	bmi088_interface_t *iface = (bmi088_interface_t*) intf_ptr;

	HAL_GPIO_WritePin(iface->cs_port, iface->cs_pin, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(iface->hspi, &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK
			|| HAL_SPI_Receive(iface->hspi, reg_data, len, HAL_MAX_DELAY)
					!= HAL_OK) {
		HAL_GPIO_WritePin(iface->cs_port, iface->cs_pin, GPIO_PIN_SET);
		return -1;
	}

	HAL_GPIO_WritePin(iface->cs_port, iface->cs_pin, GPIO_PIN_SET);
	return 0;
}

/**
 * @brief   SPI-Write-Funktion für BMI088 (ACC und GYRO).
 * @param   reg_addr: Registeradresse.
 * @param   reg_data: Zeiger auf Datenpuffer.
 * @param   len: Anzahl der zu schreibenden Bytes.
 * @param   intf_ptr: Zeiger auf Interface-Struktur.
 * @return  0 bei Erfolg, -1 bei Fehler.
 */
int8_t bmi088_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
		void *intf_ptr) {
	bmi088_interface_t *iface = (bmi088_interface_t*) intf_ptr;

	HAL_GPIO_WritePin(iface->cs_port, iface->cs_pin, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(iface->hspi, &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK) {
		HAL_GPIO_WritePin(iface->cs_port, iface->cs_pin, GPIO_PIN_SET);
		return -1;
	}

	if (HAL_SPI_Transmit(iface->hspi, (uint8_t*) reg_data, len,
	HAL_MAX_DELAY) != HAL_OK) {
		HAL_GPIO_WritePin(iface->cs_port, iface->cs_pin, GPIO_PIN_SET);
		return -1;
	}

	HAL_GPIO_WritePin(iface->cs_port, iface->cs_pin, GPIO_PIN_SET);
	return 0;
}

/**
 * @brief   Initialisiert und konfiguriert den BMI088 IMU.
 * @return  None
 */
void userInitBMI088() {

	acc_iface.hspi = &hspi1;
	acc_iface.cs_port = SPI1_CS_ACC_GPIO_Port;
	acc_iface.cs_pin = SPI1_CS_ACC_Pin;

	accel_int_cfg.int_channel = BMI08_INT_CHANNEL_1;
	accel_int_cfg.int_type = BMI08_ACCEL_INT_DATA_RDY;
	accel_int_cfg.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
	accel_int_cfg.int_pin_cfg.lvl = BMI08_INT_ACTIVE_LOW;
	accel_int_cfg.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

	gyro_iface.hspi = &hspi1;
	gyro_iface.cs_port = SPI1_CS_GYRO_GPIO_Port;
	gyro_iface.cs_pin = SPI1_CS_GYRO_Pin;

	gyro_int_cfg.int_channel = BMI08_INT_CHANNEL_3;
	gyro_int_cfg.int_type = BMI08_GYRO_INT_DATA_RDY;
	gyro_int_cfg.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
	gyro_int_cfg.int_pin_cfg.lvl = BMI08_INT_ACTIVE_LOW;
	gyro_int_cfg.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

	bmi088.intf_ptr_accel = &acc_iface;
	bmi088.intf_ptr_gyro = &gyro_iface;
	bmi088.intf = BMI08_SPI_INTF;
	bmi088.read_write_len = 32;
	bmi088.read = bmi088_spi_read;
	bmi088.write = bmi088_spi_write;
	bmi088.delay_us = user_delay_us;

	bmi088.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
	bmi088.accel_cfg.bw = BMI08_ACCEL_BW_OSR4;
	bmi088.accel_cfg.odr = BMI08_ACCEL_ODR_800_HZ;

	accel_int_cfg.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
	accel_int_cfg.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
	accel_int_cfg.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

	bmi088.gyro_cfg.range = BMI08_GYRO_RANGE_500_DPS;
	bmi088.gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ;
	bmi088.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

	bmi08a_soft_reset(&bmi088);
	bmi08a_init(&bmi088);
	bmi08a_set_meas_conf(&bmi088);
	bmi08a_set_power_mode(&bmi088);
	bmi08a_set_int_config(&accel_int_cfg, &bmi088);

	bmi08g_soft_reset(&bmi088);
	bmi08g_init(&bmi088);
	bmi08g_set_meas_conf(&bmi088);
	bmi08g_set_power_mode(&bmi088);
	bmi08g_set_int_config(&gyro_int_cfg, &bmi088);
	bmi08g_get_data(&rawGyro, &bmi088);
}
