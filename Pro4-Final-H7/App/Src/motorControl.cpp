/**
 ******************************************************************************
 * @file           : motorControl.cpp
 * @brief          : Beschreibt die Motorsteuerung.
 ******************************************************************************
 * @author Ralf Pianzola
 * @date 06.05.2025
 * @version 1.0
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "motorControl.h"

/* Externs  ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart3;
extern osMessageQueueId_t qTargetQueue;
extern osMessageQueueId_t qCurrentQueue;

/* Structs -------------------------------------------------------------------*/

/**
 * @brief globale Quaternions für Ist- und Zielausrichtung
 */
Quaternion qTarget;
Quaternion qCurrent;

/**
 * @brief PID-Regler für Roll, Pitch & Yaw
 */
PIDController PIDControllerRoll;
PIDController PIDControllerPitch;
PIDController PIDControllerYaw;

/**
 * @brief Encoder-Objekte
 */
AS5048A encoderM1(&hspi4, SPI4_CS_M1_GPIO_Port, SPI4_CS_M1_Pin);
AS5048A encoderM2(&hspi4, SPI4_CS_M2_GPIO_Port, SPI4_CS_M2_Pin);
AS5048A encoderM3(&hspi4, SPI4_CS_M3_GPIO_Port, SPI4_CS_M3_Pin);

/* Declarations --------------------------------------------------------------*/
static uint32_t previousMotorControlTick = 0;

/* Pre-Declarations ----------------------------------------------------------*/

void motorControlTask(void *argument);

void controlTimerCallback(void *argument);

void getGimbalAttitudeTask(void *argument);

void getCurrentQuaternionTask(void *argument);

Quaternion getErrorQuaternion(Quaternion qTarget, Quaternion qCurrent);

void getErrorAngle(Quaternion qError, float *errorRoll, float *errorPitch,
		float *errorYaw);

void updatePIDController(PIDController *pid, float error, float deltaTime,
		float *output);

void setPWMOutput(TIM_HandleTypeDef *tim, AS5048A encoder, float speed);

void testMotor(TIM_HandleTypeDef *tim, AS5048A encoder);

/* Free-RTOS -----------------------------------------------------------------*/

/**
 * @brief RTOS Flag für die Motorregelung
 */
osEventFlagsId_t controlIntFlag;

/**
 * @brief RTOS Timer für den motorControlTask (1kHz)
 */
osTimerId_t controlTimerHandle;

/**
 * @brief Task-Handle für die Motorregelung.
 */
osThreadId_t motorControlTaskHandle;

/**
 * @brief Task-Attribute für die Motorregelung
 */
static const osThreadAttr_t motorControlTask_attributes = { .name =
		"motorControlTask", .stack_size = 8192, .priority =
		(osPriority_t) osPriorityAboveNormal };

/**
 * @brief Task-Handle zum Empfangen des Ziel-Quaternions
 */
osThreadId_t getGimbalAttitudeTaskHandle;

/**
 * @brief Task-Attribute für das Empfangen des Ziel-Quaternions
 */
static const osThreadAttr_t getGimbalAttitudeTask_attributes = { .name =
		"getGimbalAttitudeTask", .stack_size = 1024, .priority =
		(osPriority_t) osPriorityNormal };

/**
 * @brief Task-Handle zum Empfangen des Ist-Quaternions.
 */
osThreadId_t getCurrentQuaternionTaskHandle;

/**
 * @brief Task-Attribute für das Empfangen des Ist-Quaternions
 */
static const osThreadAttr_t getCurrentQuaternionTask_attributes = { .name =
		"getCurrentQuaternionTask", .stack_size = 1024, .priority =
		(osPriority_t) osPriorityNormal };

/**
 * @brief Attribute für den Timer
 */
static const osTimerAttr_t controlTimer_attributes = { .name = "controlTimer" };

/* Funktionen ----------------------------------------------------------------*/

/**
 * @brief Initialisiert die Encoder.
 */
void initEncoder(void) {
	encoderM1.init();
	encoderM2.init();
	encoderM3.init();
}

/**
 * @brief Initialisiert die PID-Regler für Roll, Pitch und Yaw.
 *
 * Setzt die Parameter gemäss den Werten in der PID_Config.h
 */
void initPIDController(void) {

	PIDControllerRoll.Kp = PID_ROLL_KP;
	PIDControllerRoll.Ki = PID_ROLL_KI;
	PIDControllerRoll.Kd = PID_ROLL_KD;
	PIDControllerRoll.tau = PID_D_FILTER_TAU;
	PIDControllerRoll.limMin = PID_OUTPUT_MIN;
	PIDControllerRoll.limMax = PID_OUTPUT_MAX;
	PIDControllerRoll.limMinInt = PID_INT_MIN;
	PIDControllerRoll.limMaxInt = PID_INT_MAX;
	PIDControllerRoll.T = PID_SAMPLE_TIME_S;

	PIDControllerPitch.Kp = PID_PITCH_KP;
	PIDControllerPitch.Ki = PID_PITCH_KI;
	PIDControllerPitch.Kd = PID_PITCH_KD;
	PIDControllerPitch.tau = PID_D_FILTER_TAU;
	PIDControllerPitch.limMin = PID_OUTPUT_MIN;
	PIDControllerPitch.limMax = PID_OUTPUT_MAX;
	PIDControllerPitch.limMinInt = PID_INT_MIN;
	PIDControllerPitch.limMaxInt = PID_INT_MAX;
	PIDControllerPitch.T = PID_SAMPLE_TIME_S;

	PIDControllerYaw.Kp = PID_YAW_KP;
	PIDControllerYaw.Ki = PID_YAW_KI;
	PIDControllerYaw.Kd = PID_YAW_KD;
	PIDControllerYaw.tau = PID_D_FILTER_TAU;
	PIDControllerYaw.limMin = PID_OUTPUT_MIN;
	PIDControllerYaw.limMax = PID_OUTPUT_MAX;
	PIDControllerYaw.limMinInt = PID_INT_MIN;
	PIDControllerYaw.limMaxInt = PID_INT_MAX;
	PIDControllerYaw.T = PID_SAMPLE_TIME_S;

	PIDController_Init(&PIDControllerRoll);
	PIDController_Init(&PIDControllerPitch);
	PIDController_Init(&PIDControllerYaw);
}

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
void initMotorControl(void) {

	// Enable Pins
	HAL_GPIO_WritePin(DRIVER_M1_EN1_GPIO_Port, DRIVER_M1_EN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DRIVER_M1_EN2_GPIO_Port, DRIVER_M1_EN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DRIVER_M1_EN3_GPIO_Port, DRIVER_M1_EN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M1_EN_DIAG_GPIO_Port, M1_EN_DIAG_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(DRIVER_M2_EN1_GPIO_Port, DRIVER_M2_EN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DRIVER_M2_EN2_GPIO_Port, DRIVER_M2_EN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DRIVER_M2_EN3_GPIO_Port, DRIVER_M2_EN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M2_EN_DIAG_GPIO_Port, M2_EN_DIAG_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(DRIVER_M3_EN1_GPIO_Port, DRIVER_M3_EN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DRIVER_M3_EN2_GPIO_Port, DRIVER_M3_EN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DRIVER_M3_EN3_GPIO_Port, DRIVER_M3_EN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M3_EN_DIAG_GPIO_Port, M3_EN_DIAG_Pin, GPIO_PIN_SET);

	// PWM-Timer
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	// RTOS-Timer
	// TODO Timer Zeit prüfen
	osTimerStart(controlTimerHandle, 4);

	// RTOS-Flag
	controlIntFlag = osEventFlagsNew(NULL);

	// RTOS-Tasks
	controlTimerHandle = osTimerNew(controlTimerCallback, osTimerPeriodic, NULL,
			&controlTimer_attributes);
	motorControlTaskHandle = osThreadNew(motorControlTask, NULL,
			&motorControlTask_attributes);
	getGimbalAttitudeTaskHandle = osThreadNew(getGimbalAttitudeTask, NULL,
			&getGimbalAttitudeTask_attributes);
	getCurrentQuaternionTaskHandle = osThreadNew(getCurrentQuaternionTask, NULL,
			&getCurrentQuaternionTask_attributes);
}

/**
 * @brief FreeRTOS-Task zur Motorregelung des Gimbals.
 *
 * Diese Task wird zyklisch durch ein EventFlag (controlTimer) ausgelöst.
 * Auf Basis des Ist- und Ziel-Quaternions wird folgendes berechnet:
 * - das Fehlerquaternion,
 * - die Fehlerwinkel in Roll, Pitch, Yaw,
 * - die Stellgrössen über drei PID-Regler.
 *
 * Danach werden die einzelnen Motoren mit der jeweiligen Stellgrösse angesteuert.
 */
void motorControlTask(void *argument) {
	float errorRoll;
	float errorPitch;
	float errorYaw;

	Quaternion qTargetCopy;
	Quaternion qCurrentCopy;
	Quaternion qError;

	float speedRoll;
	float speedPitch;
	float speedYaw;

	uint32_t currentTick;
	float deltaTime;

	for (;;) {
		osEventFlagsWait(controlIntFlag, flagIntReceived, osFlagsWaitAny,
		osWaitForever);

		taskENTER_CRITICAL();
		qTargetCopy = qTarget;
		qCurrentCopy = qCurrent;
		taskEXIT_CRITICAL();

		qError = getErrorQuaternion(qTargetCopy, qCurrentCopy);
		getErrorAngle(qError, &errorRoll, &errorPitch, &errorYaw);

		currentTick = osKernelGetTickCount();
		deltaTime = (currentTick - previousMotorControlTick) / 1000.0f;
		previousMotorControlTick = currentTick;

		updatePIDController(&PIDControllerRoll, errorRoll, deltaTime,
				&speedRoll);
		updatePIDController(&PIDControllerPitch, errorPitch, deltaTime,
				&speedPitch);
		updatePIDController(&PIDControllerYaw, errorYaw, deltaTime, &speedYaw);

		// TODO prüfen, ob zuweisung stimmt
		setPWMOutput(&htim1, encoderM1, speedRoll);
		setPWMOutput(&htim2, encoderM2, speedPitch);
		setPWMOutput(&htim3, encoderM3, speedYaw);

	}
}

/**
 * @brief Callback-Funktion des Timer-Interrupts.
 *
 * Setzt ein Flag, welches den motorControlTask startet.
 */
void controlTimerCallback(void *argument) {
	osEventFlagsSet(controlIntFlag, flagIntReceived);
}

/**
 * @brief RTOS-Task zum Auslesen des Ziel-Quaternions.
 * Wird durch die Queue aktiviert und schreibt die Daten
 * in das globale qTarget.
 */
void getGimbalAttitudeTask(void *argument) {
	float qTargetCopy[4];

	for (;;) {
		if (osMessageQueueGet(qTargetQueue, &qTargetCopy, NULL, osWaitForever)
				== osOK) {
			taskENTER_CRITICAL();
			qTarget.w = qTargetCopy[0];
			qTarget.x = qTargetCopy[1];
			qTarget.y = qTargetCopy[2];
			qTarget.z = qTargetCopy[3];
			taskEXIT_CRITICAL();
		}
	}
}

/**
 * @brief RTOS-Task zum Auslesen des Ist-Quaternions.
 * Wird durch die Queue aktiviert und schreibt die Daten
 * in das globale qCurrent.
 */
void getCurrentQuaternionTask(void *argument) {
	float qCurrentCopy[4];

	for (;;) {
		if (osMessageQueueGet(qCurrentQueue, &qCurrentCopy, NULL, osWaitForever)
				== osOK) {
			taskENTER_CRITICAL();
			qCurrent.w = qCurrentCopy[0];
			qCurrent.x = qCurrentCopy[1];
			qCurrent.y = qCurrentCopy[2];
			qCurrent.z = qCurrentCopy[3];
			taskEXIT_CRITICAL();
		}
	}
}

/**
 * @brief berechnet das Fehler-Qauternion, welches für Winkelabweichung
 * benötigt wird.
 *
 * @param qTarget das Ziel-Quaternion
 * @param qCurrent das aktuelle Ist-Quaternion
 *
 * @return qError Fehler-Qauternion
 */
Quaternion getErrorQuaternion(Quaternion qTarget, Quaternion qCurrent) {
	Quaternion qError;

	qTarget.normalize();
	qCurrent.normalize();
	qError = qTarget.conjugate();
	qError.normalize();
	qError *= qCurrent;
	qError.normalize();

	return qError;
}

/**
 * @brief Berechnet den Fehlerwinkel.
 * Der Fehlerwinkel der Achsen berechnet sich anhand des Achsenwinkels mit der jeweiligen Achse multipliziert.
 *
 * @param qError Fehlerquaternion
 * @param errorRoll Zeiger auf den Fehler der Roll-Achse
 * @param errorPitch Zeiger auf den Fehler der Pitch-Achse
 * @param errorYaw Zeiger auf den Fehler der Yaw-Achse
 *
 *@note: siehe https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
 */
void getErrorAngle(Quaternion qError, float *errorRoll, float *errorPitch,
		float *errorYaw) {

	float angle = 2.0f * acos(qError.w);
	float sinAngle = sin(angle / 2.0f);

	if (fabsf(sinAngle) < 1e-6f) {
		*errorRoll = 0.0f;
		*errorPitch = 0.0f;
		*errorYaw = 0.0f;
		return;
	}

	*errorRoll = angle * qError.x / sinAngle;
	*errorPitch = angle * qError.y / sinAngle;
	*errorYaw = angle * qError.z / sinAngle;
}

/**
 * @brief Aktualisiert den PID-Regler
 *
 * @param pid Zeiger auf den PID-Regler
 * @param error der aktuelle Fehler
 * @param deltaTime die Zeit in s, seit dem letzten Aufruf
 * @param output Zeiger auf die berechnete Stellgrösse
 */
void updatePIDController(PIDController *pid, float error, float deltaTime,
		float *output) {
	pid->T = deltaTime;
	*output = PIDController_Update(pid, 0.0f, error);
}

/**
 * @brief setzt den richtigen Duty-Cycle
 *
 * @param tim Timer-Handle zur PWM-Ausgabe.
 * @param encoder Encoder des Motors.
 * @param speed entspricht dem Ausgang des PID-Reglers und
 * liegt im Bereich von [-1...1]
 */
void setPWMOutput(TIM_HandleTypeDef *tim, AS5048A encoder, float speed) {
	uint32_t raw = encoder.getRawRotation();
	uint32_t electricalAngle = (raw * NUM_POLEPAIRS) % ENCODER_RESOLUTION;
	uint16_t lutIndex = (electricalAngle * (SIN_NVALUES - 1))
			/ ENCODER_RESOLUTION;

	lutIndex = (lutIndex + SIN_NVALUES / 4) % SIN_NVALUES;

	uint16_t lutIndexU = lutIndex;
	uint16_t lutIndexV = (lutIndex + SIN_NVALUES / 3) % SIN_NVALUES;
	uint16_t lutIndexW = (lutIndex + 2 * SIN_NVALUES / 3) % SIN_NVALUES;

	uint16_t u = (speed * (sintable[lutIndexU] + 1.0f)) * PWM_MAX_VAL / 2;
	uint16_t v = (speed * (sintable[lutIndexV] + 1.0f)) * PWM_MAX_VAL / 2;
	uint16_t w = (speed * (sintable[lutIndexW] + 1.0f)) * PWM_MAX_VAL / 2;

	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, u);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, v);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_3, w);
}

/**
 * @brief Testfunktion, zur Überprüfung der Motorfunktion.
 * Steuert die Motoren per Sinuskommutierung mit
 * 20% des maximalen Speeds an.
 *
 * @param tim Timer-Handle zur PWM-Ausgabe.
 * @param encoder Encoder des Motors.
 */
void testMotor(TIM_HandleTypeDef *tim, AS5048A encoder) {
	float speed = 0.2f;
	uint32_t raw = encoder.getRawRotation();
	uint32_t electricalAngle = (raw * NUM_POLEPAIRS) % ENCODER_RESOLUTION;
	uint16_t lutIndex = (electricalAngle * (SIN_NVALUES - 1))
			/ ENCODER_RESOLUTION;

	lutIndex = (lutIndex + SIN_NVALUES / 4) % SIN_NVALUES;

	uint16_t lutIndexU = lutIndex;
	uint16_t lutIndexV = (lutIndex + SIN_NVALUES / 3) % SIN_NVALUES;
	uint16_t lutIndexW = (lutIndex + 2 * SIN_NVALUES / 3) % SIN_NVALUES;

	uint16_t u = (speed * (sintable[lutIndexU] + 1.0f)) * PWM_MAX_VAL / 2;
	uint16_t v = (speed * (sintable[lutIndexV] + 1.0f)) * PWM_MAX_VAL / 2;
	uint16_t w = (speed * (sintable[lutIndexW] + 1.0f)) * PWM_MAX_VAL / 2;

	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, u);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, v);
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_3, w);
}

