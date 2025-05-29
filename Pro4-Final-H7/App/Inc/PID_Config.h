#ifndef __PID_CONFIG_H
#define __PID_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file           : PID_Config.h
 * @brief          : Enthält alle PID-Parameter für die drei PID-Regler
 ******************************************************************************
 * @author Ralf Pianzola
 * @date 08.05.2025
 * @version 1.0
 ******************************************************************************
 */

#define PID_SAMPLE_TIME_S 0.001f
#define PID_D_FILTER_TAU 0.01f

#define PID_OUTPUT_MIN -1.0f
#define PID_OUTPUT_MAX 1.0f

#define PID_INT_MIN	-0.5f
#define PID_INT_MAX	0.5f

#define PID_ROLL_KP	1.5f
#define PID_ROLL_KI	0.2f
#define PID_ROLL_KD 0.05f

#define PID_PITCH_KP 2.0f
#define PID_PITCH_KI 0.3f
#define PID_PITCH_KD 0.10f

#define PID_YAW_KP 2.5f
#define PID_YAW_KI 0.01f
#define PID_YAW_KD 0.5f

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONFIG_H */
