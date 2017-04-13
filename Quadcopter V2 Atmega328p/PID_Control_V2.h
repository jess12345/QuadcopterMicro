/*
 * PID_Control_V2.h
 *
 * Created: 8/03/2017 5:06:26 PM
 * Author: Jessica Hu
 */ 


#ifndef PID_CONTROL_V2_H_
#define PID_CONTROL_V2_H_

#include <stdio.h>

// ===================== PID Algorithm ==================== //

typedef struct PID_DATA {
	int16_t roll_error;
	int32_t roll_sumError;
	//int16_t roll_maxError;
	//int32_t roll_maxSumError;
	int16_t pitch_error;
	int32_t pitch_sumError;
	//int16_t pitch_maxError;
	//int32_t pitch_maxSumError;
	int16_t yaw_error;
	int32_t yaw_sumError;
	//int16_t yaw_maxError;
	//int32_t yaw_maxSumError;
} pidData_t;
pidData_t pid_last_data;

//#define MAX_ERROR		INT16_MAX
//#define MAX_SUM_ERROR	INT32_MAX

#define SCALING_FACTOR	128
#define ROLL_PID_KP		0.250
#define ROLL_PID_KI		0.950
#define ROLL_PID_KD		0.011
//#define ROLL_PID_MIN	-200.0
//#define ROLL_PID_MAX	200.0

#define PITCH_PID_KP	0.250
#define PITCH_PID_KI	0.950
#define PITCH_PID_KD	0.011
//#define PITCH_PID_MIN	-200.0
//#define PITCH_PID_MAX	200.0

#define YAW_PID_KP		0.680
#define YAW_PID_KI		0.500
#define YAW_PID_KD		0.0001
//#define YAW_PID_MIN		100.0
//#define YAW_PID_MAX		100.0

#define MAX_INT INT16_MAX
#define MAX_LONG INT32_MAX
#define MAX_I_TERM (MAX_LONG / 2)

#define ROLL_MAX_ERROR	(MAX_INT / (ROLL_PID_KP + 1))
#define PITCH_MAX_ERROR	(MAX_INT / (PITCH_PID_KP + 1))
#define YAW_MAX_ERROR	(MAX_INT / (YAW_PID_KP + 1))

#define ROLL_MAX_SUM_ERROR	(MAX_I_TERM / (ROLL_PID_KI + 1))
#define PITCH_MAX_SUM_ERROR	(MAX_I_TERM / (PITCH_PID_KI + 1))
#define YAW_MAX_SUM_ERROR	(MAX_I_TERM / (YAW_PID_KI + 1))

float gyro_offset[3];

void pid_Initialise(void);
void flyPID_calibrateGyro(void);
void pid_compute(	int32_t roll_desired,	int32_t roll_actual,	int32_t *roll_after_pid,
					int32_t pitch_desired,	int32_t pitch_actual,	int32_t *pitch_after_pid,
					int32_t yaw_desired,	int32_t yaw_actual,		int32_t *yaw_after_pid);
int32_t pid_P_term(int32_t error, int32_t limit, int32_t kp);
int32_t pid_I_term(int32_t error, int32_t *lastSumError, int32_t limit,int32_t ki);
int32_t pid_D_term(int32_t error, int32_t *lastError, int32_t kd);
int32_t pid_calculateOutput(int32_t p_term, int32_t i_term, int32_t d_term);


#endif /* PID_CONTROL_V2_H_ */