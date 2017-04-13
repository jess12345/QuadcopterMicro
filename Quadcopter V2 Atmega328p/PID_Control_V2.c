/*
 * PID_Control_V2.c
 *
 * Created: 8/03/2017 5:06:09 PM
 * Author: Jessica Hu
 */ 

#include "PID_Control_V2.h"

// ===================== PID Algorithm ==================== //

void pid_Initialise(void)
{
	pid_last_data.roll_error = 0;
	pid_last_data.pitch_error = 0;
	pid_last_data.yaw_error = 0;
	
	pid_last_data.roll_sumError = 0;
	pid_last_data.pitch_sumError = 0;
	pid_last_data.yaw_sumError = 0;
	/*
	pid_last_data->roll_maxError = MAX_INT / (ROLL_PID_KP + 1);
	pid_last_data->pitch_maxError = MAX_INT / (PITCH_PID_KP + 1);
	pid_last_data->yaw_maxError = MAX_INT / (YAW_PID_KP + 1);
	
	pid_last_data->roll_maxSumError = MAX_INT / (ROLL_PID_KI + 1);
	pid_last_data->pitch_maxSumError = MAX_INT / (PITCH_PID_KI + 1);
	pid_last_data->yaw_maxSumError = MAX_INT / (YAW_PID_KI + 1);
	*/
}


void pid_compute(	int32_t roll_desired,	int32_t roll_actual,	int32_t *roll_after_pid,
					int32_t pitch_desired,	int32_t pitch_actual,	int32_t *pitch_after_pid,
					int32_t yaw_desired,	int32_t yaw_actual,	int32_t *yaw_after_pid)
{
	int32_t roll_error = roll_desired - roll_actual;
	int32_t pitch_error = pitch_desired - pitch_actual;
	int32_t yaw_error = yaw_desired - yaw_actual;
	
	int32_t roll_p, pitch_p, yaw_p;
	int32_t roll_d, pitch_d, yaw_d;
	int32_t roll_i, pitch_i, yaw_i;	
	
	// Calculate P-term and limit error overflow
	roll_p = pid_P_term(roll_error, ROLL_MAX_ERROR, ROLL_PID_KP);
	pitch_p = pid_P_term(pitch_error, PITCH_MAX_ERROR, PITCH_PID_KP);
	yaw_p = pid_P_term(yaw_error, YAW_MAX_ERROR, YAW_PID_KP);
	
	// Calculate I-term and limit integral runaway
	roll_i = pid_I_term(roll_error, &(pid_last_data.roll_sumError), ROLL_MAX_SUM_ERROR, ROLL_PID_KI);
	pitch_i = pid_I_term(pitch_error, &(pid_last_data.pitch_sumError), PITCH_MAX_SUM_ERROR, PITCH_PID_KI);
	yaw_i = pid_I_term(yaw_error, &(pid_last_data.yaw_sumError), YAW_MAX_SUM_ERROR, YAW_PID_KI);
	
	// Calculate D-term
	roll_d = pid_D_term(roll_error, &(pid_last_data.roll_error), ROLL_PID_KD);
	pitch_d = pid_D_term(pitch_error, &(pid_last_data.pitch_error), PITCH_PID_KD);
	yaw_d = pid_D_term(yaw_error, &(pid_last_data.yaw_error), YAW_PID_KD);
	
	// Calculate PID output
	roll_after_pid = pid_calculateOutput(roll_p, roll_i, roll_d);
	pitch_after_pid = pid_calculateOutput(pitch_p, pitch_i, pitch_d);
	yaw_after_pid = pid_calculateOutput(yaw_p, yaw_i, yaw_d);
}


int32_t pid_P_term(int32_t error, int32_t limit, int32_t kp)
{
	if(error > limit){
		return MAX_INT;
	}else if (error < -limit){
		return -MAX_INT;
	}else{
		return (error * kp);
	}
}

int32_t pid_I_term(int32_t error, int32_t *lastSumError, int32_t limit,int32_t ki)
{
	int32_t sum_error = lastSumError + error;
	if(sum_error > limit){
		lastSumError = limit;
		return MAX_I_TERM;
	}else if (sum_error < -limit){
		lastSumError = -limit;
		return -MAX_I_TERM;
	}else{
		lastSumError = sum_error;
		return sum_error * ki;
	}
}

int32_t pid_D_term(int32_t error, int32_t *lastError, int32_t kd)
{
	int32_t error_diff = lastError - error;
	lastError = error;
	return (error_diff * kd);
}

int32_t pid_calculateOutput(int32_t p_term, int32_t i_term, int32_t d_term)
{
	int32_t output = (p_term + i_term + d_term) / SCALING_FACTOR;
	
	if (output > MAX_INT) {
		output = MAX_INT;
	} else if (output < -MAX_INT) {
		output = -MAX_INT;
	}
	
	return output;
}