/*
 * Fly_Control.c
 *
 * Created: 19/06/2017 07:42:56
 *  Author: hhu88
 */ 

#include "Fly_Control.h"
#include "PID_Control_V2.h"
#include "Peripherals_V2.h"

// ================== PID Flight Control ================== //

/*
void flyPID_gyroCalibrationTest(void)
{
	flyPID_initialise();
	//printf("ANG\t%d\t%d\t%d\tdeg/s\n\r",(int32_t)(angleX*1000),(int32_t)(angleY*1000),(int16_t)(angleZ*1000));
	
	flyPID_calibrateGyro();
	
	printf("ANG\t%d\t%d\t%d\tdegrees\n\r",(int32_t)angleX,(int32_t)angleY,(int16_t)angleZ);
	
	while(1)
	{
		pid_roll_setpoint = 0;
		pid_pitch_setpoint = 0;
		pid_yaw_setpoint = 0;
		
		flyPID_imu_update();
		//flyPID_control_update();
		//printf("ANG\t%d\t%d\t%d\tdeg/s\n\r",(int32_t)(angleX*1000),(int32_t)(angleY*1000),(int16_t)(angleZ*1000));
		printf("ANG\t%d\t%d\t%d\tdegrees\n\r",(int32_t)angleX,(int32_t)angleY,(int16_t)angleZ);
		
		_delay_ms(100); // Loop approximately every 100ms.
	}
}
*/

void flyPID_initialise(void)
{
	// Angles
	angleX = 0.0;
	angleY = 0.0;
	angleZ = 0.0;

	gyro_offset[0] = 0.0;
	gyro_offset[1] = 0.0;
	gyro_offset[2] = 0.0;
	
	// PID variables
	pid_roll_in = 0;
	pid_roll_out = 0;
	pid_roll_setpoint = 0;
	pid_pitch_in = 0;
	pid_pitch_out = 0;
	pid_pitch_setpoint = 0;
	pid_yaw_in = 0;
	pid_yaw_out = 0;
	pid_yaw_setpoint = 0;
	
	// Motors
	m0 = 0; // front
	m1 = 0; // right
	m2 = 0; // back
	m3 = 0; // left

	// Track loop time.
	prev_time = 0;
	
	//printf("ANG\t%d\t\t%d\t%d\t%d\tdeg/s\n\r",(int32_t)(angleX*1000),(int32_t)(angleY*1000),(int16_t)(angleZ*1000));
}


void flyPID_calibrateGyro(void)
{
	int16_t i = 0;
	float gyro[4] = {0.0, 0.0, 0.0, 0.0}; // [Temp, X, Y, Z]
	
	for(i=0;i<20;i++){ // Ignore first 20 data points
		ITG3200_ReadGyro(&gyro); // Update gyro
		printf("IGNORE ROT(%d)\t%d\t%d\t%d\tdeg/s\n\r",i,(int32_t)gyro[1],(int32_t)gyro[2],(int16_t)gyro[3]);
		_delay_ms(100); // Same frequency as control loop to reduce potential effect of frequency coupling
	}	
	
	float sum_x = 0.0;
	float sum_y = 0.0;
	float sum_z = 0.0;
	
	for(i=0;i<100;i++){
		ITG3200_ReadGyro(&gyro); // Update gyro
		printf("CALIBRATE ROT(%d)\t%d\t%d\t%d\tdeg/s\n\r",i,(int32_t)gyro[1],(int32_t)gyro[2],(int16_t)gyro[3]);
		sum_x += gyro[1];
		sum_y += gyro[2];
		sum_z += gyro[3];
		printf("CALIBRATE SUM(%d)\t%d\t%d\t%d\tdeg/s\n\r",i,(int32_t)sum_x,(int32_t)sum_y,(int16_t)sum_z);
		_delay_ms(100); // Same frequency as control loop to reduce potential effect of frequency coupling
	}
	
	gyro_offset[0] = sum_x/100;
	gyro_offset[1] = sum_y/100;
	gyro_offset[2] = sum_z/100;
	
	printf("CALIBRATE OFFSET\t%d\t%d\t%d\tdeg/s\n\r",i,(int32_t)gyro_offset[0],(int32_t)gyro_offset[1],(int16_t)gyro_offset[2]);
}

void flyPID_imu_update(void)
{
	float gyro[4] = {0.0, 0.0, 0.0, 0.0}; // [Temp, X, Y, Z]
	float dt = 0.1; // Loop period
	
	ITG3200_ReadGyro(&gyro); // Update gyro
	
	// Calculate angles
	angleX += (gyro[1]-gyro_offset[0])*dt;
	angleY += (gyro[2]-gyro_offset[1])*dt;
	angleZ += (gyro[3]-gyro_offset[2])*dt;
	
	printf("ROT\t%d\t%d\t%d\tdegrees\n\r",(int32_t)(gyro[1]-gyro_offset[0]),(int32_t)(gyro[2]-gyro_offset[1]),(int16_t)(gyro[3]-gyro_offset[2]));
	
}

void flyPID_control_update(void)
{
	//throttle = flyPID_linear_map(rx_values[RX_THROTTLE],THROTTLE_RMIN,THROTTLE_RMAX,MOTOR_ZERO_LEVEL,MOTOR_MAX_LEVEL);
	  
	flyPID_setpoint_update();
	flyPID_pid_update();
	flyPID_pid_compute();
	  
	// yaw control disabled for stabilization testing...
	m0 = throttle + pid_pitch_out ;//+ pid_yaw_out;
	m1 = throttle + pid_roll_out ;//- pid_yaw_out;
	m2 = throttle - pid_pitch_out ;//+ pid_yaw_out;
	m3 = throttle - pid_roll_out ;//- pid_yaw_out;
	
	/* 
	#ifdef SAFE
	if(throttle < THROTTLE_SAFE_SHUTOFF)
	{
		m0 = m1 = m2 = m3 = MOTOR_ZERO_LEVEL;
	}
	#endif
	*/
	flyPID_update_motors(m0, m1, m2, m3);
}

int flyPID_linear_map(int input_value, int input_low, int input_high, int output_low, int output_high)
{
	return (output_low + (output_high - output_low) * (input_value - input_low) / (input_high - input_low));
}

void flyPID_setpoint_update(void)
{
	
}

void flyPID_pid_update(void)
{
	pid_roll_in = angleX;
	pid_pitch_in = angleY;
	pid_yaw_in = angleZ;
}

void flyPID_pid_compute(void)
{
	pid_compute(	pid_roll_setpoint,	pid_roll_in,	&pid_roll_out,
					pid_pitch_setpoint,	pid_pitch_in,	&pid_pitch_out,
					pid_yaw_setpoint,	pid_yaw_in,		&pid_yaw_out);
}

void flyPID_update_motors(int m0, int m1, int m2, int m3)
{
	
	
	
}
