/*
 * Fly_Control.h
 *
 * Created: 19/06/2017 07:43:20
 *  Author: hhu88
 */ 



#ifndef FLY_CONTROL_H_
#define FLY_CONTROL_H_


// ================== PID Flight Control ================== //
#define SAFE

//-------Motor PWM Levels
#define MOTOR_ZERO_LEVEL  1000
#define MOTOR_ARM_START  1500
#define MOTOR_MAX_LEVEL  2000

float angleX,angleY,angleZ; // Angles

int throttle; // RX Signals
volatile int rx_values[4]; // ROLL, PITCH, THROTTLE, YAW
#define RX_ROLL 0
#define RX_PITCH 1
#define RX_THROTTLE 2
#define RX_YAW 3
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint; // PID variables
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint; // PID variables
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint; // PID variables
int m0, m1, m2, m3; // Front, Right, Back, Left motors
unsigned long prev_time; // Track loop time.

void flyPID_gyroCalibrationTest(void);
void flyPID_initialise(void);
void flyPID_imu_update(void);
void flyPID_control_update(void);
void flyPID_setpoint_update(void);
void flyPID_pid_update(void);
void flyPID_pid_compute(void);
void flyPID_update_motors(int m0, int m1, int m2, int m3);




#endif /* FLY_CONTROL_H_ */