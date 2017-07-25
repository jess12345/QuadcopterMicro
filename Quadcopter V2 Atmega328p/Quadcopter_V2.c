/*
 * Quadcopter_V2.c
 *
 * Created: 4/03/2017 8:08:30 AM
 * Author: Jessica Hu
 */ 


#include "Quadcopter_V2.h"
#include "Fly_Control.h"
#include "bool.h"

int main(void)
{
	gpio_initialize(); // Initialize GPIO pins
	printf("Jess's Quadcopter\n\r"); //Intro sentence
	led_startUpSequence(); // LED startup indicator
	
	runStateMachine();
}

// ===================== State Machine ==================== //

void runStateMachine(void)
{
	QState quadcopter_state = CALIBRATION;
	
	while(1){
		switch(quadcopter_state){
			case CALIBRATION:	quadcopter_state = runCalibrationState();	break;
			case IDLE:			quadcopter_state = runIdleState();			break;
			case BLASTOFF:		quadcopter_state = runBlastoffState();		break;
			case FLY:			quadcopter_state = runFlyState();			break;
			case TOUCHDOWN:		quadcopter_state = runTouchdownState();		break;
			case TEST:			quadcopter_state = runTestState();			break;
			default:
				printf("Error occurred in the main state machine: ID=%d", quadcopter_state);
				break;
		}
	}
}

// ======================== States ======================== //

QState runTestState(void)
{
	//twi_test();
	//ADXL345_test();
	//ITG3200_test();
	//HMC5883L_test();
	//GP2Y0A21_test();
	//flyPID_gyroCalibrationTest();
	pwm_test_2();
	
	return IDLE;
}


QState runCalibrationState(void)
{
	
	//test that all sensors are connected
	
	//calibrate gyro
	
	
	return IDLE;
}


QState runIdleState(void)
{
	printf("Please select from options:\n\r"
			"\t[1] run tests\n\r"
			"\t[2] hover\n\r");
	
	int selected_option = USART_Receive_Number();
	
	switch(selected_option){
		case 1: 
			return TEST;
		case 2: 
			return BLASTOFF;
		default: 
			printf("Invalid option %d\n\r",selected_option); 
			return IDLE;
	}
}


QState runBlastoffState(void)
{
	Boolean blastoff_success = TRUE;
	
	//elevate quad copter to 30 cm.
	
	
	return (blastoff_success? FLY : IDLE);
}


QState runFlyState(void)
{
	Boolean encountered_error = FALSE;
	
	//give control to the user, however limit the ground distance to be greater than 30 cm.
	
	
	return (encountered_error? IDLE : TOUCHDOWN);
}


QState runTouchdownState(void)
{	
	//land the quadcopter safely
	
	
	
	return IDLE;	
}