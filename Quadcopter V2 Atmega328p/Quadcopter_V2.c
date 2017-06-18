/*
 * Quadcopter_V2.c
 *
 * Created: 4/03/2017 8:08:30 AM
 * Author: Jessica Hu
 */ 


#include "Quadcopter_V2.h"
#include "Fly_Control.h"

int main(void)
{
	gpio_initialize(); // Initialize GPIO pins
	printf("Jess's Quadcopter\n\r"); //Intro sentence
	led_startUpSequence(); // LED startup indicator
	
	//twi_test();
	//ADXL345_test();
	//ITG3200_test();
	//HMC5883L_test();
	//GP2Y0A21_test();
	//flyPID_gyroCalibrationTest();
	pwm_test_2();
}




