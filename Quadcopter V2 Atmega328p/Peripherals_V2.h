/*
 * Peripherals_V2.h
 *
 * Created: 6/12/2016 9:23:52 PM
 * Author: Jessica Hu
 */ 


#ifndef Peripherals_V2_H_
#define Peripherals_V2_H_

// ===================== Definitions ====================== //
#define F_CPU 1000000UL
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#define USART_BAUD 4800
#define UBRR_VALUE ((F_CPU/(USART_BAUD*16UL)) - 1)
#define PWM_MAX 256;



// ========================= GPIO ========================= //
void gpio_initialize(void);


// ========================= LED ========================== //
//void led_test_1(void);
void led_startUpSequence(void);
void led_initialize(void);
void led_on(void);
void led_off(void);
void led_toggle(void); //UNTESTED!!!

// ========================= UART ========================= //
//void uart_test_1(void);
void usart_initialize(void);
void usart_send_char(unsigned char data);
//void uart_sendstring(char *stringprint);
int usart_printf(unsigned char data, FILE *stream);
//void print_float(float num);
//unsigned char uart_flush(void);
unsigned char usart_receive_char(void);
unsigned int USART_Receive_Number(void);
static FILE UARTstdout = FDEV_SETUP_STREAM(usart_printf, NULL, _FDEV_SETUP_WRITE); //Allow printf to work



// ========================= PWM ========================== //
uint8_t pwm_dutycycle_FL;
uint8_t pwm_dutycycle_FR;
uint8_t pwm_dutycycle_BL;
uint8_t pwm_dutycycle_BR;
//void pwm_test_1(void);
void pwm_test_2(void);
void pwm_initialize(void);
void pwm_enable(void);
void pwm_disable(void);
void pwm_apply_dutycycle(uint8_t pwm_FL, uint8_t pwm_FR, uint8_t pwm_BL, uint8_t pwm_BR);



// ========================= ESC ========================== //




// ==================== GP2Y0A21 (IR) ===================== //
//#define GP2Y0A21_SCALE_FACTOR 1
void GP2Y0A21_test(void);
void GP2Y0A21_initialise(void);
float GP2Y0A21_getDistance(void);

// ====================== Ultrasound ====================== //
//void ultrasound_test();
void untrasound_initialise(void);
int untrasound_getDistance(void);

// ======================== Timer ========================= //
void timer1_test(void); //UNTESTED!!!
void timer1_initialise(void); //UNTESTED!!!



// ====================== I2C(TWI) ======================== //
#define TWI_TWBR 2
#define TWI_PRESCALER 1
void twi_test(void);
void twi_initialise(void);
void twi_start(void);
void twi_stop(void);
void twi_write(uint8_t u8data);
uint8_t twi_readACK(void);
uint8_t twi_readNACK(void);
uint8_t twi_getStatus(void);

uint8_t twi_readRegister(uint8_t slave_address_write,uint8_t slave_address_read, uint8_t register);
void twi_WriteRegister(uint8_t slave_address_write,uint8_t reg, uint8_t data);

// =============== ADXL345 (Accelerometer) ================ //
#define ADXL345_ADDRESS_W		0xA6 //Device address + W
#define ADXL345_ADDRESS_R		0xA7 //Device address + R
#define ADXL345_DEVID_R			0x00 //Device ID
#define ADXL345_THRESH_TAP_RW	0x1D //Tap threshold
#define ADXL345_OFSX_RW			0x1E //X-axis offset
#define ADXL345_OFSY_RW			0x1F //Y-axis offset
#define ADXL345_OFSZ_RW			0x20 //Z-axis offset
#define ADXL345_DUR				0x21 //Tap duration
#define ADXL345_BW_RATE			0x2C //Data rate and power mode control
#define ADXL345_POWER_CTRL		0x2D //Power-saving feature control
#define ADXL345_INT_SOURCE		0x30 //Source of interrupts
#define ADXL345_DATA_FORMAT_RW	0x31 //Data format control
#define ADXL345_DATAX0_R		0x32 //X-Axis Data 0
#define ADXL345_DATAX1_R		0x33 //X-Axis Data 1
#define ADXL345_DATAY0_R		0x34 //Y-Axis Data 0
#define ADXL345_DATAY1_R		0x35 //Y-Axis Data 1
#define ADXL345_DATAZ0_R		0x36 //Z-Axis Data 0
#define ADXL345_DATAZ1_R		0x37 //Z-Axis Data 1
#define ADXL345_FIFO_CTL_RW		0x38 //FIFO control
#define ADXL345_FIFO_STATUS_R	0x39 //FIFO status


#define ADXL345_SCALE_FACTOR    0.0039 //ADXL345 Full Resolution Scale Factor
//#define ADXL345_SCALE_FACTOR_UM    3900 //ADXL345 Full Resolution Scale Factor (to convert to 10^-6 m/s2)


void ADXL345_initialise(void);
void ADXL345_WriteRegister(uint8_t reg, uint8_t data);
//uint8_t ADXL345_ReadRegister(uint8_t reg);
//char ADXL345_ReadDeviceId(void);
void ADXL345_ReadAccelerometer(float* accelerometer);
//void ADXL345_test0(void);
void ADXL345_test(void);


// ================= ITG3200 (Gyroscope) ================== //
#define ITG3200_ADDRESS_W		0xD0 //Device address + W
#define ITG3200_ADDRESS_R		0xD1 //Device address + R
#define ITG3200_DEVID			0x00 //Device ID
#define ITG3200_TEMP_H			0x1B //Temp high
#define ITG3200_TEMP_L			0x1C //Temp low
#define ITG3200_DATAX_H			0x1D //X-Axis data high
#define ITG3200_DATAX_L			0x1E //X-Axis data low
#define ITG3200_DATAY_H			0x1F //Y-Axis data high
#define ITG3200_DATAY_L			0x20 //Y-Axis data low
#define ITG3200_DATAZ_H			0x21 //Z-Axis data high
#define ITG3200_DATAZ_L			0x22 //Z-Axis data low

void ITG3200_test(void);
void ITG3200_initialise(void);
void ITG3200_ReadGyro(float* gyro);


// =============== HMC5883L (Magnetometer) ================ //
#define HMC5883L_ADDRESS_W		0x3C //Device address + W
#define HMC5883L_ADDRESS_R		0x3D //Device address + R
#define HMC5883L_CONFIG_A		0x00 //Configuration register A
#define HMC5883L_CONFIG_B		0x01 //Configuration register B
#define HMC5883L_MODE			0x02 //Mode register
#define HMC5883L_DATAX_H		0x03 //X-Axis data high
#define HMC5883L_DATAX_L		0x04 //X-Axis data low
#define HMC5883L_DATAY_H		0x05 //Y-Axis data high
#define HMC5883L_DATAY_L		0x06 //Y-Axis data low
#define HMC5883L_DATAZ_H		0x07 //Z-Axis data high
#define HMC5883L_DATAZ_L		0x08 //Z-Axis data low

void HMC5883L_test(void);
void HMC5883L_initialise(void);
void HMC5883L_ReadMag(float* mag);


// ============= MPU9150 (Mag + Acc + Gyro) =============== //
typedef struct MPU9150_DATA {
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float acc_x;
	float acc_y;
	float acc_z;
	float mag_x;
	float mag_y;
	float mag_z;
} Mpu9150_data_t;

void MPU9150_test(void);
void MPU9150_initialise(Mpu9150_data_t *data);
void MPU9150_readData(Mpu9150_data_t *data);

// ================= Radio Frequency(RF) ================== //
void radio_test(void); //UNTESTED!!!
void radio_initialise(void); //UNTESTED!!!

#endif /* Peripherals_V2_H_ */