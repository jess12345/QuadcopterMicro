/*
 * Peripherals_V2.c
 *
 * Created: 4/12/2016 1:31:33 PM
 * Author: Jessica Hu
 */ 

#include "Peripherals_V2.h"


// ========================= GPIO ========================= //

// FUNCTION: Set up all input and output pins
void gpio_initialize(void)
{
	led_initialize();
	pwm_initialize();
	usart_initialize();
	twi_initialise();
	ITG3200_initialise(); // Gyro
}

// ========================= LED ========================== //

/*
// TEST: LED Test 1
void led_test_1(void)
{
	while(1)
	{
		led_off();
		_delay_ms(1000);
		led_on();
		_delay_ms(1000);
	}
}
*/

// FUNCTION: Sequence of LEDs to indicate startup
void led_startUpSequence(void)
{
	led_on();
	_delay_ms(1000);
	led_off();
	_delay_ms(100);
	led_on();
	_delay_ms(100);
	led_off();
	_delay_ms(100);
	led_on();
	_delay_ms(100);
	led_off();
	_delay_ms(1000);
}

// FUNCTION: Initialize LED pins
void led_initialize(void)
{
	DDRB |= (1 << DDB0); // PB0: Debug - Output - LED flash (debug.c Test_Flash_PB0_LED_1s)
}

void led_on(void)
{
	PORTB |= (1<<PORTB0); // PB0 on
}

void led_off(void)
{
	PORTB &= ~(1<<PORTB0); //PB0 off
}

void led_toggle(void) //UNTESTED!!!
{
	PORTB = PORTB ^ (1<<PORTB0); //Toggle PB0
}

// ========================= UART ========================= //

/*
// FUNCTION: Test UART TX
void uart_test_1(void)
{
	while(1)
	{
		printf("a");
		led_on();
		_delay_ms(500);
		led_off();
		_delay_ms(500);
	}
}
*/

// FUNCTION: Initialize USART pins
void usart_initialize(void)
{	
	UBRR0H = (unsigned char) (UBRR_VALUE>>8); // Set baud rate
	UBRR0L = (unsigned char) UBRR_VALUE;
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);	// Turn on the transmission and reception circuitry
	
	//UCSR0C = 0b00001110;// Async,no parity, 1stop, 8 bit,raising clk
	
	// Set frame format: 8data, 2stop bit
	//UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	
	UCSR0C = (3<<UCSZ00);
	
	stdout = &UARTstdout;	//Allow printf to work

	printf("\E[H\E[J"); //Reset display
}

// FUNCTION: send data to USART
void usart_send_char(unsigned char data)
{
	while (  !(UCSR0A & (1<<UDRE0)) ); // wait for empty transit buffer
	UDR0= data; // put data into butter sends the data
}

/*
void uart_sendstring(char *stringprint)
{
	while (*stringprint != 0x00)
	{
		usart_send_char(*stringprint);
		stringprint++;
	}
}
*/

int usart_printf(unsigned char data, FILE *stream)
{
	usart_send_char(data);
	return 0;
}

// FUNCTION: Read data from USART
unsigned char usart_receive_char(void)
{
	unsigned char data;
	while (!(UCSR0A&(1<<RXC0)));				// Wait for data to be received.
	data=UDR0;								// Now save received data
	if (data=='\r'){
		printf("\n\r");
	}else{
		usart_send_char(data);
	}
	return data;
}

// FUNCTION: Read number from USART
unsigned int USART_Receive_Number(void)
{
	unsigned char data[5]={0};
	int loop=0;
	
	do{
		data[loop] = usart_receive_char();
		loop++;
		if(loop>=5)break;
	}while(data[loop-1]!='\r');
	
	int number=0;
	sscanf(data, "%d", &number);
	
	return number;
}

/*
void print_float(float num)
{
	int16_t before_decimal = (int16_t)num;
	int16_t after_decimal = (int16_t)((num-before_decimal)*100.0);
	printf("%d.%d",before_decimal,after_decimal);
}
*/

/*
unsigned char uart_flush(void)
{
	unsigned char dummy;
	while (UCSR0A & (1<<RXC0)) dummy=UDR0;
}
*/

// ========================= PWM ========================== //

/*
// TEST: PWM Test 1
void pwm_test_1(void)
{
	pwm_enable();
	int test_pwm = 0;
	//int max=PWM_MAX;
	//printf("PWM_MAX=%d\n\r",max);
	
	while(1)
	{
		led_on();
		test_pwm = (test_pwm+5)%PWM_MAX;
		//printf("%d\n\r",test_pwm);
		pwm_apply_dutycycle(test_pwm, test_pwm, test_pwm, test_pwm);
		_delay_ms(100);
		
		led_off();
		_delay_ms(100);
	}
}
*/

// TEST: PWM Test 2
void pwm_test_2(void)
{
	pwm_enable();
	int test_pwm = 0;
	pwm_dutycycle_FL = 0;
	pwm_dutycycle_FR = 0;
	pwm_dutycycle_BL = 0;
	pwm_dutycycle_BR = 0;
	
	while(1)
	{
		led_on();
		printf("Enter duty cycle (<100): ");
		test_pwm = USART_Receive_Number();
		led_off();
		pwm_apply_dutycycle(test_pwm, test_pwm, test_pwm, test_pwm);
		_delay_ms(500);
	}
}

// FUNCTION: Initialize PWM pins
void pwm_initialize(void)
{
	// Set mode of operation to FastPWM
	TCCR0A |= (1<<WGM00 | 1<<WGM01);
	TCCR2A |= (1<<WGM20 | 1<<WGM21);
	// Set clock source (prescaler)
	TCCR0B |= (1<<CS01);
	TCCR2B |= (1<<CS21);
	// Set to 0% duty cycle
	OCR0A = 0x00;
	OCR0B = 0x00;
	OCR2A = 0x00;
	OCR2B = 0x00;
	// 4 PWM channel outputs
	DDRB |= 1<<PB3; // OC2A
	DDRD |= 1<<PD3; // OC2B
	DDRD |= 1<<PD5; // OC0B
	DDRD |= 1<<PD6; // OC0A
}

// FUNCTION: Enable PWM channels
void pwm_enable(void) 
{
	TCCR0A |= 1<<COM0A1;
	TCCR0A |= 1<<COM0B1;
	TCCR2A |= 1<<COM2A1;
	TCCR2A |= 1<<COM2B1;
}

// FUNCTION: Disable PWM channels
void pwm_disable() 
{
	TCCR0A &= ~(1<<COM0A1);
	TCCR0A &= ~(1<<COM0B1);
	TCCR2A &= ~(1<<COM2A1);
	TCCR2A &= ~(1<<COM2B1);
}

// FUNCTION: Sets the channel duty cycle
void pwm_apply_dutycycle(uint8_t pwm_FL, uint8_t pwm_FR, uint8_t pwm_BL, uint8_t pwm_BR) 
{
	OCR0A = pwm_FL*255/100;
	OCR0B = pwm_FR*255/100;
	OCR2A = pwm_BL*255/100;
	OCR2B = pwm_BR*255/100;
	printf("Motors[ FL=%d\tFR=%d\tBL=%d\tBR=%d]\n\r",pwm_FL,pwm_FR,pwm_BL,pwm_BR);
}


// ========================= ESC ========================== //

void esc_test(void)
{
	
	
	
}



// ==================== GP2Y0A21 (IR) ===================== //
/*
void GP2Y0A21_test(void)
{
	GP2Y0A21_initialise();
	while(1){
		printf("distance = %d\n\r", (int)(GP2Y0A21_getDistance()*100));
		_delay_ms(500);
	}
}
*/

void GP2Y0A21_initialise(void)
{
	// Select Vref=AVcc
	ADMUX |= (1<<REFS0);
	//set prescaller to 128 and enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
	
}

float GP2Y0A21_getDistance(void)
{
	float distance = 0;
	
	int8_t ADCchannel = 0; //set adc channel to ADC0, pin = PC0
	
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	// wait until ADC conversion is complete
	while( ADCSRA & (1<<ADSC) );
	//int16_t adc_value = ADC;
	
	distance = ADC;
	
	//printf(">>%d\n\r",(int)(distance*100));
	
	distance = (distance*(-0.3142)) + 276.27;
	
	//printf(">%d\n\r",(int)(distance*100));
	
	return distance;
}


// ====================== Ultrasound ====================== //

/*
// TEST: Ultrasound Test
void ultrasound_test()
{
	untrasound_initialise();
	while(1){
		printf("distance = %d\n\r", untrasound_getDistance());
		_delay_ms(100);
	}
}
*/

// FUNCTION: Initialise ultrasound
void untrasound_initialise(void)
{
	
	
}

// FUNCTION: Get distance (in millimeters)
int untrasound_getDistance(void)
{
	int distance = 0;
	
	// Logic goes here
	
	return distance;
}


// ======================== Timer ========================= //

/*
void timer1_test(void) //UNTESTED!!!
{
	timer1_initialise();

	while (1)
	{
		// we have a working Timer
	}
}
*/

void timer1_initialise(void) //UNTESTED!!!
{
	// CLK = 1MHz
	// Timer frequency = 1ms
	// Thus:	prescaler = none
	//			OCR1A
	
	OCR1A = 1000; // OCR1A = (1MHz/1)*0.001 - 1

	TCCR1B |= (1 << WGM12);
	// Mode 4, CTC on OCR1A

	TIMSK1 |= (1 << OCIE1A);
	//Set interrupt on compare match

	TCCR1B &= ~(1<<CS12);
	TCCR1B &= ~(1<<CS11);
	TCCR1B &= ~(1<<CS10);
	// set prescaler to none and start the timer

	sei();
	// enable interrupts
}

ISR (TIMER1_COMPA_vect) //UNTESTED!!!
{
	// action to be done every 1 ms
	led_toggle();
}



// ====================== I2C(TWI) ======================== //


// TEST: TWI Test
/*
void twi_test(void)
{
	pwm_enable();
	int test_pwm = 0;
	
	twi_start();
	printf("TWI Status = %02x h\n\r",twi_getStatus()); //See what prints out here
	
	while(1)
	{
		led_on();
		printf("Enter duty cycle (<100): ");
		test_pwm = USART_Receive_Number();
		led_off();
		pwm_apply_dutycycle(test_pwm, test_pwm, test_pwm, test_pwm);
		_delay_ms(500);
	}
}
*/

// FUNCTION: Initialise TWI pinout
void twi_initialise(void)
{
	//check frequency
	printf("TWI parameters:\tPrescaler=%u, TWBR=%u, SCL_freq=%u\n\r",
		TWI_PRESCALER,TWI_TWBR,((F_CPU)/(16+2*TWI_TWBR*TWI_PRESCALER)));
	
	switch(TWI_PRESCALER){
		case 1: 
			TWSR &= ~(1<<TWPS1);
			TWSR &= ~(1<<TWPS0);
			break;
			
		case 4:
			TWSR &= ~(1<<TWPS1);
			TWSR |= (1<<TWPS0);
			break;
			
		case 16:
			TWSR |= (1<<TWPS1);
			TWSR &= ~(1<<TWPS0);
			break;
		
		case 64:
			TWSR |= (1<<TWPS1);
			TWSR |= (1<<TWPS0);
			break;
		
		default:
			printf("unknown prescaler");
	}
	
	TWBR = TWI_TWBR;

	//enable TWI
	TWCR = (1<<TWEN);
	
}

// FUNCTION: Start TWI communication
void twi_start(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	//For start we need to set TWSTA and for stop TWSTO bits along with TWINT and TWEN bits. After start signal is sent we need to wait for status (until TWINT resets to zero).
}

// FUNCTION: Stop TWI communication
void twi_stop(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

// FUNCTION: Write data through TWI
void twi_write(uint8_t u8data)
{
	TWDR = u8data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	//it writes data byte to TWDR register which is shifted to SDA line. It is important to wait for transmission complete within while loop. After which status can be read from status register TWSR.
}

// FUNCTION: Read byte from TWI and send ACK
uint8_t twi_readACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
	//transmits ACK signal after byte transfer
}

// FUNCTION: Read byte from TWI with NACK
uint8_t twi_readNACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

// FUNCTION: Get TWI status
uint8_t twi_getStatus(void)
{
	return TWSR & 0xF8; // return first 5 bits in TWSR register. if == 0x08, then error
}

uint8_t twi_readRegister(uint8_t slave_address_write,uint8_t slave_address_read, uint8_t reg)
{
	twi_start();
	twi_write(slave_address_write);
	twi_write(reg);
	twi_start();
	twi_write(slave_address_read);
	uint8_t data = twi_readNACK();
	twi_stop();
	return data;
}

void twi_WriteRegister(uint8_t slave_address_write,uint8_t reg, uint8_t data)
{
	twi_start();//printf("%02x h\n\r",twi_getStatus());
	twi_write(slave_address_write);//printf("%02x h\n\r",twi_getStatus());
	twi_write(reg);//printf("%02x h\n\r",twi_getStatus());
	twi_write(data);//printf("%02x h\n\r",twi_getStatus());
	twi_stop();//printf("%02x h\n\r",twi_getStatus());
}

// =============== ADXL345 (Accelerometer) ================ //
/*
void ADXL345_test(void)
{
	float accelerometer[3]={0.0, 0.0, 0.0}; //[x-axis, y-axis, z-axis] unit (um/s2)
	
	ADXL345_initialise();
	
	while(1)
	{
		ADXL345_ReadAccelerometer(&accelerometer);
		
		printf("%d\t%d\t%d \tmm/s^2\n\r",(int16_t)(accelerometer[0]*1000),(int16_t)(accelerometer[1]*1000),(int16_t)(accelerometer[2]*1000));
		
		
		//Test other registers
		//printf("__device_id = %u\n\r",	ADXL345_ReadRegister(ADXL345_DEVID_R)); // Device ID
		//printf("__thresh_tap = %u\n\r",	ADXL345_ReadRegister(ADXL345_THRESH_TAP_RW)); // tap threshold
		//printf("__ofsx = %u\n\r",		ADXL345_ReadRegister(ADXL345_OFSX_RW)); // X-axis offset
		//printf("__ofsy = %u\n\r",		ADXL345_ReadRegister(ADXL345_OFSY_RW)); // Y-axis offset
		//printf("__ofsz = %u\n\r",		ADXL345_ReadRegister(ADXL345_OFSZ_RW)); // Z-axis offset
		//printf("__data format = %u\n\r",ADXL345_ReadRegister(ADXL345_DATA_FORMAT_RW)); // Data format control
		//printf("__fifo_status = %u\n\r",ADXL345_ReadRegister(ADXL345_FIFO_CTL_RW)); // fifo control
		//printf("__fifo_ctrl = %u\n\r",	ADXL345_ReadRegister(ADXL345_FIFO_STATUS_R)); // fifo status
		//printf("__x0 = %u\n\r",			ADXL345_ReadRegister(ADXL345_DATAX0_R)); // x0
		//printf("__x1 = %u\n\r",			ADXL345_ReadRegister(ADXL345_DATAX1_R)); // x1
		//printf("__bw_rate = %u\n\r",	ADXL345_ReadRegister(ADXL345_BW_RATE)); // data rate and power mode control
		//printf("__int_source = %u\n\r",	ADXL345_ReadRegister(ADXL345_INT_SOURCE)); // source of interrupts
		//printf("__pwr_ctrl = %u\n\r",	ADXL345_ReadRegister(ADXL345_POWER_CTRL)); //Power-saving feature control
		
		
		_delay_ms(500);
	}
	
	twi_stop();
}
*/


void ADXL345_initialise(void)
{
	//When registers are written, there's no need to rewrite over it because the 
	//value is still there when the program starts up.
	
	//ADXL345_WriteRegister(ADXL345_DATA_FORMAT_RW, 0);// Set range to + - 2g
	twi_WriteRegister(ADXL345_ADDRESS_W,ADXL345_POWER_CTRL, 0x08);// Change to measurement mode
}

/*
void ADXL345_WriteRegister(uint8_t reg, uint8_t data)
{
	twi_start();//printf("%02x h\n\r",twi_getStatus());
	twi_write(ADXL345_ADDRESS_W);//printf("%02x h\n\r",twi_getStatus());
	twi_write(reg);//printf("%02x h\n\r",twi_getStatus());
	twi_write(data);//printf("%02x h\n\r",twi_getStatus());
	twi_stop();//printf("%02x h\n\r",twi_getStatus());
}
*/

/*
uint8_t ADXL345_ReadRegister(uint8_t reg)
{
	twi_start();
	twi_write(ADXL345_ADDRESS_W);
	twi_write(reg);
	twi_start();
	twi_write(ADXL345_ADDRESS_R);
	uint8_t data = twi_readNACK();
	twi_stop();
	return data;
}
*/

void ADXL345_ReadAccelerometer(float* accelerometer)
{
	twi_start(); //printf("%02x h\n\r",twi_getStatus());
	twi_write(ADXL345_ADDRESS_W);//printf("%02x h\n\r",twi_getStatus());
	twi_write(ADXL345_DATAX0_R);//printf("%02x h\n\r",twi_getStatus());
	twi_start();//printf("%02x h\n\r",twi_getStatus());
	twi_write(ADXL345_ADDRESS_R);//printf("%02x h\n\r",twi_getStatus());
	uint8_t x0 = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t x1 = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t y0 = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t y1 = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t z0 = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t z1 = twi_readNACK();//printf("%02x h\n\r",twi_getStatus());
	twi_stop();//printf("%02x h\n\r",twi_getStatus());
	
	// Convert to int
	int32_t x = (int32_t)((x1 << 8) | (x0 & 0xff));
	int32_t y = (int32_t)((y1 << 8) | (y0 & 0xff));
	int32_t z = (int32_t)((z1 << 8) | (z0 & 0xff));
	
	//printf("[%ld\t%ld\t%ld]\n\r",x,y,z);
	
	accelerometer[0] = (float)x * ADXL345_SCALE_FACTOR*9.81;
	accelerometer[1] = (float)y * ADXL345_SCALE_FACTOR*9.81;
	accelerometer[2] = (float)z * ADXL345_SCALE_FACTOR*9.81;
	
	return 0;
}

// ================= ITG3200 (Gyroscope) ================== //

/*
void ITG3200_test(void)
{
	ITG3200_initialise();
	float gyro [4] = {0.0, 0.0, 0.0, 0.0};
	
	while(1){
		ITG3200_ReadGyro(&gyro);
		//printf("%d\ttemp\t%d\t%d\t%d\tdeg/s\n\r",(int32_t)(gyro[0]*1000),(int_t)(gyro[1]*1000),(int32_t)(gyro[2]*1000),(int16_t)(gyro[3]*1000));
		_delay_ms(500);
	}
}
*/

void ITG3200_initialise(void)
{
	
}

void ITG3200_ReadGyro(float* gyro)
{
	float rot_sensitivity = 14.375;
	float temp_sensitivity = 280.0;
	
	twi_start(); //printf("%02x h\n\r",twi_getStatus());
	twi_write(ITG3200_ADDRESS_W);//printf("%02x h\n\r",twi_getStatus());
	twi_write(ITG3200_TEMP_H);//printf("%02x h\n\r",twi_getStatus());
	twi_start();//printf("%02x h\n\r",twi_getStatus());
	twi_write(ITG3200_ADDRESS_R);//printf("%02x h\n\r",twi_getStatus());
	uint8_t th = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t tl = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t xh = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t xl = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t yh = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t yl = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t zh = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t zl = twi_readNACK();//printf("%02x h\n\r",twi_getStatus());
	twi_stop();//printf("%02x h\n\r",twi_getStatus());
	
	// Convert to int
	int32_t t = (int32_t)((th << 8) | (tl & 0xff));
	int32_t x = (int32_t)((xh << 8) | (xl & 0xff));
	int32_t y = (int32_t)((yh << 8) | (yl & 0xff));
	int32_t z = (int32_t)((zh << 8) | (zl & 0xff));
	
	//printf("[%ld\t%ld\t%ld\t%ld]\n\r",t,x,y,z);
	
	gyro[0] = ((float)t)/temp_sensitivity;
	gyro[1] = ((float)x)/rot_sensitivity;
	gyro[2] = ((float)y)/rot_sensitivity;
	gyro[3] = ((float)z)/rot_sensitivity;
	
	//printf("ROT\t%d\ttemp\t%d\t%d\t%d\tdeg/s\n\r",(int32_t)(gyro[0]*1000),(int32_t)(gyro[1]*1000),(int32_t)(gyro[2]*1000),(int16_t)(gyro[3]*1000));
	//printf("ROT\t%d\t%d\t%d\tdeg/s\n\r",(int32_t)(gyro[1]*1000),(int32_t)(gyro[2]*1000),(int16_t)(gyro[3]*1000));
	//printf("ROT\t%d\t%d\t%d\tdeg/s\n\r",(int32_t)gyro[1],(int32_t)gyro[2],(int16_t)gyro[3]);

}

// =============== HMC5883L (Magnetometer) ================ //

/*
void HMC5883L_test(void)
{
	HMC5883L_initialise();
	float mag [3] = {0.0, 0.0, 0.0};
	
	printf("config_a=%d\n\r",twi_readRegister(HMC5883L_ADDRESS_W,HMC5883L_ADDRESS_R,HMC5883L_CONFIG_A));
	printf("config_b=%d\n\r",twi_readRegister(HMC5883L_ADDRESS_W,HMC5883L_ADDRESS_R,HMC5883L_CONFIG_B));
	printf("mode=%d\n\r",twi_readRegister(HMC5883L_ADDRESS_W,HMC5883L_ADDRESS_R,HMC5883L_MODE));
	
	
	while(1){
		HMC5883L_ReadMag(&mag);
		//printf("%d\t%d\t%d\n\r",(int16_t)(mag[0]*1000),(int16_t)(mag[1]*1000),(int32_t)(mag[2]*1000));
		_delay_ms(500);
	}
}
*/

void HMC5883L_initialise(void)
{
	// Start continuous operation
	twi_WriteRegister(HMC5883L_ADDRESS_W, HMC5883L_MODE, 0x00);
}

void HMC5883L_ReadMag(float* mag)
{
	twi_start(); //printf("%02x h\n\r",twi_getStatus());
	twi_write(HMC5883L_ADDRESS_W);//printf("%02x h\n\r",twi_getStatus());
	twi_write(HMC5883L_DATAX_H);//printf("%02x h\n\r",twi_getStatus());
	twi_start();//printf("%02x h\n\r",twi_getStatus());
	twi_write(HMC5883L_ADDRESS_R);//printf("%02x h\n\r",twi_getStatus());
	uint8_t xh = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t xl = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t yh = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t yl = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t zh = twi_readACK();//printf("%02x h\n\r",twi_getStatus());
	uint8_t zl = twi_readNACK();//printf("%02x h\n\r",twi_getStatus());
	twi_stop();//printf("%02x h\n\r",twi_getStatus());
	
	// Convert to int
	int32_t x = (int32_t)((xh << 8) | (xl & 0xff));
	int32_t y = (int32_t)((yh << 8) | (yl & 0xff));
	int32_t z = (int32_t)((zh << 8) | (zl & 0xff));
	
	printf("[%ld\t%ld\t%ld]\n\r",x,y,z);
	
	mag[0] = (float)x;
	mag[1] = (float)y;
	mag[2] = (float)z;
}

// ============= MPU9150 (Mag + Acc + Gyro) =============== //

void MPU9150_test(void)
{
	Mpu9150_data_t data;
	MPU9150_initialise(&data);
	while (1)
	{
		MPU9150_readData(&data);
		_delay_ms(100);
	}
}

void MPU9150_initialise(Mpu9150_data_t *data)
{
	data->acc_x = 0.0;
	data->acc_y = 0.0;
	data->acc_z = 0.0;
	data->gyro_x = 0.0;
	data->gyro_y = 0.0;
	data->gyro_z = 0.0;
	data->mag_x = 0.0;
	data->mag_y = 0.0;
	data->mag_z = 0.0;
	
}

void MPU9150_readData(Mpu9150_data_t *data)
{
	
	
}

// ================= Radio Frequency(RF) ================== //

/*
void radio_test(void) //UNTESTED!!!
{
	
	
	
}
*/

void radio_initialise(void) //UNTESTED!!!
{
	
	
	
	
}