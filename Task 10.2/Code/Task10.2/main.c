/*
 * Task10.2.c
 *
 * Created: 8/31/2024 11:09:32 AM
 * Author : Ahmed Ezzat
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include<util/delay.h>

//#define F_CPU 16000000UL					// Clock of micro controller (16MHz)
#define ENCODER_PIN PD2					   // Connect encoder with interrupt pin INT0				
#define SPEED_CONTROL_POT PC2			  // Potentiometer to control speed of motor (must be ADC) 
#define DRIVER_FORWARD PD6			     //  Driver pin for motor move forward
#define SS PB2						 
#define MOSI PB3
#define MISO PB4
#define SCK PB5

volatile uint16_t Encoder_Count;
volatile uint16_t Motor_Speed;


/**
  * @brief   Function for initialize ADC
  * @param   ------
  * @retval  void
  */
void adc(){
	ADMUX = (1 << REFS0);									 // AVcc with AREF pin
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS2);		// Enable ADC at Prescaler 64
}


/**
  * @brief   Function for reading data from ADC
  * @param   Channel_Selector
  * @retval  uint16 ADC
  */
uint16_t adc_Read(uint8_t Channel_Selector){
	ADMUX &= 0xF0;			           // Clear the older channel that was selected
	ADMUX |= Channel_Selector;
	ADCSRA |= (1 << ADSC);           // Start conversion
	while((ADCSRA) & (1 << ADSC));  // Wait for conversion to finish
	return ADC;					   // Return the ADC value

}


/**
  * @brief   Function for initialize PWM
  * @param   ------
  * @retval  void
  */
void pwm(){
	TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1); // Set Fast PWM mod and non-inverted
	TCCR0B = (1 << CS01);								// Prescaler 8
	DDRD |= (1 << DRIVER_FORWARD);					   // Make PD6 as output
}


/**
  * @brief   Function for make PWM duty
  * @param   duty
  * @retval  void
  */
void pwm_Duty(uint8_t duty){
	OCR0A = duty;
}




/**
  * @brief   Function for initialize encoder
  * @param   ------
  * @retval  void
  */
void Encoder(){
	EICRA = (1 << ISC00);    // Generates an interrupt request
	EIMSK = (1 << INT0);    // Enable INT0
	sei();				   // Enable global interrupt
}



/**
  * @brief   Function for external interrupt service routine for encode
  * @param   ------
  * @retval  void
  */
ISR(INT0_vect){
	Encoder_Count++;
}




/**
  * @brief   Function for Calculating speed
  * @param   ------
  * @retval  void
  */
void RPM_Calculation(){
	Motor_Speed = (Encoder_Count * 60);   // One pulse per revolution
	Encoder_Count = 0;                   // Reset Encoder Counter
}



/**
  * @brief   Function for initialize SPI
  * @param   ------
  * @retval  void
  */
void SPI(){
	DDRB = (1 << SS) | (1 << MOSI) | (1 << SCK);      // Make MOSI, SCK ,SS as output
	DDRB &= ~(1 << MISO);                            // Make MISO as input
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);  // Enable SPI, Master mode and FOSC/16
}



/**
  * @brief   Function for initialize SPI
  * @param   uint8_t data that will be sen
  * @retval  void
  */
void SPI_Data(uint8_t data){
	SPDR = data;
	while (! (SPSR & (1 << SPIF)));  // Wait for transmission complete
}


int main(void)
{
	adc();
	pwm();
	Encoder();
	SPI();
	
    while (1) 
    {	 
		uint16_t Potentiometer_value = adc_Read(SPEED_CONTROL_POT);		 // Read the potentiometer value
		uint8_t PWM_duty_read = Potentiometer_value / 4;			    // Scale 10-bit ADC value to 8-bit PWM
		
		
		pwm_Duty(PWM_duty_read);								     // Set PWM duty cycle based on potentiometer
		
		_delay_ms(1000);										   // Wait for 1 second		          
		RPM_Calculation();
		
		SPI_Data(Motor_Speed >> 8);								// Send high value
		SPI_Data(Motor_Speed & 0xFF);						   // Send low value
	}
}
