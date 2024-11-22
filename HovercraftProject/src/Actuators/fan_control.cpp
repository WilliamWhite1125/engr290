#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

// Constants and Macros
#define F_CPU 16000000UL          // 16 MHz clock speed
#define IMU_ADDRESS 0x68          // MPU-6050 I2C address
#define I2C_DELAY_US 5            // I2C communication delay in microseconds

#define SDA_PIN PC4               // SDA connected to PC4 (Arduino A4)
#define SCL_PIN PC5               // SCL connected to PC5 (Arduino A5)

#define LED_L_PIN PB5             // LED "L" pin (Arduino pin 13)
#define LED_D3_PIN PB3            // LED D3 pin (OC2A, Arduino pin 11)

//function prototypes
void startFan();
void stopFan();

//function implementations 
//BOTH ARE THE SAME RN
void startHoverFan(){
    TCCR0A|=(1<<COM0A1); // non-inverted pin operation
    TCCR0A|=(1<<WGM00); // PWM, Phase Correct
    OCR0A=0; // D=0, i.e. the fan is OFF. OCRA=255 will turn the fan ON at 100% of its power.
    TCCR0B|=((1<<CS01)|(1<<CS00)); // Prescaler=64. Start the timer.
}
void startPropulsionFan(){
    TCCR0A|=(1<<COM0A1); // non-inverted pin operation
    TCCR0A|=(1<<WGM00); // PWM, Phase Correct
    OCR0A=0; // D=0, i.e. the fan is OFF. OCRA=255 will turn the fan ON at 100% of its power.
    TCCR0B|=((1<<CS01)|(1<<CS00)); // Prescaler=64. Start the timer.
}
void stopFan(){
    TCCR0A|=(1<<COM0A1); // non-inverted pin operation
    TCCR0A|=(1<<WGM00); // PWM, Phase Correct
    OCR0A=0; // D=0, i.e. the fan is OFF. OCRA=255 will turn the fan ON at 100% of its power.
    TCCR0B|=((1<<CS01)|(1<<CS00)); // Prescaler=64. Start the timer.
}
