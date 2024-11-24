#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

// Constants
#define F_CPU 16000000UL         // Clock speed

// Pin Definitions
#define SDA_PIN PC4              // I2C SDA
#define SCL_PIN PC5              // I2C SCL
#define LED_L_PIN PB5            // Onboard LED
#define LED_D3_PIN PB3           // PWM LED 
#define HOVER_FAN_PIN PD6        // Hover fan ???
#define PROPULSION_FAN_PIN PD5   // Propulsion fan ???

// Timer Prescaler
#define TIMER0_PRESCALER 64

// Function Prototypes
void startHoverFan();
void startPropulsionFan();
void stopFan();

//function implementations 
//ALL ARE THE SAME RN
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
