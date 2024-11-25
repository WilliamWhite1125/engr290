#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

/*13.Be certain to connect an “over 4.7µF” capacitor to the fan externally when the application calls
for using multiple fans in parallel, to avoid any unstable power.
https://www.delta-fan.com/Download/Spec/AFB1212SH.pdf*/

// Constants
#define F_CPU 16000000UL         // Clock speed

// Pin Definitions
#define SDA_PIN PC4              // I2C SDA
#define SCL_PIN PC5              // I2C SCL
#define LED_L_PIN PB5            // Onboard LED
#define HOVER_FAN_PIN PB1        // Hover fan(FAN #1) WITH TIMER 1
#define PROPULSION_FAN_PIN PB2   // Propulsion fan(FAN #2) WITH TIMER 1

// Timer Prescaler
#define TIMER0_PRESCALER 64

// Function Prototypes
void startHoverFan();
void startPropulsionFan();
void stopFan();

//function implementations 
//ALL ARE THE SAME RN
void startFan(uint8_t fan, uint8_t dutyCycle) {
     // Configure Timer1 if not already done
    static uint8_t timerInitialized = 0;
    if (!timerInitialized) {
        // Set Timer1 to Fast PWM mode with 8-bit resolution
        TCCR1A |= (1 << WGM10); // Fast PWM, 8-bit
        TCCR1A |= (1 << WGM11); // Fast PWM, 8-bit
        TCCR1B |= (1 << WGM12); // Fast PWM, 8-bit

        // Enable non-inverting mode for both OC1A and OC1B
        TCCR1A |= (0 << COM1A0) | (0 << COM1B0);
        TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
       

        // Set clock prescaler to 64 (PWM frequency ~490 Hz)
        TCCR1B |= (1 << CS11) | (1 << CS10);

        // Set OC1A (PB1) and OC1B (PB2) as output pins
        DDRB |= (1 << PB1) | (1 << PB2);

        timerInitialized = 1; // Mark the timer as initialized
    }

    // Set the duty cycle for the specified fan
    if (fan == 1) {
        OCR1A = dutyCycle; // Fan 1 on PB1 (OC1A)
    } else if (fan == 2) {
        OCR1B = dutyCycle; // Fan 2 on PB2 (OC1B)
    }
}
void stopFan(uint8_t fan) {
    if (fan == 1) {
        OCR1A = 0; // Stop Fan 1 by setting duty cycle to 0%
    } else if (fan == 2) {
        OCR1B = 0; // Stop Fan 2 by setting duty cycle to 0%
    }
}
