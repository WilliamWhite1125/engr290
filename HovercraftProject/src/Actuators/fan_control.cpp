#include <avr/io.h>
#include <util/delay.h>

// Constants
#define F_CPU 16000000UL         // Clock speed

// Pin Definitions
#define LED_L_PIN PB5            // Onboard LED
#define HOVER_FAN_PIN PD4        // Hover fan (Fan 1) - On/Off
#define PROPULSION_FAN_PIN PD6   // Propulsion fan (Fan 2) - PWM (0C0A)

// Function Prototypes
void startHoverFan();
void stopHoverFan();
void startPropulsionFan(uint8_t dutyCycle);
void stopPropulsionFan();

/*
int main() {
    // Initialize pins
    DDRD |= (1 << HOVER_FAN_PIN) | (1 << PROPULSION_FAN_PIN); // Set PD4 and PD6 as outputs

    while (1) {
        startHoverFan();              // Turn on hover fan
        startPropulsionFan(128);      // Set propulsion fan to 50% duty cycle
        _delay_ms(2000);

        stopHoverFan();               // Turn off hover fan
        stopPropulsionFan();          // Stop propulsion fan
        _delay_ms(2000);
    }
}
*/

// Hover Fan (PD4) Control
void startHoverFan() {
    PORTD |= (1 << HOVER_FAN_PIN); // Set PD4 high to turn on hover fan
}

void stopHoverFan() {
    PORTD &= ~(1 << HOVER_FAN_PIN); // Set PD4 low to turn off hover fan
}

// Propulsion Fan (PD6) Control Using Timer 1
void startPropulsionFan(uint8_t dutyCycle) {
    static uint8_t timerInitialized = 0;
    if (!timerInitialized) {
        // Configure Timer 1 for Fast PWM, 8-bit mode
        TCCR1A |= (1 << WGM10);       // Fast PWM, 8-bit
        TCCR1A |= (1 << COM1A1);      // Non-inverting mode on 0C0A (PD6)
        TCCR1B |= (1 << WGM12);       // Fast PWM, part 2
        TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64 (PWM ~490 Hz)

        timerInitialized = 1; // Mark Timer 1 as initialized
    }
    0CR0A = dutyCycle; // Set duty cycle for propulsion fan (0-255)
}

void stopPropulsionFan() {
    0CR0A = 0; // Set duty cycle to 0 to stop propulsion fan
}
