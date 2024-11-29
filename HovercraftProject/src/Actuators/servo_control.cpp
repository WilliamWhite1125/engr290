extern "C" {
    #include "init_290.h"
    #include "TWI_290.h"  // Include TWI library for I2C communication
}

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define F_CPU 16000000UL // CPU frequency
#define BAUD 9600 // Desired baud rate
#define MYUBRR F_CPU/16/BAUD-1 // UBRR calculation

// Initialize PWM for servo control and D3
void pwm_init() {
    DDRB |= (1 << PORTB1); // Set PB1 as output
    TCCR1A  |= (1 << COM1A1) | (1 << COM1B1); // non-inv PWM on channels A and B
    TCCR1B |= (1 << WGM13); // Phase and Frequency Correct PWM
    ICR1 = 39999; // Set TOP value for 50Hz PWM (for servo control)
    TCCR1B |= ((1 << CS11) | (1 << CS10)); // Prescaler 8, starts PWM
}
ISR(TIMER0_COMPA_vect) {
  timer_millis++; // Increment the millisecond counter
}

unsigned long millis2() {
  return timer_millis; // Return the current millisecond count
}

void setup_timer() {
    TCCR0A |= (1 << WGM01); // Set Timer0 to CTC mode
    TCCR0B |= (1 << CS01);  // Set prescaler to 8
    OCR0A = 199;            // Set compare value for 1 ms
    TIMSK0 |= (1 << OCIE0A);// Enable compare interrupt
    sei();                  // Enable global interrupts
}

void update_servo(const char* direction) {
  uint16_t servo_index;

  // Check the string input and set the servo position based on direction
  if (strcmp(direction, "straight") == 0) {
    // Servo points straight
    servo_index = 185;  
  } else if (strcmp(direction, "left") == 0) {
    // Servo points left for a left turn
    servo_index = 253;  
  } else if (strcmp(direction, "right") == 0) {
    // Servo points right for a right turn
    servo_index = 85;  
  } else {
    // Default behavior if the direction is not recognized
    servo_index = 169.5;  // Set to straight if invalid input
  }

  // Ensure the servo position is within valid range
  if (servo_index < 85) servo_index = 85;
  if (servo_index > 253) servo_index = 253;

  // Set the PWM value for the servo motor
  OCR1A = servo_index;  // Update PWM for the servo motor
}
void setup() {
  UART_init(); // Initialize UART
  ultrasonic_init();  // Initialize US pins and interrupts
  setup_timer();
  UART_puts("System Initializing...\r\n");
  _delay_ms(200);
  pwm_init(); // Initialize PWM
  _delay_ms(200);
  previousTime = millis2();
  
  UART_puts("System Initialized...\r\n");
  _delay_ms(1000);
}
int main(){
  setup();

  while(1){
    //1. Initial Navigation Check
    //Servo faces straight ahead and fans turn full power
    direction = "straight";
    update_servo(direction);


    for(int i=0; i<15; i++){
      direction = "left";
    update_servo(direction);
      direction = "straight";
    update_servo(direction);
      direction = "right";
    update_servo(direction);
      direction = "straight";
    update_servo(direction);
    }
  }
}

