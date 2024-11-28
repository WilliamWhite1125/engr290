#include <avr/io.h>
#include <util/delay.h>

// Pin Definitions
#define LED_PIN PB5            // Onboard LED
#define HOVER_FAN_PIN PD4        // Hover fan (Fan 1) - On/Off
#define PROPULSION_FAN_PIN PD6   // Propulsion fan (Fan 2) - PWM (0C0A)
#define TRIG_PIN PB3
#define ECHO_PIN PD2
#define SERVO_PIN PD5             // Servo motor control pin (Arduino pin 5)
#define SDA_PIN PC4               // SDA connected to PC4 (Arduino A4)
#define SCL_PIN PC5               // SCL connected to PC5 (Arduino A5)
#define IR_PIN PC0  

// Constants
#define F_CPU 16000000UL         // Clock speed

#define IMU_ADDRESS 0x68          // MPU-6050 I2C address
#define I2C_DELAY_US 5            // I2C communication delay in microseconds
#define MIN_YAW -85               // Minimum yaw angle
#define MAX_YAW 85               // Maximum yaw angle

#define MIN_SERVO_ANGLE 0         // Minimum servo angle
#define MAX_SERVO_ANGLE 180       // Maximum servo angle

#define GYRO_SENSITIVITY 131.0    // Gyroscope sensitivity scale factor
#define ACCEL_SCALE 16384.0       // Accelerometer sensitivity scale factor

#define GOAL_LOW 20  // Adjust based on IR sensor data
#define GOAL_HIGH 50  // Adjust based on IR sensor data
#define DISTANCE_THRESHOLD 30  //cm //adjust based on testing results

// Define states
enum State { INIT, IDLE, SCAN, MOVE, GOAL, STOP };
State currentState = INIT;
enum Status{SUCCESS, FAILURE}; //could use this to check if the setup is complete
Status initStatus=FAILURE;

//global variables
volatile unsigned long milliseconds = 0; // Millisecond counter for timing
float servo_angle = 90.0; // Initialize to middle position

// Function Prototypes
void startHoverFan();
void stopHoverFan();
void startPropulsionFan(uint8_t dutyCycle);
void stopPropulsionFan();
void setupSystem();         //COPIED FROM SETUPS IN DEMOS, MAYBE MISSING SOME
void handleState();         //DONE
void scanEnvironment();     //DONE
bool isGoalDetected();      //DONE
bool isObstacleDetected();  //DONE
void setup();
void handleState();
void scanEnvironment();
bool isObstacleDetected();
long measureUltrasonicDistance();
void servo_write(uint16_t angle);
void setupPWM();

void setup(){
  // Initialize pins
  DDRD |= (1 << HOVER_FAN_PIN) | (1 << PROPULSION_FAN_PIN); // Set PD4 and PD6 as outputs
  DDRB |= (1 << TRIG_PIN);  // TRIG_PIN output
  DDRD &= ~(1 << ECHO_PIN); // ECHO_PIN input
  DDRC &= ~(1 << IR_PIN); //ir pin input
  DDRD |= (1 << PD5); // Set PD5 (Arduino pin 5) as output for servo control

  //timer 2
  TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  
  TCCR2B |= (1 << CS22);

 //start servo in the middle
  servo_write((uint16_t)servo_angle);
   _delay_ms(4000);  // Allow sensor to stabilize
}

int main() {

    while (1) {
        //main code
    }
}
void handleState() {
 switch (currentState) {

    case INIT:  //DONE
      //Set up sensors and actuators (IMU, fans, servo, IR, ultrasonic sensor).
      while(initStatus == FAILURE){
        setupSystem();
      if(initStatus == SUCCESS){
        currentState = IDLE;
      }
      }
      break;

    case IDLE:  //DONE
      stopPropulsionFan();
      stopHoverFan();
      _delay_ms(1000);  // Wait for stability
      //switches to scan when it stops moving
      currentState = SCAN;
      break;

    case SCAN:  //DONE
                // Scan the environment to find the best path forward
      scanEnvironment();
      // Transition to MOVE if a viable path is found
      currentState = MOVE;
      break;

    case MOVE:  //DONE
                // Start fans and move forward
      startHoverFan();
      startPropulsionFan(255); //max speed fan
      _delay_ms(4000);  // Wait ~ 4s to not immediately cycle back to scan

      // Monitor for obstacles and transition accordingly
      if (isObstacleDetected()) {
        currentState = SCAN;
      } else if (isGoalDetected()) {
        currentState = STOP;
      }
      break;

    case GOAL:  //TODO, OR NOT
    //Use the IR sensor to detect the bar above the hovercraft at the maze's end.
    // Detect the goal and stop under the bar
      break;

    case STOP:  //DONE
      // Halt all systems and indicate completion
      stopPropulsionFan();
      stopHoverFan();

      //blink lights and end program
      while(1){
        PORTB |= (1 << LED_PIN);
        _delay_ms(100);
        PORTB &= ~(1 << LED_PIN);
        _delay_ms(100);
      }
      break;
  }
}
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
        TCCR0A |= (1 << WGM00);       // Fast PWM, 8-bit
        TCCR0A |= (1 << WGM01);       // Fast PWM, part 2
        TCCR0A |= (1 << COM0A1); // Non-inverting mode on 0C0A (PD6)
        TCCR0A |= (1 << COM0B1); 
  
        TCCR0B |= (1 << WGM02);  
        TCCR0B |= (1 << CS01) | (1 << CS00); // Prescaler 64 (PWM ~490 Hz)

        timerInitialized = 1; // Mark Timer 1 as initialized
    }
    OCR0A = dutyCycle; // Set duty cycle for propulsion fan (0-255)
}

void stopPropulsionFan() {
    OCR0A = 0; // Set duty cycle to 0 to stop propulsion fan
}
bool isGoalDetected() {
  // Use IR sensor to check for the goal
  float irReading = measureIRDistance();  // Assume measureIR() returns the IR sensor reading
  return (irReading > GOAL_LOW && irReading < GOAL_HIGH); //minumim reading is 20cm
}

bool isObstacleDetected() {
  // Check if the ultrasonic sensor detects an obstacle within threshold
  float distance = measureUltrasonicDistance();
  return distance < DISTANCE_THRESHOLD;
}
void delay_ms(unsigned int ms) {
    unsigned long start = customMillis();
    while (customMillis() - start < ms);
}

void delay_us(unsigned int us) {
    while (us--) {
        _delay_us(1); // Use built-in function for accuracy
    }
}
unsigned long customMillis() {
    unsigned long ms;
    cli();
    ms = milliseconds;
    sei();
    return ms;
}
void servo_write(uint16_t angle) {
    // Constrain the angle within 0 to 180 degrees
    angle = constrain(angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

    // Convert angle to pulse width (1 ms to 2 ms)
    uint16_t pulse_width_us = ((angle * 1000UL) / 180UL) + 1000UL; // 1000us to 2000us

    // Generate the servo pulse
    // Start of pulse
    PORTD |= (1 << PD5);
    delay_us(pulse_width_us); // Keep pin high for the duration of the pulse

    // End of pulse
    PORTD &= ~(1 << PD5);

    // Wait for the remainder of the 20 ms period
    unsigned long pulse_duration_ms = pulse_width_us / 1000;
    if (pulse_duration_ms < 20) {
        delay_ms(20 - pulse_duration_ms);
    }
}
long measureUltrasonicDistance() {
    long duration; 
    int distance;  

    PORTB &= ~(1 << TRIG_PIN); 
    _delay_us(2); 
    PORTB |= (1 << TRIG_PIN); 
    _delay_us(10); 
    PORTB &= ~(1 << TRIG_PIN); 

    // Measure the duration of the echo
    duration = pulseIn(ECHO_PIN, HIGH); 
    distance = duration * 0.0344 / 2; 

    return distance; 
}
float measureIRDistance() {
    ADMUX = (1 << REFS0) | (IR_PIN & 0x0F); 
    ADCSRA |= (1 << ADSC);  
    while (ADCSRA & (1 << ADSC));  

    int reading = ADC;

    // Convert reading to voltage and then to distance
    float voltage = reading * (5.0 / 1024.0); 
    //float distance = 29.988 * pow(voltage, -1.173); 
  	float distance = 75/(voltage-1.25);

    return distance;
}
