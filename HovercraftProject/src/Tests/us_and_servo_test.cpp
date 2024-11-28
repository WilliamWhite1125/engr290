#include <util/delay.h>
#include <avr/io.h>

#define TRIG_PIN PB3
#define ECHO_PIN PD2
#define SERVO_PIN PD5             // Servo motor control pin (Arduino pin 5)

#define MIN_SERVO_ANGLE 0         // Minimum servo angle
#define MAX_SERVO_ANGLE 180       // Maximum servo angle
#define DISTANCE_THRESHOLD 30     //(cm)  adjust based on testing results

#define constrain(x, a, b) ((x)<(a)?(a):((x)>(b)?(b):(x)))

// Global variables
volatile unsigned long milliseconds = 0; // Millisecond counter for timing
float servo_angle = 90.0; // Initialize to middle position
enum State { INIT, IDLE, SCAN, MOVE };
State currentState = INIT;

// Function Prototypes
void setup();
void handleState();
void scanEnvironment();
bool isObstacleDetected();
long measureUltrasonicDistance();
void servo_write(uint16_t angle);
void setupPWM();

//functions
void setup(){
  Serial.begin(9600);
  Serial.println("CURRENT STATE INIT");
    
  // Set TRIG_PIN as output, ECHO_PIN as input
  DDRB |= (1 << TRIG_PIN);  // TRIG_PIN output
  DDRD &= ~(1 << ECHO_PIN); // ECHO_PIN input
  
  // Setup Servo pin
  DDRD |= (1 << PD5); // Set PD5 (Arduino pin 5) as output for servo control
  servo_write((uint16_t)servo_angle);
   _delay_ms(4000);  // Allow sensor to stabilize
  currentState = IDLE;
}
//***************************//
//****MAIN FUNCTION**********//
//***************************//
int main() {
    setup();
    while (1) {
        handleState();
    }
    return 0; // Not reached
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
void handleState() {

  switch (currentState) {

    case IDLE:  //DONE
      Serial.println("CURRENT STATE IDLE");
      Serial.println("StopFans()");
      _delay_ms(1000);  // Wait for stability
      //switches to scan when it stops moving
      currentState = SCAN;
      break;

    case SCAN:  //DONE
      Serial.println("CURRENT STATE SCAN");
                // Scan the environment to find the best path forward
      scanEnvironment();
      // Transition to MOVE if a viable path is found
      currentState = MOVE;
      break;

    case MOVE:  //DONE
      Serial.println("CURRENT STATE MOVE");
                // Start fans and move forward
      Serial.println("startHoverFan()");
      Serial.println("startPropulsionFan()");
      _delay_ms(4000);  // Wait ~ 4s to not immediately cycle back to scan

      // Monitor for obstacles and transition accordingly
      if (isObstacleDetected()) {
        Serial.println("Obstacle Detected");
        currentState = SCAN;
      }
      break;
  }
}
void scanEnvironment() {

  float maxDistance = -1;
  float chosenAngle = 0;
  // Rotate servo to scan surroundings
  for (int angle = 0; angle <= 180; angle += 15) {
    servo_angle = angle;
    servo_angle = constrain(servo_angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    servo_write((uint16_t)servo_angle);
    _delay_ms(1000);  // Allow sensor to stabilize

    // Evaluate distance using ultrasonic sensor
    long distance = measureUltrasonicDistance();

    //output for testing
    Serial.println("Distance at ");
    Serial.println(angle);
    Serial.println("* degrees is ");
    Serial.println(distance);
    Serial.println(" cm");

    // dont want to lock onto the wall nearby
    if (distance > 25 && distance > maxDistance) {
      maxDistance = distance;
      chosenAngle = (float)angle;
    }
  }
  //write the angle with the greatest distance
  servo_angle = chosenAngle;
  servo_angle = constrain(servo_angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  servo_write((uint16_t)servo_angle);

  //output for testing
  Serial.println("Chosen Angle is ");
  Serial.println(servo_angle);
  Serial.println(" degrees");
}
bool isObstacleDetected() {
  // Check if the ultrasonic sensor detects an obstacle within threshold
  long distance = measureUltrasonicDistance();
  return distance < DISTANCE_THRESHOLD;
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
void setupPWM() {
  //ORIGINALLY PWM SETUP IS FOR THE LED BRIGHTNESS
  //WE SHOULD SET IT UP FOR THE SERVO INSTEAD?
  //OR JUST USE THE SERVO WRITE FUNCTION THE OTHER TEAM MADE?
  
   // Configure Timer1 for PWM (20ms period, 50Hz frequency)
    TCCR1A = (1 << WGM11) | (1 << COM1A1); // Fast PWM, non-inverting
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    ICR1 = 20000; // 20ms period
    OCR1A = 1500; // Center position (1.5ms pulse)
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
