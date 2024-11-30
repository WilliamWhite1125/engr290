#include <util/delay.h>
#include <avr/io.h>

#define F_CPU 16000000UL         // Clock speed

// Updated Pin Definitions for WOWKI design
#define TRIG_PIN PD3             // Trigger pin assigned to Arduino pin 3
#define ECHO_PIN PD2             // Echo pin assigned to Arduino pin 2
#define SERVO_PIN PB1            // Servo motor control pin (Arduino pin 9)

// Servo and Distance Parameters
#define MIN_SERVO_ANGLE 0         // Minimum servo angle
#define MAX_SERVO_ANGLE 180       // Maximum servo angle
#define DISTANCE_THRESHOLD 30     //(cm) Adjust based on testing results

#define constrain(x, a, b) ((x)<(a)?(a):((x)>(b)?(b):(x)))

// Global variables
volatile unsigned long milliseconds = 0; // Millisecond counter for timing
float servo_angle = 90.0; // Initialize to middle position
enum State { INIT, IDLE, SCAN, MOVE };
State currentState = INIT;
float currentSpeed = 0;

// Function Prototypes
void setup();
void handleState();
void scanEnvironment();
bool isObstacleDetected();
long measureUltrasonicDistance();
void servo_write(uint16_t angle);
void setupPWM();

// Timer interrupt for custom millis
ISR(TIMER1_COMPA_vect) {
    milliseconds++;
}

void setup() {
    // Initialize Timer1 for millisecond timing
    TCCR1A = (1 << WGM11);        // CTC mode
    OCR1A = 249;                  // 1 ms interrupt at 16 MHz with prescaler 64
    TIMSK1 = (1 << OCIE1A);       // Enable Timer1 compare match interrupt
    TCCR1B = (1 << CS11) | (1 << CS10); // Prescaler 64

    Serial.begin(9600);
    Serial.println("CURRENT STATE INIT");

    // Set TRIG_PIN as output, ECHO_PIN as input
    DDRD |= (1 << TRIG_PIN);  // TRIG_PIN (PD3) output
    DDRD &= ~(1 << ECHO_PIN); // ECHO_PIN (PD2) input

    // Setup Servo pin (PB1 - Arduino pin 9)
    DDRB |= (1 << SERVO_PIN); // Set PB1 (Arduino pin 9) as output for servo control
    servo_write((uint16_t)servo_angle);
    _delay_ms(1000);  // Allow sensor to stabilize

    setupPWM();
    
    currentSpeed = 255;
    currentState = IDLE;
}

// Main Function
int main() {
    setup();
    while (1) {
        servo_write(0);
        _delay_ms(3000);
        servo_write(90);
        _delay_ms(3000);
        servo_write(180);
        _delay_ms(3000);

    }
    return 0; // Not reached
}

// Measure distance with ultrasonic sensor
long measureUltrasonicDistance() {
    long duration;
    int distance;

    PORTD &= ~(1 << TRIG_PIN); // Set TRIG_PIN LOW
    _delay_us(2);
    PORTD |= (1 << TRIG_PIN);  // Set TRIG_PIN HIGH
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN); // Set TRIG_PIN LOW

    // Measure the duration of the echo
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.0344 / 2;

    return distance;
}

void handleState() {
    switch (currentState) {
        case IDLE:
            Serial.println("CURRENT STATE IDLE");
            Serial.println("StopFans()");
            _delay_ms(1000); // Wait for stability
            currentState = SCAN;
            break;

        case SCAN:
            Serial.println("CURRENT STATE SCAN");
            scanEnvironment();
            currentState = MOVE;
            break;

        case MOVE:
            Serial.println("CURRENT STATE MOVE");
            Serial.println("startHoverFan()");
            Serial.println("startPropulsionFan()");
            _delay_ms(4000); // Simulate movement
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

    for (int angle = 0; angle <= 180; angle += 15) {
        servo_angle = constrain(angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
        servo_write((uint16_t)servo_angle);
        _delay_ms(100); // Allow sensor to stabilize

        long distance = measureUltrasonicDistance();

        Serial.print("Distance at ");
        Serial.print(angle);
        Serial.print("* degrees: ");
        Serial.print(distance);
        Serial.println(" cm");

        if (distance > 25 && distance > maxDistance) {
            maxDistance = distance;
            chosenAngle = (float)angle;
        }
    }
    chosenAngle = 90;
    servo_angle = chosenAngle;
    servo_write((uint16_t)servo_angle);

    Serial.print("Chosen Angle: ");
    Serial.println(servo_angle);
}

bool isObstacleDetected() {
    long distance = measureUltrasonicDistance();
    return distance < DISTANCE_THRESHOLD;
}

void servo_write(uint16_t angle) {
    // Constrain angle to 0-180
    angle = constrain(angle, 0, 180);
    
    // Convert angle to pulse width (900-2100 microseconds)
    // 900µs = 1800 timer ticks, 2100µs = 4200 timer ticks
    uint16_t pulse_width = 1000 + ((4000 * (uint32_t)angle) / 180);
    pulse_width = constrain(pulse_width, 1000, 5000);
    
    
    // Update PWM compare value
    OCR1A = pulse_width;
    
    // Debug output
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" Pulse width: ");
    Serial.println(pulse_width);
}


void setupPWM() {
    // Configure Timer1 for PWM generation
    // Fast PWM mode with ICR1 as TOP
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    
    // Set PWM frequency to 50Hz (20ms period)
    ICR1 = 39999;  // = 16MHz/(8 * 50Hz) - 1
    
    // Initialize servo to middle position
    OCR1A = 3000;  // ~1.5ms pulse (90 degrees)
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

// Linear map function for unsigned long
unsigned long linear_map(unsigned long value, unsigned long in_min, unsigned long in_max, unsigned long out_min, unsigned long out_max) {
    if (in_max == in_min) {
        // Avoid division by zero
        fprintf(stderr, "Error: Input range cannot be zero.\n");
        return 0;
    }
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
