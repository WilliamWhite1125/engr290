extern "C" {
    #include "init_290.h"
    #include "TWI_290.h"  // Include TWI library for I2C communication
}
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// Pin Definitions
#define LED_PIN PB5            // Onboard LED
#define HOVER_FAN_PIN PD4        // Hover fan (Fan 1) - On/Off
#define PROPULSION_FAN_PIN PD6   // Propulsion fan (Fan 2) - PWM (0C0A)
#define TRIG_PIN PB3
#define ECHO_PIN PD2
#define SERVO_PIN PD5             // Servo motor control pin (Arduino pin 5) // MAYBE CHANGE TO PB1???
#define SDA_PIN PC4               // SDA connected to PC4 (Arduino A4)
#define SCL_PIN PC5               // SCL connected to PC5 (Arduino A5)
#define IR_PIN PC0  

// Constants
#define F_CPU 16000000UL // CPU frequency
#define BAUD 9600 // Desired baud rate
#define MYUBRR F_CPU/16/BAUD-1 // UBRR calculation

#define IMU_ADDRESS 0x68          // MPU-6050 C address
#define C_DELAY_US 5            // I2C communication delay in microseconds
#define MIN_YAW -85               // Minimum yaw angle
#define MAX_YAW 85               // Maximum yaw angle

#define GYRO_SENSITIVITY 131.0    // Gyroscope sensitivity scale factor
#define ACCEL_SCALE 16384.0       // Accelerometer sensitivity scale factor
#define I2C_DELAY_US 5  // I2C communication delay in microseconds

#define GOAL_LOW 20  // Adjust based on IR sensor data
#define GOAL_HIGH 50  // Adjust based on IR sensor data
#define DISTANCE_THRESHOLD 30  //cm //adjust based on testing results

// Function-like macros
#define constrain(x, a, b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define fabs(x) ((x)>=0?(x):-(x))

/ IMU data
int16_t gx_raw, gy_raw, gz_raw;
int16_t ax_raw, ay_raw, az_raw;
float gyro_z;                   // Gyroscope Z-axis data in degrees/second
float yaw = 0;                  // Yaw angle in degrees
float roll = 0.0, pitch = 0.0;  // Calculated roll and pitch angles
float delta_t;

// Calibrated accelerometer values
float Ax_cal, Ay_cal, Az_cal;

// Distance measurement variables
float distance_traveled = 0;    // Total distance traveled in X-axis (meters)
float previous_velocity = 0;    // Previous velocity along X-axis (m/s)

// Gyroscope and accelerometer biases
float gz_bias = 0.0;            // Gyroscope Z-axis bias
float ax_bias = 0.0;
float ay_bias = 0.0;
float az_bias = 0.0;

// For filtering
float accelX_filtered = 0.0;
float accelX_high_pass = 0.0;
float accelX_prev = 0.0;
float alpha_hp = 0.98; // High-pass filter coefficient

float alpha_lp = 0.2; // Low-pass filter coefficient for acceleration

// Define states
enum State { INIT, IDLE, SCAN, MOVE, GOAL, STOP };
State currentState = INIT;
enum Status{SUCCESS, FAILURE}; //could use this to check if the setup is complete
Status initStatus=FAILURE;

//global variables
volatile unsigned long timer_millis;
unsigned long lastOutputTime = 0;
unsigned long previous_time = 0;
// Initialize to middle position

// Function Prototypes
void MPU6050_init();
void readMPU6050();
void calculateAngles();
void measureDistance();
void controlAccelerationLED();
void controlYawLED();
void setupPWM(); //old servo, but good imu (to fix)
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

//OLD SERVO write FN
void setupPWM(); //OLD SERVO FN

void I2C_init();
void I2C_start();
void I2C_stop();
uint8_t I2C_write_byte(uint8_t data);
uint8_t I2C_read_byte(uint8_t ack);
void SDA_HIGH();
void SDA_LOW();
void SCL_HIGH();
void SCL_LOW();
uint8_t SDA_READ();
void UART_init(unsigned int baudrate);
void UART_transmit(unsigned char data);
void UART_print(const char* str);
void UART_println(const char* str);
void UART_printFloat(float number, int decimalPlaces);
void delay_ms(unsigned int ms); //OLD SERVO FN? DOES IMU USE?
void delay_us(unsigned int us); //OLD SERVO FN? DOES IMU USE?
unsigned long customMillis(); //OLD SERVO FN? DOES IMU USE?

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
void setup(){
  cli(); // Disable global interrupts
     setup_timer();
     UART_puts("System Initializing...\r\n");
  _delay_ms(200);
 // Setup Timer1 for millisecond timing
    TCCR1A = (1 << WGM11); // CTC mode
    OCR1A = 249;           // 1 ms interrupt at 16 MHz with prescaler 64
    TIMSK1 = (1 << OCIE1A); // Enable Timer0 compare match interrupt
    TCCR1B = (1 << CS11) | (1 << CS10); // Prescaler 64
  // Initialize pins
  DDRD |= (1 << HOVER_FAN_PIN) | (1 << PROPULSION_FAN_PIN); // Set PD4 and PD6 as outputs
  DDRB |= (1 << TRIG_PIN);  // TRIG_PIN output
  DDRD &= ~(1 << ECHO_PIN); // ECHO_PIN input
  DDRC &= ~(1 << IR_PIN); //ir pin input
  DDRD |= (1 << PD5); // Set PD5 (Arduino pin 5) as output for servo control

   // Setup Timer0 for millisecond timing

    TCCR0A = (1 << WGM01); // CTC mode
    OCR0A = 249;           // 1 ms interrupt at 16 MHz with prescaler 64
    TIMSK0 = (1 << OCIE0A); // Enable Timer0 compare match interrupt
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64

  // Setup UART for serial communication
    UART_init(9600);

    // Setup I2C
    I2C_init();
  //start servo in the middle

   pwm_init(); // Initialize PWM
 _delay_ms(200);
  sei(); // Enable global interrupts
  previous_time = customMillis();
  UART_println("Calibration complete.");
  servo_write((uint16_t)servo_angle); //OLD SERVO
  _delay_ms(4000);  // Allow sensor to stabilize

  UART_println("Calibration complete.");

    // Initialize MPU6050
    MPU6050_init();

    // Calibrate gyroscope and accelerometer biases
    int num_samples = 2000; // Increased number of samples for better accuracy
    long gz_sum = 0;
    long ax_sum = 0, ay_sum = 0, az_sum = 0;

    
    for (int i = 0; i < num_samples; i++) {
        readMPU6050();
        gz_sum += gz_raw;
        ax_sum += ax_raw;
        ay_sum += ay_raw;
        az_sum += az_raw;
        delay_ms(1);
    }
    gz_bias = (float)gz_sum / num_samples;
    ax_bias = ((float)ax_sum / num_samples) / ACCEL_SCALE;
    ay_bias = ((float)ay_sum / num_samples) / ACCEL_SCALE;
    az_bias = ((float)az_sum / num_samples) / ACCEL_SCALE - 1.0; // Assuming gravity on Z-axis

    sei(); // Enable global interrupts

    previous_time = customMillis();

}

int main() {

    while (1) {
        handleState();
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
//DONE
// Hover Fan (PD4) Control
void startHoverFan() {
    PORTD |= (1 << HOVER_FAN_PIN); // Set PD4 high to turn on hover fan
}

void stopHoverFan() {
    PORTD &= ~(1 << HOVER_FAN_PIN); // Set PD4 low to turn off hover fan
}
//DONE
// Propulsion Fan (PD6) Control Using Timer 1
void startPropulsionFan(uint8_t dutyCycle) {
    static uint8_t timerInitialized = 0;
    if (!timerInitialized) {
        // Configure Timer 0 for Fast PWM, 8-bit mode
        TCCR0A |= (1 << WGM00);       // Fast PWM, 8-bit
        TCCR0A |= (1 << WGM01);       // Fast PWM, part 2
        TCCR0A |= (1 << COM0A1); // Non-inverting mode on 0C0A (PD6)
        TCCR0A |= (1 << COM0B1); 
  
        TCCR0B |= (1 << WGM02);  
        TCCR0B |= (1 << CS01) | (1 << CS00); // Prescaler 64 (PWM ~490 Hz)

        timerInitialized = 1; // Mark Timer 0 as initialized
    }
    OCR0A = dutyCycle; // Set duty cycle for propulsion fan (0-255)
}
//DONE
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
//servo write
//DONE
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
//DONE
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


void I2C_init() {
    PORTC |= (1 << PC4) | (1 << PC5); // Enable pull-up resistors on SDA and SCL
}

void I2C_start() {
    SDA_HIGH();
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    SDA_LOW();
    delay_us(I2C_DELAY_US);
    SCL_LOW();
}

void I2C_stop() {
    SDA_LOW();
    delay_us(I2C_DELAY_US);
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    SDA_HIGH();
    delay_us(I2C_DELAY_US);
}

uint8_t I2C_write_byte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        data <<= 1;
        SCL_HIGH();
        delay_us(I2C_DELAY_US);
        SCL_LOW();
        delay_us(I2C_DELAY_US);
    }
    // ACK/NACK
    SDA_HIGH(); // Release SDA
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    uint8_t ack = !(SDA_READ());
    SCL_LOW();
    return ack;
}

uint8_t I2C_read_byte(uint8_t ack) {
    uint8_t data = 0;
    SDA_HIGH(); // Release SDA
    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        SCL_HIGH();
        delay_us(I2C_DELAY_US);
        if (SDA_READ()) {
            data |= 1;
        }
        SCL_LOW();
        delay_us(I2C_DELAY_US);
    }
    // Send ACK/NACK
    if (ack) {
        SDA_LOW(); // ACK
    } else {
        SDA_HIGH(); // NACK
    }
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    SCL_LOW();
    return data;
}

void SDA_HIGH() {
    PORTC |= (1 << PC4);
    DDRC &= ~(1 << PC4); // Input mode
}

void SDA_LOW() {
    DDRC |= (1 << PC4);  // Output mode
    PORTC &= ~(1 << PC4); // Low
}

void SCL_HIGH() {
    PORTC |= (1 << PC5);
    DDRC &= ~(1 << PC5); // Input mode
}

void SCL_LOW() {
    DDRC |= (1 << PC5);  // Output mode
    PORTC &= ~(1 << PC5); // Low
}

uint8_t SDA_READ() {
    return (PINC & (1 << PC4)) != 0;
}

void MPU6050_init() {
    I2C_start();
    I2C_write_byte(IMU_ADDRESS << 1); // Write address
    I2C_write_byte(0x6B);             // Power management register
    I2C_write_byte(0x00);             // Wake up the MPU6050
    I2C_stop();

    // Set accelerometer configuration (full scale ±2g)
    I2C_start();
    I2C_write_byte(IMU_ADDRESS << 1);
    I2C_write_byte(0x1C); // Accelerometer config register
    I2C_write_byte(0x00); // ±2g
    I2C_stop();

    // Set gyroscope configuration (full scale ±250°/s)
    I2C_start();
    I2C_write_byte(IMU_ADDRESS << 1);
    I2C_write_byte(0x1B); // Gyroscope config register
    I2C_write_byte(0x00); // ±250°/s
    I2C_stop();
}

void readMPU6050() {
    I2C_start();
    I2C_write_byte(IMU_ADDRESS << 1); // Write address
    I2C_write_byte(0x3B);             // Starting register for accelerometer data
    I2C_stop();

    I2C_start();
    I2C_write_byte((IMU_ADDRESS << 1) | 1); // Read address

    ax_raw = (I2C_read_byte(1) << 8) | I2C_read_byte(1);
    ay_raw = (I2C_read_byte(1) << 8) | I2C_read_byte(1);
    az_raw = (I2C_read_byte(1) << 8) | I2C_read_byte(1);
    I2C_read_byte(1); I2C_read_byte(1); // Skip temperature
    gx_raw = (I2C_read_byte(1) << 8) | I2C_read_byte(1);
    gy_raw = (I2C_read_byte(1) << 8) | I2C_read_byte(1);
    gz_raw = (I2C_read_byte(0) << 8) | I2C_read_byte(0); // Last byte NACK

    I2C_stop();
}

void calculateAngles() {
    // Calibrate accelerometer readings
    Ax_cal = ((float)ax_raw / ACCEL_SCALE) - ax_bias;
    Ay_cal = ((float)ay_raw / ACCEL_SCALE) - ay_bias;
    Az_cal = ((float)az_raw / ACCEL_SCALE) - az_bias;

    roll = atan2(Ay_cal, Az_cal) * 180.0 / M_PI;
    pitch = atan2(-Ax_cal, sqrt(Ay_cal * Ay_cal + Az_cal * Az_cal)) * 180.0 / M_PI;

    // Time difference
    unsigned long current_time = customMillis();
    delta_t = (current_time - previous_time) / 1000.0; // Convert ms to s
    previous_time = current_time;

    // Gyroscope readings
    float Gz = ((float)gz_raw - gz_bias) / GYRO_SENSITIVITY;

    // Update yaw
    yaw += Gz * delta_t;

    // Constrain yaw angle within ±180 degrees
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;

    // Constrain yaw within ±85 degrees (optional)
    if (yaw > MAX_YAW) yaw = MAX_YAW;
    if (yaw < MIN_YAW) yaw = MIN_YAW;
}

//to change 
void controlYawLED() {
    // Check if the yaw angle is outside of ±85 degrees
    if (yaw >= MAX_YAW || yaw <= MIN_YAW) {
        PORTB |= (1 << LED_L_PIN);  // Turn on LED "L"
    } else {
        PORTB &= ~(1 << LED_L_PIN); // Turn off LED "L"
    }
}

void controlAccelerationLED() {
    // Use the led acceleration value
    float absAccelX = fabs(accelX_filtered); // Use fabs for floating-point absolute value

    uint16_t brightness = 0;

    // Define thresholds
    const float minAccel = 0.08;
    const float maxAccel = 1.08;

    if (absAccelX <= minAccel) {
        brightness = 0; // LED off
    } else if (absAccelX >= maxAccel) {
        brightness = 255; // Maximum brightness
    } else {
        // Map acceleration to brightness linearly
        brightness = (uint16_t)(((absAccelX - minAccel) / (maxAccel - minAccel)) * 255.0);
    }

    // Set PWM duty cycle
    OCR2A = (uint8_t)brightness;
}

void measureDistance() {
    // Use the calibrated accelerometer value
    float accelX_g = Ax_cal;

    // Apply low-pass filter to acceleration data
    accelX_filtered = alpha_lp * accelX_g + (1 - alpha_lp) * accelX_filtered;

    // Apply high-pass filter to remove bias and drift
    accelX_high_pass = alpha_hp * (accelX_high_pass + accelX_filtered - accelX_prev);
    accelX_prev = accelX_filtered;

    // Implement dynamic thresholding based on noise standard deviation
    float noise_threshold = 0.05; // Adjust this value based on observed noise
    if (fabs(accelX_high_pass) < noise_threshold) {
        accelX_high_pass = 0.0;
    }

    float accelX_m_s2 = accelX_high_pass * 9.80665;

    // Integrate acceleration to get velocity and distance using trapezoidal rule
    float current_velocity = previous_velocity + accelX_m_s2 * delta_t;

    // Apply damping to reduce drift
    float damping_factor = 0.99;
    current_velocity *= damping_factor;

    distance_traveled += ((previous_velocity + current_velocity) / 2.0) * delta_t;
    previous_velocity = current_velocity;
}

void setupPWM() {
    // Set LED D3 pin as output
    DDRB |= (1 << LED_D3_PIN);

    // Setup Timer2 for Fast PWM mode
    TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2A1); // Fast PWM, non-inverting
    TCCR2B = (1 << CS21); // Prescaler 8
    OCR2A = 0; // Initial duty cycle
}


// Initialize PWM for servo control and D3
void pwm_init() {
    DDRB |= (1 << PORTB1); // Set PB1 as output
    TCCR1A  |= (1 << COM1A1) | (1 << COM1B1); // non-inv PWM on channels A and B
    TCCR1B |= (1 << WGM13); // Phase and Frequency Correct PWM
    ICR1 = 39999; // Set TOP value for 50Hz PWM (for servo control)
    TCCR1B |= ((1 << CS11) | (1 << CS10)); // Prescaler 8, starts PWM
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
// Initialization of the UART for serial communication
void UART_init() {
    // Set baud rate
    UBRR0H = (MYUBRR >> 8);
    UBRR0L = MYUBRR;
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Set frame format: 8data, 1stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Transmit a single character over UART 
void UART_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait until the transmit buffer is empty 
    UDR0 = c; // Put character into buffer 
}

// Transmit a string over UART 
void UART_puts(const char *s) {
    while (*s) {
        UART_putc(*s++);
    }
}
//Transmit float
void UART_putfloat(float value) {
    int int_part = (int)value; // Get integer part
    int decimal_part = (int)((value - int_part) * 100); // Get two decimal places

    // Handle negative values for correct formatting
    if (decimal_part < 0) decimal_part = -decimal_part;

    // Prepare the string representation
    char buffer[20];
    sprintf(buffer, "%d.%02d", int_part, decimal_part); // Format as "X.XX"
    UART_puts(buffer); // Send the string over UART
}

// Print an integer over UART 
void UART_putint(uint32_t num) {
    char buffer[10];
    itoa(num, buffer, 10);  // Convert integer to string 
    UART_puts(buffer);      // Send string 
}
// UART Ends Here. Delete later.

