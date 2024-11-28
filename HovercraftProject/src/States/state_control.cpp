
// Include necessary libraries and headers
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define ECHO_PIN PD2
#define LED_PIN PD3
#define IR_PIN PC0
#define TRIG_PIN PB0


// Constants and Macros
#define F_CPU 16000000UL          // 16 MHz clock speed
#define IMU_ADDRESS 0x68          // MPU-6050 I2C address
#define I2C_DELAY_US 5            // I2C communication delay in microseconds

#define SDA_PIN PC4               // SDA connected to PC4 (Arduino A4)
#define SCL_PIN PC5               // SCL connected to PC5 (Arduino A5)

#define MIN_YAW -85               // Minimum yaw angle
#define MAX_YAW 85                // Maximum yaw angle
#define MIN_SERVO_ANGLE 0         // Minimum servo angle
#define MAX_SERVO_ANGLE 180       // Maximum servo angle

#define LED_L_PIN PB5             // LED "L" pin (Arduino pin 13)
#define LED_D3_PIN PB3            // LED D3 pin (OC2A, Arduino pin 11)
#define SERVO_PIN PD5             // Servo motor control pin (Arduino pin 5) WITH TIMER 0

#define GYRO_SENSITIVITY 131.0    // Gyroscope sensitivity scale factor
#define ACCEL_SCALE 16384.0       // Accelerometer sensitivity scale factor

#define HOVER_FAN_PIN PD4        // Hover fan (Fan 1) - On/Off
#define PROPULSION_FAN_PIN PD6   // Propulsion fan (Fan 2) - PWM (0C0A)

// Define states
enum State { INIT, IDLE, SCAN, MOVE, GOAL, STOP };
State currentState = INIT;

//could use this to check if the setup is complete
enum Status{SUCCESS, FAILURE};
State initStatus=FAILURE;

// Constants
#define DISTANCE_THRESHOLD 40         //cm //adjust based on testing results
#define GOAL_LOW 20  // Adjust based on IR sensor data
#define GOAL_HIGH 50  // Adjust based on IR sensor data

// Function prototypes
void setupSystem();         //COPIED FROM SETUPS IN DEMOS, MAYBE MISSING SOME
void handleState();         //DONE
void scanEnvironment();     //DONE
bool isGoalDetected();      //DONE
bool isObstacleDetected();  //DONE

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
      stopFans();
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
      startPropulsionFan();
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
      stopFans();

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

void scanEnvironment() {

  float maxDistance = -1;
  float chosenAngle = 0;
  // Rotate servo to scan surroundings
  for (int angle = 0; angle <= 180; angle += 15) {
    servo_write(angle);
    _delay_ms(100);  // Allow sensor to stabilize

    // Evaluate distance using ultrasonic sensor
    float distance = measureUltrasonicDistance();

    //output for testing
    UART_print("Distance at ");
    UART_printFloat(distance, 2);
    UART_println(" cm");

    // need dont want to lock onto the wall nearby
    if (distance > 25 && distance > maxDistance) {
      maxDistance = distance;
      chosenAngle = (float)angle;
    }
  }
  //write the angle with the greatest distance
  servo_write(chosenAngle);

  //output for testing
  UART_print("Angle at ");
  UART_printFloat(chosenAngle, 2);
  UART_println(" degrees");
}

bool isGoalDetected() {
  // Use IR sensor to check for the goal
  float irReading = measureIRDistance();  // Assume measureIR() returns the IR sensor reading
  return (irReading > GOAL_LOW && irReading < GOAL_HIGH); //minumim reading is 20cm
}

bool isObstacleDetected() {
  // Check if the ultrasonic sensor detects an obstacle within threshold
  float distance = measureUltrasonic();
  return distance < DISTANCE_THRESHOLD;
}

void setupSystem() { //SETUP FUNCTION FOR ALL SUBSYSTEMS
  cli(); // Disable global interrupts

  // ********** Setup code from demo1.ino **********
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0B = (1 << TXEN0);  // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data format

    DDRB |= (1 << DEBUG_LED) | (1 << TRIG_PIN);  
    DDRD |= (1 << LED_PIN);  
    DDRD &= ~(1 << ECHO_PIN); 
    DDRC &= ~(1 << IR_PIN); 
    
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  
    TCCR2B |= (1 << CS22); 

    // ********** Setup code from sensor_control.cpp **********

    // Setup Timer0 for millisecond timing
    TCCR0A = (1 << WGM01); // CTC mode
    OCR0A = 249;           // 1 ms interrupt at 16 MHz with prescaler 64
    TIMSK0 = (1 << OCIE0A); // Enable Timer0 compare match interrupt
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64

    // Setup UART for serial communication
    UART_init(9600);

    // Setup I2C
    I2C_init();

    // Setup LED pins
    DDRB |= (1 << LED_L_PIN); // LED "L" pin as output

    // Setup Servo pin
    DDRD |= (1 << PD5); // Set PD5 (Arduino pin 5) as output for servo control

    // Initialize PWM for LED D3 brightness control
    setupPWM();

    // Initialize MPU6050
    MPU6050_init();

    // Calibrate gyroscope and accelerometer biases
    int num_samples = 2000; // Increased number of samples for better accuracy
    long gz_sum = 0;
    long ax_sum = 0, ay_sum = 0, az_sum = 0;

    UART_println("Calibrating sensors. Please keep the device still...");

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

    UART_println("Calibration complete.");

    return SUCCESS; //RIGHT NOW ALWAYS RETURNS SUCCESS
}
