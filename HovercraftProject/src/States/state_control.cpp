
// Include necessary libraries and headers
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "sensor_control.ino"
#include "fan_control.ino"

// Define states
enum State { INIT, IDLE, SCAN, MOVE, GOAL, STOP };
State currentState = INIT;

// Constants
#define DISTANCE_THRESHOLD 70  // cm
#define GOAL_DETECTION_THRESHOLD 0.5  // Adjust based on IR sensor data

// Function prototypes
void setupSystem();
void handleState();
void scanEnvironment();
bool isGoalDetected();
bool isObstacleDetected();

void handleState(){

    switch(currentState){
        
        case INIT:
        //Set up sensors and actuators (IMU, fans, servo, IR, ultrasonic sensor).
        //Perform system checks and calibration.
        setupSystem();
        //if setup complete
        currentState=IDLE;
        break;

        case IDLE:
        //if fans are on
        stopFans();
        _delay_ms(1000); // Wait for stability
        //switches to scan when it stops moving
        currentState=SCAN;
        break;

        case SCAN:
        // Scan the environment to find the best path forward
            scanEnvironment();
            // Transition to MOVE if a viable path is found
            currentState = MOVE;
        break;

        case MOVE:
        // Start fans and move forward
            startHoverFan();
            startPropulsionFan();
            // Monitor for obstacles and transition accordingly
            if (isObstacleDetected()) {
                currentState = SCAN;
            } else if (isGoalDetected()) {
                currentState = GOAL;
            }
        break;

        case GOAL:
        //Use the IR sensor to detect the bar above the hovercraft at the maze's end.
        // Detect the goal and stop under the bar
            stopFan();
            if (isGoalDetected()) {
                currentState = STOP;
            }
        break;

        case STOP:
        // Halt all systems and indicate completion
            stopFan();
            // Optional: Flash LED or buzzer
            PORTB |= (1 << LED_L_PIN);
            while (1);  // End program
        break;
    }
}

void setupSystem() {
    // Call initialization functions from the files
    setupPWM();       // Fan control PWM
    MPU6050_init();   // IMU initialization
    UART_init(9600);  // Serial communication
}

void scanEnvironment() {
    // Rotate servo to scan surroundings
    for (int angle = 0; angle <= 180; angle += 15) {
        servo_write(angle);
        _delay_ms(100);  // Allow sensor to stabilize

        // Evaluate distance using ultrasonic sensor
        float distance = measureUltrasonic();
        UART_print("Distance at ");
        UART_printFloat(distance, 2);
        UART_println(" cm");
        // Add logic to decide the best path based on distances
    }
}

bool isGoalDetected() {
    // Use IR sensor to check for the goal (e.g., bar above)
    float irReading = measureIR();  // Assume measureIR() returns the IR sensor reading
    return irReading < GOAL_DETECTION_THRESHOLD;
}

bool isObstacleDetected() {
    // Check if the ultrasonic sensor detects an obstacle within threshold
    float distance = measureUltrasonic();
    return distance < DISTANCE_THRESHOLD;
}