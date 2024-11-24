
// Include necessary libraries and headers
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "sensor_control.ino"
#include "fan_control.ino"

// Define states
enum State { INIT,
             IDLE,
             SCAN,
             MOVE,
             GOAL,
             STOP };
State currentState = INIT;

// Constants
#define DISTANCE_THRESHOLD 40         //cm //adjust based on testing results
#define GOAL_DETECTION_THRESHOLD 20  // Adjust based on IR sensor data

// Function prototypes
void setupSystem();         //TODO
void handleState();         //almostdone
void scanEnvironment();     //DONE
bool isGoalDetected();      //TODO
bool isObstacleDetected();  //DONE

void handleState() {

  switch (currentState) {

    case INIT:  //DONE
      //Set up sensors and actuators (IMU, fans, servo, IR, ultrasonic sensor).
      setupSystem();
      //if setup complete
      currentState = IDLE;
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
        currentState = GOAL;
      }
      break;

    case GOAL:  //TODO
                //Use the IR sensor to detect the bar above the hovercraft at the maze's end.
                // Detect the goal and stop under the bar
      stopFan();
      if (isGoalDetected()) {
        currentState = STOP;
      }
      break;

    case STOP:  //DONE
      // Halt all systems and indicate completion
      stopFans();

      //blink lights and end program
      while(1){
        PORTB |= (1 << DEBUG_LED);
        _delay_ms(100);
        PORTB &= ~(1 << DEBUG_LED);
        _delay_ms(100);
      }
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
  return irReading > GOAL_DETECTION_THRESHOLD; //minumim reading is 20cm
}

bool isObstacleDetected() {
  // Check if the ultrasonic sensor detects an obstacle within threshold
  float distance = measureUltrasonic();
  return distance < DISTANCE_THRESHOLD;
}