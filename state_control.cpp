
//define states
enum State { INIT, HOVER, SCAN, MOVE, TURN, AVOID, GOAL, STOP };

//Initialization
//Set up sensors and actuators (IMU, fans, servo, IR, ultrasonic sensor).
//Perform system checks and calibration.

//Scanning
//Rotate the servo motor with the ultrasonic sensor to scan for open paths forward.
//Evaluate path options and decide on a direction.

//Turning
//Adjust the servo angle to change direction.
//Use the IMU for angular position feedback to ensure accurate turning.

//Hovering
//Keep the lift fan active to maintain hovering. Check for stability using the IMU. 

//Moving Forward and Obstacle detection
//Propel the hovercraft in the selected direction.
//Continuously check for obstacles with the ultrasonic sensor.
//Detect obstacles in the forward path.
//Transition back to Scanning or execute an evasive maneuver.

//Goal Detection
//Use the IR sensor to detect the bar above the hovercraft at the maze's end.
//Slow down and align under the bar.

//Stopping
//Turn off the propulsion and lift fans.
//Indicate task completion (e.g., LED or buzzer).

