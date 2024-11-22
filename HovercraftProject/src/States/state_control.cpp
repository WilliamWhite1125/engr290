
//define states
enum State { INIT, IDLE, HOVER, SCAN, MOVE, TURN, GOAL, STOP };

State currentState= INIT;

void handleState(){

    switch(currentState){
        
        case INIT:
        //Set up sensors and actuators (IMU, fans, servo, IR, ultrasonic sensor).
        //Perform system checks and calibration.
        setup();
        MPU6050_init();
        I2C_init();
        I2C_start();
        //if setup complete
        currentState=IDLE;
        break;

        case IDLE:
        //if fans are on
        stopFans();
        //wait 1s to stop
        //switches to scan when it stops moving
        currentState=SCAN;
        break;

        case SCAN:
        //Rotate the servo motor with the ultrasonic sensor to scan for open paths forward.
        //Evaluate path options and decide on a direction.

        //rotate servo towards farthest distance
        /*if(angle is aligned witn imu ***FORWARD***  */
        currentState= MOVE;
        //else
        currentState= TURN;
        break;

        case TURN:
        //Use the IMU for angular position feedback to ensure accurate turning.
        //feather propulsion fan until IMU aligned with the fan, then
        currentState= MOVE;
        break;

        case HOVER:
        //Keep the lift fan active to maintain hovering. Check for stability using the IMU. 
        //is this state necessary or no?
        break;

        case MOVE:
        //Propel the hovercraft in the selected direction.
        //Continuously check for obstacles with the ultrasonic sensor.
        //Detect obstacles in the forward path.
        //Transition back to Scanning or execute an evasive maneuver.
        break;

        case GOAL:
        //Use the IR sensor to detect the bar above the hovercraft at the maze's end.
        //Slow down and align under the bar.

        //if acceleration = 0 and ir sensor sees bar
        currentState=STOP;
        break;

        case STOP:
        //Turn off the propulsion and lift fans.
        //Indicate task completion (e.g., LED or buzzer).
        break;
    }
}