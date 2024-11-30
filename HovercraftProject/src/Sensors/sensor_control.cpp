#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

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

#define LED_L_PIN PB5           // LED "L" pin (Arduino pin 13)
#define LED_D3_PIN PB3            // LED D3 pin (OC2A, Arduino pin 11)
#define SERVO_PIN PB1            // Servo motor control pin

#define GYRO_SENSITIVITY 131.0    // Gyroscope sensitivity scale factor
#define ACCEL_SCALE 16384.0       // Accelerometer sensitivity scale factor

// Function-like macros
#define constrain(x, a, b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define fabs(x) ((x)>=0?(x):-(x))

// Global Variables
volatile unsigned long milliseconds = 0; // Millisecond counter for timing
unsigned long lastOutputTime = 0;
unsigned long previous_time = 0;

// IMU data
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

// Global variable for servo angle
float servo_angle = 90.0; // Initialize to middle position

// Function Prototypes
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

void MPU6050_init();
void readMPU6050();
void calculateAngles();
void measureDistance();
void controlAccelerationLED();
void controlYawLED();
void servo_write(uint16_t angle);
void setupPWM();
void UART_init(unsigned int baudrate);
void UART_transmit(unsigned char data);
void UART_print(const char* str);
void UART_println(const char* str);
void UART_printFloat(float number, int decimalPlaces);
void delay_ms(unsigned int ms);
void delay_us(unsigned int us);
unsigned long customMillis();

ISR(TIMER0_COMPA_vect) {
    milliseconds++;
}

void setup() {
    cli(); // Disable global interrupts

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

    // Initialize PWM for SERVO CONTROL
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
}

void loop() {
    unsigned long currentMillis = customMillis();

    // Read IMU data
    readMPU6050();

    // Calculate roll, pitch, yaw
    calculateAngles();

    // Map yaw angle from (-85 to +85) to servo angle (0 to 180)
    servo_angle = ((yaw + 85.0) * 180.0) / 170.0;
    servo_angle = constrain(servo_angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    servo_write((uint16_t)servo_angle);

    // Control LED "L" based on yaw range
    controlYawLED();

    // Control LED D3 brightness based on X-axis acceleration
    controlAccelerationLED();

    // Measure distance along X-axis
    measureDistance();

    // Output data once per second
    if (currentMillis - lastOutputTime >= 1000) {
        lastOutputTime = currentMillis;

        // Output roll, pitch, yaw
        UART_print("Roll: ");
        UART_printFloat(roll, 2);
        UART_print(", Pitch: ");
        UART_printFloat(pitch, 2);
        UART_print(", Yaw: ");
        UART_printFloat(yaw, 2);
        UART_println("");

        // Output accelerations
        UART_print("AccelX: ");
        UART_printFloat(Ax_cal, 3);
        UART_print("g, AccelY: ");
        UART_printFloat(Ay_cal, 3);
        UART_print("g, AccelZ: ");
        UART_printFloat(Az_cal, 3);
        UART_println("g");

        // Output filtered acceleration and high-pass acceleration
        UART_print("Filtered AccelX: ");
        UART_printFloat(accelX_filtered, 3);
        UART_print("g, High-Pass AccelX: ");
        UART_printFloat(accelX_high_pass, 3);
        UART_println("g");

        // Output distance traveled
        UART_print("Distance X: ");
        UART_printFloat(distance_traveled, 4);
        UART_println(" meters");

        // Output servo angle for debugging
        UART_print("Servo Angle: ");
        UART_printFloat(servo_angle, 2);
        UART_println(" degrees");
    }
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

void servo_write(uint16_t angle) {
    // Constrain angle to 0-180
    angle = constrain(angle, 0, 180);
    
    // Convert angle to pulse width (900-2100 microseconds)
    // 900µs = 1800 timer ticks, 2100µs = 4200 timer ticks
    uint16_t pulse_width = 1000 + ((4000 * (uint32_t)angle) / 180);
    pulse_width = constrain(pulse_width, 1000, 5000);
    
    
    // Update PWM compare value
    OCR1A = pulse_width;
}

void controlYawLED() {
    // Check if the yaw angle is outside of ±85 degrees
    if (yaw >= MAX_YAW || yaw <= MIN_YAW) {
        PORTB |= (1 << LED_L_PIN);  // Turn on LED "L"
    } else {
        PORTB &= ~(1 << LED_L_PIN); // Turn off LED "L"
    }
}

void controlAccelerationLED() {
    // Use the filtered acceleration value
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
    // Configure Timer1 for PWM generation
    // Fast PWM mode with ICR1 as TOP
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    
    // Set PWM frequency to 50Hz (20ms period)
    ICR1 = 39999;  // = 16MHz/(8 * 50Hz) - 1
    
    // Initialize servo to middle position
    OCR1A = 3000;  // ~1.5ms pulse (90 degrees)
}

void UART_init(unsigned int baudrate) {
    unsigned int ubrr = F_CPU / 16 / baudrate - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable receiver and transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, no parity, 1 stop bit
}

void UART_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait for buffer to be empty
    UDR0 = data;
}

void UART_print(const char* str) {
    while (*str) {
        UART_transmit(*str++);
    }
}

void UART_println(const char* str) {
    UART_print(str);
    UART_transmit('\r');
    UART_transmit('\n');
}

void UART_printFloat(float number, int decimalPlaces) {
    // Handle negative numbers
    if (number < 0.0) {
        UART_transmit('-');
        number = -number;
    }

    // Integer part
    unsigned long intPart = (unsigned long)number;
    // Decimal part
    float decimalPart = number - (float)intPart;

    // Convert integer part
    char buffer[12];
    ultoa(intPart, buffer, 10);
    UART_print(buffer);

    // Decimal point and decimal part
    if (decimalPlaces > 0) {
        UART_transmit('.');
        for (int i = 0; i < decimalPlaces; i++) {
            decimalPart *= 10.0;
            int digit = (int)decimalPart;
            decimalPart -= digit;
            UART_transmit('0' + digit);
        }
    }
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
