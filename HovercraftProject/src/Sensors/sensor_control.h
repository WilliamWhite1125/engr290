#ifndef IMU_CONTROL_H
#define IMU_CONTROL_H

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

#define LED_L_PIN PB5             // LED "L" pin (Arduino pin 13)
#define LED_D3_PIN PB3            // LED D3 pin (OC2A, Arduino pin 11)
#define SERVO_PIN PD5             // Servo motor control pin (Arduino pin 5)

#define GYRO_SENSITIVITY 131.0    // Gyroscope sensitivity scale factor
#define ACCEL_SCALE 16384.0       // Accelerometer sensitivity scale factor

// Function-like macros
#define constrain(x, a, b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define fabs(x) ((x)>=0?(x):-(x))

// Global Variables
extern volatile unsigned long milliseconds;
extern unsigned long lastOutputTime;
extern unsigned long previous_time;

extern int16_t gx_raw, gy_raw, gz_raw;
extern int16_t ax_raw, ay_raw, az_raw;
extern float gyro_z;
extern float yaw;
extern float roll, pitch;
extern float delta_t;

extern float Ax_cal, Ay_cal, Az_cal;
extern float distance_traveled;
extern float previous_velocity;

extern float gz_bias, ax_bias, ay_bias, az_bias;

extern float accelX_filtered;
extern float accelX_high_pass;
extern float accelX_prev;
extern float alpha_hp;

extern float alpha_lp;
extern float servo_angle;

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

#endif // IMU_CONTROL_H
