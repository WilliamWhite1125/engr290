#define ECHO_PIN PD2
#define LED_PIN PD3
#define IR_PIN PC0  
#define DEBUG_LED PB5   

#define D1 15   
#define D2 42   

#include <util/delay.h>
#include <avr/io.h>

#define UART_BAUD_RATE 9600
#define UBRR_VALUE ((F_CPU / (UART_BAUD_RATE * 16UL)) - 1)

//function definitions
void uartSendByte(uint8_t data);
void uartSendString(const char* str);
void uartSendNumber(long num);
void uartSendFloat(float num);
long measureUltrasonicDistance();
float measureIRDistance();
uint8_t mapDistanceToBrightness(long distance, int minDistance, int maxDistance);
//map distance to propulsion??

void uartSendByte(uint8_t data) {
  while (!(UCSR0A & (1 << UDRE0)));  // Wait for the transmit buffer to be empty
  UDR0 = data;  // Transmit byte
}

void uartSendString(const char* str) {
  while (*str) {
    uartSendByte(*str++);
  }
}

void uartSendNumber(long num) {
  char buffer[11];  // Buffer to hold the string representation of the number
  ltoa(num, buffer, 10);  // Convert the number to a string (base 10)
  uartSendString(buffer);  // Send the string
}

void uartSendFloat(float num) {
  char buffer[10];
  dtostrf(num, 7, 3, buffer);  // Convert float to string with 3 decimal places
  uartSendString(buffer);
}

void setup() {
    // Initialize UART
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
}

long measureUltrasonicDistance() {
    long duration; 
    int distance;  

    PORTB &= ~(1 << TRIG_PIN); 
    _delay_us(2); 
    PORTB |= (1 << TRIG_PIN); 
    _delay_us(10); 
    PORTB &= ~(1 << TRIG_PIN); 

    duration = pulseIn(ECHO_PIN, HIGH); 
    distance = duration * 0.0344 / 2; 

    return distance; 
}

float measureIRDistance() {
    ADMUX = (1 << REFS0) | (IR_PIN & 0x0F); 
    ADCSRA |= (1 << ADSC);  
    while (ADCSRA & (1 << ADSC));  

    int reading = ADC;

    float voltage = reading * (5.0 / 1024.0); 
    float distance = 29.988 * pow(voltage, -1.173); 
    
    if (distance < D1) {
        distance = D1;
    } else if (distance > D2) {
        distance = D2;
    }

    return distance;
}

uint8_t mapDistanceToBrightness(long distance, int minDistance, int maxDistance) {
    if (distance <= minDistance) {
        return 0; 
    } else if (distance >= maxDistance) {
        return 255; 
    } else {
        return (uint8_t)(((distance - minDistance) * 255.0) / (maxDistance - minDistance));
    }
}

void loop() {
    long ultrasonicDistance = measureUltrasonicDistance();
    float irDistance = measureIRDistance();

    // Replace Serial.print with UART send functions
    uartSendString("Ultrasonic Distance: ");
    uartSendNumber(ultrasonicDistance);
    uartSendString(" cm, IR Distance: ");
    uartSendFloat(irDistance);
    uartSendString(" cm\n");

    if (ultrasonicDistance < D1 && irDistance < D1) {
        PORTB |= (1 << DEBUG_LED); 
        _delay_ms(100);
        PORTB &= ~(1 << DEBUG_LED); 
        _delay_ms(100);
    }

    uint8_t usBrightness = mapDistanceToBrightness(ultrasonicDistance, D1, D2);
    OCR2B = usBrightness;  

    _delay_ms(100);
}