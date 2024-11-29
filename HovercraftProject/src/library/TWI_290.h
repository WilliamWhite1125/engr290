#ifndef TWI_290_H
#define TWI_290_H

#include <avr/io.h> // For AVR register definitions
#include <stdint.h> // Include this for uint8_t and other standard types

// Function prototypes
void twi_init(void);

uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write); // Return type added

void TWI_stop(void);

uint8_t TWI_write(uint8_t tx_data); // Return type added

uint8_t TWI_ack_read(void);

uint8_t TWI_nack_read(void);

uint8_t TWI_byte_return(void);

uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value); // Return type added

uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr); // Adjusted parameters

uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t num_bytes, uint8_t* data); // Return type and parameters adjusted

#endif // TWI_290_H