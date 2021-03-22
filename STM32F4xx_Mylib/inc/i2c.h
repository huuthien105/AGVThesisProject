#ifndef I2C_H
#define I2C_H

#include "stm32f4xx.h"


uint8_t I2C_Write(uint8_t Address, uint8_t *pData, uint8_t length);
uint8_t I2C_Read(uint8_t Address, uint8_t *pData, uint8_t length);

extern void i2c_init(void);
extern void i2c_start(void);
extern void i2c_stop(void);
extern uint8_t i2c_write(uint8_t u8Data);
extern uint8_t i2c_read(uint8_t u8Ack);
extern void delay_ms(uint32_t u32DelayInMs);
extern void delay_us(uint32_t delay);
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t u8Data);
uint8_t i2c_read(uint8_t u8Ack);

#endif
