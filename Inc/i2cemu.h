#ifndef __I2C_Driver_H
#define __I2C_Driver_H

/* Includes ------------------------------------------------------------------*/

#include "stm32l0xx_hal.h"

#define SCL_H         GPIOB->BSRR = GPIO_PIN_6
#define SCL_L         GPIOB->BRR  = GPIO_PIN_6 
   
#define SDA_H         GPIOB->BSRR = GPIO_PIN_7
#define SDA_L         GPIOB->BRR  = GPIO_PIN_7

#define SCL_read      GPIOB->IDR  & GPIO_PIN_7
#define SDA_read      GPIOB->IDR  & GPIO_PIN_7

#define I2C_PageSize  8  //24C02¨C­¶8¦r¸`

void I2C_GPIO_Config(void);

uint8_t I2C_ReadByte(uint8_t* pBuffer,   uint8_t length,  uint8_t DeviceAddress);
uint8_t I2C_BufferWrite(uint8_t* pBuffer, uint8_t length, uint8_t DeviceAddress);
uint8_t I2C_WriteByte(uint8_t SendByte,  uint8_t DeviceAddress);
uint8_t I2C_ReadByteAddress(uint8_t* pBuffer,   uint8_t length,  uint8_t DeviceAddress);
/*
void I2C_SoftWare_Master_Init(void);
void I2C_delay(void);
uint8_t I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
uint8_t I2C_WaitAck(void);
void I2C_SendByte(uint8_t byte);
uint8_t I2C_ReceiveByte(void);
void I2C_SoftWare_Master_Init(void);
int I2C_SoftWare_Master_Write(uint8_t DeviceAddr, uint8_t* pBuffer, uint16_t NumByteToWrite);
int I2C_SoftWare_Master_Read(uint8_t DeviceAddr, uint8_t* pBuffer, uint16_t NumByteToRead);
int I2C_SoftWare_Master_ReInit(void);
*/
#endif 



