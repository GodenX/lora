#include "i2cemu.h"
/********************************************************/

/******************************************************************************* 
* ��������: Simulate_I2C_Init                                                                      
* ��    ��: I2C��ʼ������                                                                      
*                                                                                
* ��    ��: ��                                                                      
* ��    ��: ��                                                                      
* ��    ��: ��                                                                      
* ��    ��:                                                                      
* �޸�����: 2013��1��24��                                                                     
*******************************************************************************/ 

void I2C_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    /* Configure I2C1 pins: SCL and SDA */
    GPIO_InitStructure.Pin =  GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void I2C_delay(void)
{
    uint16_t i=50; //�@�e���ԃ����ٶ�	�����yԇ��͵�5߀�܌���
    while(i)
    {
        i--;
        asm("nop");
    }
}

uint8_t I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if(!SDA_read)return 0;	//SDA������ƽ�t����æ,�˳�
    SDA_L;
    I2C_delay();
    if(SDA_read) return 0;	//SDA������ƽ�t�������e,�˳�
    SDA_L;
    I2C_delay();
    return 1;
}

void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

uint8_t I2C_WaitAck(void) 	 //���؞�:=1��ACK,=0�oACK
{
    SCL_L;
    I2C_delay();
    SDA_H;
	I2C_delay();
    SCL_H;
	I2C_delay();
	I2C_delay();
    if(SDA_read)
    {
        SCL_L;
        return 0;
    }
    SCL_L;
    return 1;
}

void I2C_SendByte(uint8_t SendByte) //�����ĸ�λ����λ//
{
    uint8_t i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
        if(SendByte&0x80)
            SDA_H;
        else
            SDA_L;
        SendByte<<=1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

uint8_t I2C_ReceiveByte(void)  //�����ĸ�λ����λ//
{
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;
    while(i--)
    {
        ReceiveByte<<=1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if(SDA_read)
        {
            ReceiveByte|=0x01;
        }
    }
    SCL_L;
    return ReceiveByte;
}

//����1�ֹ�����       �����딵��    �������ַ       �������(24c16��SD2403)
uint8_t I2C_WriteByte(uint8_t SendByte,  uint8_t DeviceAddress)
{
    if(!I2C_Start())return 0;
    I2C_SendByte(DeviceAddress);//�O�ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return 0;
    }

    I2C_SendByte(SendByte);
    I2C_WaitAck();
    I2C_Stop();
    //ע�⣺����@�eҪ�ȴ�EEPROM���꣬���Ԓ��ò�ԃ���ӕr��ʽ(10ms)
    //Systick_Delay_1ms(10);
    return 1;
}

//ע�ⲻ�ܿ�퓌�
//����1������      �����딵�M��ַ    �������L��      �������ַ       �������(24c16��SD2403)
uint8_t I2C_BufferWrite(uint8_t* pBuffer, uint8_t length, uint8_t DeviceAddress)
{
    if(!I2C_Start())return 0;
    I2C_SendByte(DeviceAddress);//�O�ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return 0;
    }

    while(length--)
    {
        I2C_SendByte(* pBuffer);
        I2C_WaitAck();
        pBuffer++;
    }
    I2C_Stop();
    //ע�⣺����@�eҪ�ȴ�EEPROM���꣬���Ԓ��ò�ԃ���ӕr��ʽ(10ms)
    //Systick_Delay_1ms(10);
    return 1;
}


//�x��1������         ����x������  ���x���L��      ���x����ַ       �������(24c16��SD2403)
uint8_t I2C_ReadByte(uint8_t* pBuffer,   uint8_t length,  uint8_t DeviceAddress)
{
    if(!I2C_Start())
	  return 0;
    I2C_SendByte( DeviceAddress +1 );//�O�ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()) 
	{  
        I2C_Stop();
        return 0;
    }


    while(length)
    {
        *pBuffer = I2C_ReceiveByte();
        if(length == 1)I2C_NoAck();
        else I2C_Ack();
        pBuffer++;
        length--;
    }
    I2C_Stop();
    return 1;
}

uint8_t I2C_ReadByteAddress(uint8_t* pBuffer,   uint8_t length,  uint8_t DeviceAddress)
{
    if(!I2C_Start())
	  return 0;
    I2C_SendByte( DeviceAddress );//�O�ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()) 
	{  
        I2C_Stop();
        return 0;
    }
	
    I2C_SendByte( pBuffer[0] );//�O�ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()) 
	{  
        I2C_Stop();
        return 0;
    }
	I2C_Stop();
	I2C_Start();
	I2C_SendByte( DeviceAddress + 1 );//�O�ø���ʼ��ַ+������ַ
    while(length)
    {
        *pBuffer = I2C_ReceiveByte();
        if(length == 1)I2C_NoAck();
        else I2C_Ack();
        pBuffer++;
        length--;
    }
    I2C_Stop();
    return 1;
}


#if 0
#define SCL_H         GPIOB->BSRR = GPIO_PIN_6
#define SCL_L         GPIOB->BRR = GPIO_PIN_6

#define SDA_H         GPIOB->BSRR = GPIO_PIN_7
#define SDA_L         GPIOB->BRR = GPIO_PIN_7 

#define SCL_read      GPIOB->IDR & GPIO_PIN_6
#define SDA_read      GPIOB->IDR & GPIO_PIN_7


#define I2C_DIRECTION_TRANSMITTER       ((uint8_t)0x00)
#define I2C_DIRECTION_RECEIVER          ((uint8_t)0x01)

void SDAIN(void)
{
      GPIO_InitTypeDef  GPIO_InitStructure;
    /* Configure I2C1 pins: SCL and SDA */
    GPIO_InitStructure.Pin =  GPIO_PIN_7;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

/**
* @brief get the control of SDA
*/
void SDAOUT(void)
{
      GPIO_InitTypeDef  GPIO_InitStructure;
    /* Configure I2C1 pins: SCL and SDA */
    GPIO_InitStructure.Pin =  GPIO_PIN_7;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void I2C_SoftWare_Master_Init(void)
{
      GPIO_InitTypeDef  GPIO_InitStructure;
    /* Configure I2C1 pins: SCL and SDA */
    GPIO_InitStructure.Pin =  GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    SCL_H;
    SDA_H;
    
    SCL_L;
    SDA_L;
    
    SCL_H;
    SDA_H;
}

	
void I2C_delay(void)
{
    volatile int i = 5;		  
    while (i){
        i--;
        __asm("nop");
    }
}

uint8_t I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();

    SDA_L;
    I2C_delay();

    SCL_L;
    I2C_delay();
    return 1;
}

void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}


void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

uint8_t I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H; 
	 
    I2C_delay();
	SDAIN();
    if (SDA_read)
    {
		SDAOUT();
        SCL_L;
        I2C_delay();
        return 0;
    }
	SDAOUT();
    SCL_L;
    I2C_delay();
    
    return 1;
}

void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--)
    {
        SCL_L;
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
	SDAIN();
    while (i--) 
    {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
		
        if (SDA_read) 
		{
            byte |= 0x01;
        }
    }
    SCL_L;
	SDAOUT();
    return byte;
}

int I2C_SoftWare_Master_Write(uint8_t DeviceAddr, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
    int i;
    if (!I2C_Start())
        return I2C_SoftWare_Master_ReInit();
    I2C_SendByte(DeviceAddr | I2C_DIRECTION_TRANSMITTER);
    if (!I2C_WaitAck())
    {
        I2C_Stop();
        return -1;
    }

    for (i = 0; i < NumByteToWrite; i++) 
    {
        I2C_SendByte(pBuffer[i]);
        if (!I2C_WaitAck()) 
	{
            I2C_Stop();
            return I2C_SoftWare_Master_ReInit();
        }
    }
    
    I2C_Stop();
    
    return 0; 
}

int I2C_SoftWare_Master_Read(uint8_t DeviceAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
    I2C_Start();
    I2C_SendByte(DeviceAddr | I2C_DIRECTION_RECEIVER);
    I2C_WaitAck();
    while (NumByteToRead) 
    {
        *pBuffer = I2C_ReceiveByte();
        if (NumByteToRead == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        pBuffer++;
        NumByteToRead--;
    }
    I2C_Stop();
    
    return 0;
}

int I2C_SoftWare_Master_ReInit(void)
{
    I2C_SoftWare_Master_Init();
  
    return -1;
}
  
#endif