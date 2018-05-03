/******************************************************************************
 * @file    vcom.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   manages virtual com port
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
#include <stdarg.h>
#include "hw.h"
#include "vcom.h"
//#include "stm32l0xx_ll_lpuart.h"
//#include "stm32l0xx_ll_rcc.h"
//#include "stm32l0xx_ll_dma.h"
#include "low_power_manager.h"
#include "tiny_vsnprintf.h"
#include "delay.h"
#include "usart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef TRACE
#define BUFSIZE_TX 256
#else
#define BUFSIZE_TX 128
#endif

#define BUFSIZE_RX 8
#define MAX_PRINT_SIZE 128
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

typedef struct {
  char buff[BUFSIZE_TX];   /* buffer to transmit */
  __IO int iw;             /* 1st free index in BuffTx */
  int ir;                  /* next char to read in buffTx */
  __IO int dmabuffSize;
} circ_buff_tx_t;

typedef struct {
  char buff[BUFSIZE_RX];   /* buffer to receive */
  __IO int iw;             /* 1st free index in BuffRx */
  int ir;                  /* next char to read in buffRx */
} circ_buff_rx_t;

static struct {
  circ_buff_rx_t rx;        /* UART rx buffer context*/
  circ_buff_tx_t tx;        /* UART tx buffer context */
} uart_context;             /* UART context*/

static struct {
  char buffer[10];        /* low power buffer*/
  int len;                /* low power buffer length */
} SleepBuff;              /* low power structure*/

volatile char rx;
volatile char dmadone=1;
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Takes one character that has been received and save it in uart_context.buffRx
 * @param  received character
 */
static void receive(char rx);

/**
 * @brief  prepare DMA print
 * @param  None
 */
static void vcom_PrintDMA(void);

/**
 * @brief  Starts DMA transfer into UART
 * @param  buffer adress to start
 * @param  length of buffer to transfer
 */
static void vcom_StartDMA(char* buf, uint16_t buffLen);

extern DMA_HandleTypeDef hdma_usart2_tx;
/* Functions Definition ------------------------------------------------------*/

void vcom_Init(void)
{

}

void vcom_DeInit(void)
{

}

void vcom_IoInit(void)
{

}

void vcom_IoDeInit(void)
{
	#if 0
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  HW_GPIO_Init(UARTX_TX_GPIO_PORT, UARTX_TX_PIN, &GPIO_InitStructure);

  HW_GPIO_Init(UARTX_RX_GPIO_PORT, UARTX_RX_PIN, &GPIO_InitStructure);
	#endif
}

void vcom_ReceiveInit(void)
{
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx, 1);

}

void vcom_Send( const char *format, ... )
{
  va_list args;
  va_start(args, format);
  uint8_t len=0;
  uint8_t lenTop;
  char tempBuff[MAX_PRINT_SIZE];
  int32_t freebuff;

  /* calculate free buffer size*/
  /*in case freebuff is negative this is an overrun*/
  freebuff = BUFSIZE_TX - (uart_context.tx.iw-uart_context.tx.ir);

  if (SleepBuff.len!=0)
  {
    /*if SleepBuff has been filled before entering lowpower, prepend it */
    memcpy(&tempBuff[0], SleepBuff.buffer, SleepBuff.len);
    len = tiny_vsnprintf_like(&tempBuff[SleepBuff.len], sizeof(tempBuff), format, args); 
    len += SleepBuff.len;
    /*erase SleepBuff*/
    memset(SleepBuff.buffer, 0,sizeof(SleepBuff.buffer) );
    SleepBuff.len=0;
  }
  else
  {
  /*convert into string at buff[0] of length iw*/
    len = tiny_vsnprintf_like(&tempBuff[0], sizeof(tempBuff), format, args); 
  }
  
  if (len>freebuff)
  {
    /*wait enough free char in buff*/
    /*1 char at 9600 lasts approx 1ms*/
    DelayMs(len-freebuff);
  }

  if (((uart_context.tx.iw)%BUFSIZE_TX)+len<BUFSIZE_TX)
  {
    memcpy( &uart_context.tx.buff[((uart_context.tx.iw)%BUFSIZE_TX)], &tempBuff[0], len);
    uart_context.tx.iw+=len;
  }
  else
  {
    /*cut buffer in high/low part*/
    lenTop= BUFSIZE_TX - ((uart_context.tx.iw)%BUFSIZE_TX);
    /*copy beginning at top part of the circ buf*/
    memcpy( &uart_context.tx.buff[((uart_context.tx.iw)%BUFSIZE_TX)], &tempBuff[0], lenTop);
     /*copy end at bottom part of the circ buf*/
    memcpy( &uart_context.tx.buff[0], &tempBuff[lenTop], len-lenTop);
    uart_context.tx.iw += len;
  }
//if((hdma_usart2_tx.Instance->CCR & DMA_CCR_EN) !=0)
//  if (HAL_DMA_GetState(&hdma_usart2_tx) != HAL_DMA_STATE_BUSY )
	if(dmadone)
//  if (! LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_7) )
  {
    vcom_PrintDMA();
  }
  
  va_end(args);
}

void vcom_Send_Lp(const char *format, ...)
{
  /*special vcomsend to avoid waking up any time MCU goes to sleep*/
  va_list args;
  va_start(args, format);
  
  SleepBuff.len = tiny_vsnprintf_like(&SleepBuff.buffer[0], sizeof(SleepBuff.buffer), format, args); 
  
  va_end(args);
}

FlagStatus IsNewCharReceived(void)
{
  FlagStatus status;
  
  BACKUP_PRIMASK();
  DISABLE_IRQ();
  
  status = ((uart_context.rx.iw == uart_context.rx.ir) ? RESET : SET);
  
  RESTORE_PRIMASK();
  return status;
}

uint8_t GetNewChar(void)
{
  uint8_t NewChar;

  BACKUP_PRIMASK();
  DISABLE_IRQ();

  NewChar = uart_context.rx.buff[uart_context.rx.ir];
  uart_context.rx.ir = (uart_context.rx.ir + 1) % sizeof(uart_context.rx.buff);

  RESTORE_PRIMASK();
  return NewChar;
}

void vcom_IRQHandler(void)
{

}

static void receive(char rx)
{
  int next_free;

  /** no need to clear the RXNE flag because it is auto cleared by reading the data*/
  uart_context.rx.buff[uart_context.rx.iw] = rx;
  next_free = (uart_context.rx.iw + 1) % sizeof(uart_context.rx.buff);
  if (next_free != uart_context.rx.iw)
  {
    /* this is ok to read as there is no buffer overflow in input */
    uart_context.rx.iw = next_free;
  }
  else
  {
    /* force the end of a command in case of overflow so that we can process it */
    uart_context.rx.buff[uart_context.rx.iw] = '\r';
    DBG_PRINTF("uart_context.buffRx buffer overflow %d\r\n");
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	receive(rx);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(!dmadone)
	{
		dmadone = 1;

		uart_context.tx.ir += uart_context.tx.dmabuffSize;
		if ( uart_context.tx.ir!= uart_context.tx.iw)
		{
			/*continue if more has been written in buffer meanwhile*/
			vcom_PrintDMA();
		}	
	}
}

void vcom_Dma_IRQHandler( void )
{

}

static void vcom_PrintDMA(void)
{
  uint16_t write_idx=  (uart_context.tx.iw)%BUFSIZE_TX;
  uint16_t read_idx=   (uart_context.tx.ir)%BUFSIZE_TX;
  /*shall not go in stop mode while printing*/
  LPM_SetStopMode(LPM_UART_TX_Id, LPM_Disable);

  if (write_idx > read_idx)
  {
    /*contiguous buffer[ir..iw]*/
    uart_context.tx.dmabuffSize= write_idx - read_idx;
    vcom_StartDMA( &uart_context.tx.buff[read_idx], uart_context.tx.dmabuffSize);
  }
  else
  {
    /*[ir:BUFSIZE_TX-1] and [0:iw]. */
     uart_context.tx.dmabuffSize= BUFSIZE_TX-read_idx;
     /*only [ir:BUFSIZE_TX-1] sent, rest will be sent in dma  handler*/
     vcom_StartDMA( &uart_context.tx.buff[read_idx], uart_context.tx.dmabuffSize);
  }
}

static void vcom_StartDMA(char* buf, uint16_t buffLen)
{
	//uart_context.tx.ir += uart_context.tx.dmabuffSize;
	dmadone = 0;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, buffLen);

}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
