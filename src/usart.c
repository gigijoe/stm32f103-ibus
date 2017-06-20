/*
 * uart.c
 *
 *  Created on: 31 Oct 2016
 *      Author: Steve Chang
 */
//#include "stm32f4xx_conf.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "usart.h"

#ifndef USART_TX_DMA

static void Usart_Puts(USART_TypeDef* USARTx, char *string)
{
    while(*string){
        /* 傳送訊息至 USARTx */
        USART_SendData(USARTx, (uint16_t) *string++);
 
        /* 等待訊息傳送完畢 */
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
    }
}

static void Usart_Write(USART_TypeDef* USARTx, uint8_t *data, uint8_t len)
{
    while(len){
        /* 傳送訊息至 USART2 */
        USART_SendData(USARTx, (uint16_t) *data++);
 
        /* 等待訊息傳送完畢 */
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
        len--;
    }	
}

#endif

typedef struct {
	char data[MAX_RX_LEN];
	int len;
} UsartRx;

static char *Usart_Gets(UsartRx *ur)
{
	static char str[MAX_RX_LEN];

	if(ur->len) {
		memset(str, 0, MAX_RX_LEN);
		memcpy(str, ur->data, ur->len);
		memset(ur->data, 0, MAX_RX_LEN);
		ur->len = 0;
		return str;
	}
	
	return 0;	
}

static int Usart_Read(UsartRx *ur, uint8_t *data, uint8_t len)
{
	int r = 0;
	if(ur->len) {
		if(data == 0) {
			r = ur->len;
			ur->len = 0;
			return r;
		}

		if(ur->len > len) {
			char str[MAX_RX_LEN];
			memcpy(data, ur->data, len);
			memset(str, 0, MAX_RX_LEN);
			memcpy(str, ur->data + len, ur->len - len);
			memset(ur->data, 0, MAX_RX_LEN);
			ur->len -= len;
			memcpy(ur->data, str, ur->len);
			r = len;
		} else {
			memcpy(data, ur->data, ur->len);
			memset(ur->data, 0, MAX_RX_LEN);
			r = ur->len;
			ur->len = 0;
		}
	}

	return r;	
}

static int Usart_Poll(UsartRx *ur)
{
	return ur->len;
}

/*
*
*/

typedef enum { false = 0, true} bool_t;

/*
*
*/

#if USART_TX_DMA

static char usart2_tx_data[MAX_TX_LEN];

#endif

static char usart2_rx_data[MAX_RX_LEN];

static UsartRx usart2_rx = {{0}}; /* Since the first member in the structure is an array so it need: {{0}}; */

void Usart2_Init(uint32_t baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;

	/* enable peripheral clock for USART2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* GPIOA clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* GPIOA Configuration:  USART2 TX on PA2, RX on PA3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	/* Here the USART2 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART2_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */

	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

#ifdef USART_TX_DMA
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Enable USART2 DMA TX Finish Interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	USART_Cmd(USART2, ENABLE); // enable USART2

	DMA_InitTypeDef  DMA_InitStructure;

#ifdef USART_TX_DMA
	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; // Transmit
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart2_tx_data;
	DMA_InitStructure.DMA_BufferSize = MAX_TX_LEN;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);
	/* Enable the USART Tx DMA request */
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	/* Enable DMA Stream Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
	/* Disable the DMA Tx Stream */
	DMA_Cmd(DMA1_Channel7, DISABLE);
	/* Enable the USART1 Tx DMA Interrupt */
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
#endif
	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
      
	DMA_DeInit(DMA1_Channel6);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // Receive
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart2_rx_data;
	DMA_InitStructure.DMA_BufferSize = MAX_RX_LEN;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
	/* Enable the USART Rx DMA request */
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	/* Enable DMA Stream Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
	/* Enable the DMA Rx Stream */
	DMA_Cmd(DMA1_Channel6, ENABLE);
	/* Enable the USART1 Rx DMA Interrupt */
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);	
}

#ifdef USART_TX_DMA

void DMA1_Channel7_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetFlagStatus(DMA1_IT_TC7)) {
    	/* Clear DMA Stream Transfer Complete interrupt pending bit */
    	DMA_ClearITPendingBit(DMA1_IT_TC7);
	}
}

#endif

void DMA1_Channel6_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetFlagStatus(DMA1_IT_TC6)) {	
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_IT_TC6);
	}
}

void Usart2_Puts(char *string) 
{
#if USART_TX_DMA
	uint16_t len = strlen(string);  
	if(len > MAX_TX_LEN)
		len = MAX_TX_LEN;	

	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

	DMA_Cmd(DMA1_Channel7, DISABLE);
	memcpy(usart2_tx_data, string, len); 
	DMA_SetCurrDataCounter(DMA1_Channel7, len);
	DMA_Cmd(DMA1_Channel7, ENABLE);
#else
	Usart_Puts(USART2, string);
#endif
}

void Usart2_Printf(const char *fmt, ...)
{
  char str[MAX_TX_LEN];
  va_list args;

  va_start(args, fmt);
  vsnprintf(str, MAX_TX_LEN, fmt, args);
  Usart2_Puts(str);
  va_end(args);
}

void Usart2_Write(uint8_t *data, uint8_t len)
{
#if USART_TX_DMA
	if(len > MAX_TX_LEN)
		len = MAX_TX_LEN;
	
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

	DMA_Cmd(DMA1_Channel7, DISABLE);
	memcpy(usart2_tx_data, data, len); 
	DMA_SetCurrDataCounter(DMA1_Channel7, len);
	DMA_Cmd(DMA1_Channel7, ENABLE);
#else
	Usart_Write(USART2, data, len);
#endif
}

char *Usart2_Gets(void)
{
	return Usart_Gets(&usart2_rx);
}

int Usart2_Read(uint8_t *data, uint8_t len)
{
	return Usart_Read(&usart2_rx, data, len);
}

int Usart2_Poll(void)
{
	return Usart_Poll(&usart2_rx);
}

void USART2_IRQHandler(void)
{	
	uint16_t len = 0;  
     
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) /* 接收完成中断 */
    {    	
        USART2->SR;  
        USART2->DR; //清USART_IT_IDLE标志  
        //关闭DMA  
        DMA_Cmd(DMA1_Channel6, DISABLE);  
        //清除标志位  
        DMA_ClearFlag(DMA1_FLAG_TC6);            
        //获得接收帧帧长  
        len = MAX_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel6);  

		if(len > 0 && (usart2_rx.len + len) < MAX_RX_LEN) {
			memcpy(&usart2_rx.data[usart2_rx.len], usart2_rx_data, len);
			usart2_rx.len += len;
		} else {
			/* USART 2 over flow !!! */
		}
        //设置传输数据长度  
        DMA_SetCurrDataCounter(DMA1_Channel6, MAX_RX_LEN);  
        //打开DMA  
        DMA_Cmd(DMA1_Channel6, ENABLE);  
    }
}

/*
*
*/

#if USART_TX_DMA

static char usart3_tx_data[MAX_TX_LEN];

#endif

static char usart3_rx_data[MAX_RX_LEN];
static UsartRx usart3_rx = {{0}}; /* Since the first member in the structure is an array so it need: {{0}}; */
#ifdef USART3_LIN_BUS
static bool_t linBusBusy = false;
#endif

void Usart3_Init(uint32_t baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;

	/* enable peripheral clock for USART2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* GPIOB Configuration:  USART2 TX on PB10, RX on PB11 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
#ifdef USART3_LIN_BUS
	USART_InitStructure.USART_Parity = USART_Parity_Even;
#else
	USART_InitStructure.USART_Parity = USART_Parity_No;
#endif
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &USART_InitStructure);

	/* Here the USART2 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART2_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

#ifdef USART_TX_DMA
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Enable USART2 DMA TX Finish Interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

#ifdef USART3_LIN_BUS
	USART_LINBreakDetectLengthConfig(USART3, USART_LINBreakDetectLength_11b);
	USART_LINCmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_LBD, ENABLE); // enable the USART1 receive interrupt 
#endif

	USART_Cmd(USART3, ENABLE); // enable USART3

	DMA_InitTypeDef  DMA_InitStructure;

#if USART_TX_DMA
	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; // Transmit
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart3_tx_data;
	DMA_InitStructure.DMA_BufferSize = MAX_TX_LEN;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	/* Enable the USART Tx DMA request */
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	/* Enable DMA Stream Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	/* Disable the DMA Tx Stream */
	DMA_Cmd(DMA1_Channel2, DISABLE);
	/* Enable the USART1 Tx DMA Interrupt */
	NVIC_EnableIRQ(DMA1_Channel2_IRQn); 
#endif

	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
      
	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // Receive
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart3_rx_data;
	DMA_InitStructure.DMA_BufferSize = MAX_RX_LEN;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	/* Enable the USART Rx DMA request */
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	/* Enable DMA Stream Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	/* Enable the DMA Rx Stream */
	DMA_Cmd(DMA1_Channel3, ENABLE);
	/* Enable the USART1 Rx DMA Interrupt */
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);		
}

#if USART_TX_DMA

void DMA1_Channel2_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetFlagStatus(DMA1_IT_TC2)) {
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_IT_TC2);
	}
}

#endif

void DMA1_Channel3_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetFlagStatus(DMA1_IT_TC3)) {
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_IT_TC3);
	}  
}

void Usart3_Puts(char *string) 
{
#ifdef USART3_LIN_BUS
	USART_SendBreak(USART3);
#endif
#if USART_TX_DMA
	uint16_t len = strlen(string);  
	if(len > MAX_TX_LEN)
		len = MAX_TX_LEN;

	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

	DMA_Cmd(DMA1_Channel2, DISABLE);
	memcpy(usart3_tx_data, string, len); 
	DMA_SetCurrDataCounter(DMA1_Channel2, len);
	DMA_Cmd(DMA1_Channel2, ENABLE);
#else
	Usart_Puts(USART3, string);
#endif
}

void Usart3_Printf(const char *fmt, ...)
{
  char str[MAX_TX_LEN];
  va_list args;

  va_start(args, fmt);
  vsnprintf(str, MAX_TX_LEN, fmt, args);
  Usart3_Puts(str);
  va_end(args);
}

void Usart3_Write(uint8_t *data, uint8_t len)
{
#ifdef USART3_LIN_BUS
	while(linBusBusy);

	USART_SendBreak(USART3);
#endif
#if USART_TX_DMA
	if(len > MAX_TX_LEN)
		len = MAX_TX_LEN;

	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

	DMA_Cmd(DMA1_Channel2, DISABLE);
	memcpy(usart3_tx_data, data, len); 
	DMA_SetCurrDataCounter(DMA1_Channel2, len);
	DMA_Cmd(DMA1_Channel2, ENABLE);
#else
	Usart_Write(USART3, data, len);
#endif
}

char *Usart3_Gets(void)
{
	return Usart_Gets(&usart3_rx);
}

int Usart3_Read(uint8_t *data, uint8_t len)
{
	return Usart_Read(&usart3_rx, data, len);
}

int Usart3_Poll(void)
{
	return Usart_Poll(&usart3_rx);
}

void USART3_IRQHandler(void)
{
	uint16_t len = 0;  
	
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) /* 接收完成中断 */
    {  
        USART3->SR;  
        USART3->DR; //清USART_IT_IDLE标志  
        //关闭DMA  
        DMA_Cmd(DMA1_Channel3, DISABLE);  
        //清除标志位  
        DMA_ClearFlag(DMA1_FLAG_TC3);            
        //获得接收帧帧长  
        len = MAX_RX_LEN - DMA_GetCurrDataCounter(DMA1_Channel3);  

		if(len > 0 && (usart3_rx.len + len) < MAX_RX_LEN){
			memcpy(&usart3_rx.data[usart3_rx.len], usart3_rx_data, len);
			usart3_rx.len += len;
		} else {
			/* USART 3 over flow !!! */
		}
        //设置传输数据长度  
        DMA_SetCurrDataCounter(DMA1_Channel3, MAX_RX_LEN);  
        //打开DMA  
        DMA_Cmd(DMA1_Channel3, ENABLE);  
#ifdef USART3_LIN_BUS
        linBusBusy = false;
#endif
    }

#ifdef USART3_LIN_BUS
	if(USART_GetITStatus(USART3, USART_IT_LBD) != RESET) {
		USART_ClearITPendingBit(USART3, USART_IT_LBD);     //清break中断位
		linBusBusy = true;
	}
#endif
}

