/*
 * uart.h
 *
 *  Created on: 31 Oct 2016
 *      Author: Steve Chang
 */

#ifndef _USART_H
#define _USART_H

#include <stdint.h>

#define USART_TX_DMA 1
#define USART3_LIN_BUS 1

#define MAX_TX_LEN 128
#define MAX_RX_LEN 128 // this is the maximum string length of our string in characters

void Usart2_Init(uint32_t baudrate);
void Usart2_Puts(char *string);
void Usart2_Printf(const char *fmt, ...);
void Usart2_Write(uint8_t *data, uint8_t len);
char *Usart2_Gets(void);
int Usart2_Read(uint8_t *data, uint8_t len);
int Usart2_Poll(void);

void Usart3_Init(uint32_t baudrate);
void Usart3_Puts(char *string);
void Usart3_Printf(const char *fmt, ...);
void Usart3_Write(uint8_t *data, uint8_t len);
char *Usart3_Gets(void);
int Usart3_Read(uint8_t *data, uint8_t len);
int Usart3_Poll(void);

#endif
