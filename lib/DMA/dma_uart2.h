#ifndef __DMA_UART2_H
#define __DMA_UART2_H

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

#define USART2_MAX_RX_LEN 128
#define USART2_MAX_TX_LEN 128

// 函数声明
void DMA_Config_USART2(uint32_t baudrate);
void USART2_DMA_Send(uint8_t *data, uint16_t len);
void USART2_Printf(const char *format, ...);

#endif