#ifndef __DMA_DRIVER_H
#define __DMA_DRIVER_H

#include "stm32f4xx.h"
// ==============================================================================
//  宏定义：缓冲区大小
// ============================================================================== 

// ==============================================================================
//  函数声明
// ============================================================================== */
// 初始化 USART1 相关的 DMA (TX: Stream7, RX: Stream2) */
void DMA_Config_USART1(void);
// 初始化 USART2 相关的 DMA (TX: Stream6, RX: Stream2) */
void DMA_Config_USART2(void);

/**
 * @brief 启动 USART1 DMA 发送指定地址的数据
 * @param buffer_addr: 要发送的缓冲区起始地址 (TX_Buffer_0 或 TX_Buffer_1 的地址)
 * @param len: 发送长度 (通常为 sizeof(BatchPacket_t))
 * @return 0: 成功, 1: DMA 忙碌
 */
uint8_t DMA_USART1_Start_TX_DoubleBuf(uint32_t buffer_addr, uint16_t len);


#endif /* __DMA_DRIVER_H */


