#include "dma_uart2.h"
#include <stdio.h> 
#include <string.h> 
#include "exti.h"
#include <stdarg.h>
#include "led.h"
//===============================================================
// 变量定义
uint8_t  USART2_RX_BUF[USART2_MAX_RX_LEN];
uint8_t  USART2_TX_BUF[USART2_MAX_TX_LEN];
volatile uint8_t u8Uart2_rx_flag = 0;
//===============================================================
void DMA_Config_USART2(uint32_t baudrate) {
    // 1. 时钟和引脚配置保持不变
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~((3 << 4) | (3 << 6));
    GPIOA->MODER |= ((2 << 4) | (2 << 6)); 
    GPIOA->AFR[0] |= (7 << 8) | (7 << 12); 

    // 2. 串口配置：清除可能存在的错误标志
    USART2->SR; USART2->DR; // 预读取清除
    USART2->BRR = 42000000 / baudrate; 
    USART2->CR1 = (1 << 3) | (1 << 2) | (1 << 13) | (1 << 4); // TE, RE, UE, IDLEIE
    USART2->CR3 |= (1 << 6) | (1 << 7); // DMAR, DMAT

    // 3. DMA RX 配置 (改为 Normal 模式，删除 1<<8)
    DMA1_Stream5->CR &= ~(1 << 0);
    while(DMA1_Stream5->CR & (1 << 0));
    DMA1_Stream5->PAR = (uint32_t)&(USART2->DR);
    DMA1_Stream5->M0AR = (uint32_t)USART2_RX_BUF;
    DMA1_Stream5->NDTR = USART2_MAX_RX_LEN;
    // Ch4, 优先级中, 内存递增 (Normal 模式，不设置 CIRC)
    DMA1_Stream5->CR = (4U << 25) | (1 << 10); 
    DMA1_Stream5->CR |= (1 << 0);

    // 4. TX 配置保持不变
    DMA1_Stream6->CR &= ~(1 << 0);
    DMA1_Stream6->PAR = (uint32_t)&(USART2->DR);
    DMA1_Stream6->CR = (4U << 25) | (1 << 6) | (1 << 10); 

    NVIC_SetPriority(USART2_IRQn, 2);
    NVIC_EnableIRQ(USART2_IRQn);
}

// 非阻塞数据回报函数
void USART2_DMA_Send(uint8_t *data, uint16_t len) {
    if (len > USART2_MAX_TX_LEN) len = USART2_MAX_TX_LEN;
    
    while(DMA1_Stream6->CR & (1 << 0)); // 等待上次发送完成
    
    memcpy(USART2_TX_BUF, data, len);   // 拷贝到专用发送缓冲区
    DMA1->HIFCR |= (1 << 21) | (1 << 20) | (1 << 19) | (1 << 18); // 清除 Stream6 标志
    DMA1_Stream6->M0AR = (uint32_t)USART2_TX_BUF;
    DMA1_Stream6->NDTR = len;
    DMA1_Stream6->CR |= (1 << 0);       // 开启 DMA 发送
}

// 格式化回报函数
void USART2_Printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    uint16_t len = vsnprintf((char*)USART2_TX_BUF, USART2_MAX_TX_LEN, format, args);
    va_end(args);
    
    // 等待上次 DMA 发送完成
    while(DMA1_Stream6->CR & (1 << 0));
    DMA1->HIFCR |= (0x3D << 16); // 清除 Stream 6 标志
    DMA1_Stream6->M0AR = (uint32_t)USART2_TX_BUF;
    DMA1_Stream6->NDTR = len;
    DMA1_Stream6->CR |= (1 << 0);
}

void USART2_IRQHandler(void) {
    // 处理 IDLE 中断
    if (USART2->SR & (1 << 4)) { 
        // 1. 清除 IDLE 标志位 (顺序：SR -> DR)
        uint32_t temp;
        temp = USART2->SR;
        temp = USART2->DR;
        (void)temp;

        // 2. 暂时关闭 DMA 以读取 NDTR 并重置
        DMA1_Stream5->CR &= ~(1 << 0);
        while(DMA1_Stream5->CR & (1 << 0)); // 等待关闭

        // 3. 计算长度并设置标志位
        uint16_t rx_len = USART2_MAX_RX_LEN - (uint16_t)(DMA1_Stream5->NDTR);
        if (rx_len > 0) {
            USART2_RX_BUF[rx_len] = '\0'; // 强制封包
            u8Uart2_rx_flag = 1;         // 触发主循环任务
        }

        // 4. 【关键】清除 DMA1 Stream5 的所有中断标志位
        // Stream5 在 HIFCR 寄存器中，对应位是 6, 8, 9, 10, 11
        DMA1->HIFCR |= (0x3D << 6); 

        // 5. 重置计数并重新启动
        DMA1_Stream5->NDTR = USART2_MAX_RX_LEN;
        DMA1_Stream5->CR |= (1 << 0);
    }
    
    // 强制清除溢出错误 (ORE)，防止串口死锁
    if (USART2->SR & (1 << 3)) {
        uint32_t temp = USART2->SR;
        temp = USART2->DR;
        (void)temp;
    }
}

