#include "dma_uart1.h"
#include <stdio.h> 
#include <string.h> 
#include "exti.h"
//=============================================
#define DMA_TX_BUFFER_SIZE  128  // 发送缓冲区大小
#define DMA_RX_BUFFER_SIZE  256  // 接收缓冲区大小 (建议2的幂次方)
//==============================================================================
// 变量定义 (真正的内存分配发生在这里)
//==============================================================================
uint8_t dma_tx_buffer[DMA_TX_BUFFER_SIZE];
uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE];
volatile uint32_t DMA_Busy_Drop_Count = 0;

PID_Params_t Balance_PID = {10.0f, 0.0f, 0.5f};
PID_Params_t Velocity_PID = {1.0f, 0.1f, 0.0f};
//==============================================================================
// USART1 相关的 DMA 配置函数
//============================================================================== 
void DMA_Config_USART1(void)
{
    // 1. 开启 DMA2 时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // RX 配置: DMA2 Stream2 Channel4 (Circular Mode)
    // 1.1 确保 Stream 关闭
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream2->CR & DMA_SxCR_EN);

    // 1.2 清除 RX 相关中断标志 (LIFCR)
    DMA2->LIFCR |= (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2);

    // 1.3 设置地址
    DMA2_Stream2->PAR = (uint32_t)&(USART1->DR);      // 外设地址
    DMA2_Stream2->M0AR = (uint32_t)dma_rx_buffer;     // 内存地址
    DMA2_Stream2->NDTR = DMA_RX_BUFFER_SIZE;          // 缓冲区大小

    // 1.4 配置控制寄存器 CR
    // Ch4(100), Periph2Mem(00), Circ(1), Minc(1), Pinc(0), PriorityHigh(10)
    DMA2_Stream2->CR = (4U << 25) | (0U << 6) | (1U << 8) | (1U << 10) | (2U << 16);

    // 1.5 立即开启 RX DMA (一直运行)
    DMA2_Stream2->CR |= DMA_SxCR_EN;


    // TX 配置: DMA2 Stream7 Channel4 (Normal Mode)
    // 2.1 确保 Stream 关闭
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while(DMA2_Stream7->CR & DMA_SxCR_EN);

    // 2.2 清除 TX 相关中断标志 (HIFCR)
    DMA2->HIFCR |= (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7);

    // 2.3 设置外设地址 (M0AR 和 NDTR 在发送函数中动态设置)
    DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);

    // 2.4 配置控制寄存器 CR
    // Ch4(100), Mem2Periph(01), Normal(0), Minc(1), Pinc(0), PriorityHigh(10)
    // 注意：这里不开启 EN，等到要发送时才开 增加 (1U << 4) 代表使能传输完成中断 (TCIE)
    DMA2_Stream7->CR = (4U << 25) | (1U << 6) | (1U << 10) | (2U << 16) | (1U << 4);

    // USART1 TX (Stream 7) 优先级：抢占 1, 响应 1 (略低于接收)
    MY_NVIC_Init(1, 1, DMA2_Stream7_IRQn, 2);
}
//------------------------------------------------
//
//------------------------------------------------
uint8_t DMA_USART1_Start_TX_DoubleBuf(uint32_t buffer_addr, uint16_t len)
{
    // 1. 检查 DMA 是否忙碌 (EN位)
    // 如果上一次  的数据还没发完，说明波特率太低或数据量太大
    if ((DMA2_Stream7->CR & (1 << 0)) != 0) 
    {
        DMA_Busy_Drop_Count++; // 记录：因为 DMA 还没发完而丢弃的包
        return 1; 
    }

    // 2. 清除上次发送完成标志 (HIFCR)
    // Stream7 对应通道的 CTCIF7(完成), CHTIF7(半完成), CTEIF7(错误) 等
    DMA2->HIFCR |= (0x3D << 22); // 0x3D 清除 Stream7 的所有中断标志

    // 3. 设置本次发送长度 (NDTR)
    DMA2_Stream7->NDTR = len;

    // 4. 【核心修改】设置本次发送的内存地址
    // 传入 Ping 或 Pong 缓冲区的首地址
    DMA2_Stream7->M0AR = buffer_addr;

    // 5. 开启 DMA
    DMA2_Stream7->CR |= (1 << 0);

    return 0; 
}
//------------------------------------------------
//
//------------------------------------------------
uint32_t DMA_Get_RX_Current_Pos(void)
{
    // NDTR 是倒计数的，所以用 总大小 - 剩余大小 = 当前位置
    return DMA_RX_BUFFER_SIZE - DMA2_Stream2->NDTR;
}
//------------------------------------------------
// usart1 发送
//------------------------------------------------
void DMA2_Stream7_IRQHandler(void)
{
    // 1. 检查是否是传输完成中断 (TCIF7)
    if (DMA2->HISR & (1 << 27)) 
    {
        DMA2->HIFCR |= (1 << 27); // 清除 TCIF7
        
        // 你的逻辑: 比如设置发送完成标志位
    }
    
    // 2. 建议增加：检查并清除传输错误标志 (TEIF7)
    // HISR 的第 25 位是 Stream7 的传输错误标志
    if (DMA2->HISR & (1 << 25))
    {
        DMA2->HIFCR |= (1 << 25); // 清除 TEIF7
        // 调试用：可以在这里点亮一个红灯
    }
}
//-------------------------------------------------------------------------------
// USART1 接收配置
//  DMA2_Stream5
//-------------------------------------------------------------------------------
void USART1_DMA_Rx_Config(void) {
    // 1. 使能 DMA2 时钟
    RCC->AHB1ENR |= (1 << 22); 

    // 2. 配置 DMA2_Stream5 (USART1_RX)
    DMA2_Stream5->CR = 0; // 先清零
    while(DMA2_Stream5->CR & (1 << 0)); // 等待开启位清零

    DMA2_Stream5->PAR = (uint32_t)&(USART1->DR);    // 外设地址：USART1 数据寄存器
    DMA2_Stream5->M0AR = (uint32_t)dma_rx_buffer;       // 内存地址：自定义缓冲区
    DMA2_Stream5->NDTR = DMA_RX_BUFFER_SIZE;               // 接收数据长度
    
    // CR 配置：通道4, 优先级中, 内存递增, 外设不递增, 内存单次模式(或循环模式)
    DMA2_Stream5->CR |= (4 << 25);  // Channel 4
    DMA2_Stream5->CR |= (1 << 10);  // MINC: 内存地址递增
    DMA2_Stream5->CR &= ~(1 << 6);  // DIR: 00 代表外设到内存
    
    DMA2_Stream5->CR |= (1 << 0);   // 使能 DMA Stream

    // 3. 配置 USART1
    USART1->CR3 |= (1 << 6);        // DMAR: 使能串口 DMA 接收
    USART1->CR1 |= (1 << 4);        // IDLEIE: 使能空闲中断
}
//=========================================================
//		 USART1 中断服务程序  DMA方式		  
//=========================================================
void USART1_IRQHandler(void) {
    if (USART1->SR & (1 << 4)) { // 监测到 IDLE
        uint8_t temp = (uint8_t)(USART1->SR);
        temp = (uint8_t)(USART1->DR);
        (void)temp;

        DMA2_Stream5->CR &= ~(1 << 0); // 关闭 DMA
        
        // 修正后的 rx_len 计算
        uint16_t rx_len = DMA_RX_BUFFER_SIZE - (uint16_t)(DMA2_Stream5->NDTR);

        if (rx_len >= sizeof(CommandPacket_t)) {
            for (int i = 0; i <= rx_len - sizeof(CommandPacket_t); i++) {
                if (dma_rx_buffer[i] == 0xAA && dma_rx_buffer[i+1] == 0x55) {
                    //ParseCommandPacket((CommandPacket_t*)&dma_rx_buffer[i]); //
                    break;
                }
            }
        }

        DMA2_Stream5->NDTR = DMA_RX_BUFFER_SIZE; // 重置计数器
        DMA2_Stream5->CR |= (1 << 0);    // 重新开启 DMA
    }
}


