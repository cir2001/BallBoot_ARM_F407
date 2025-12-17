#include "dma_driver.h"
#include <stdio.h> 
#include <string.h> 
//==============================================================================
// 变量定义 (真正的内存分配发生在这里)
//==============================================================================
uint8_t dma_tx_buffer[DMA_TX_BUFFER_SIZE];
uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE];

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
    // 注意：这里不开启 EN，等到要发送时才开
    DMA2_Stream7->CR = (4U << 25) | (1U << 6) | (1U << 10) | (2U << 16);
}

uint8_t DMA_USART1_Start_TX(uint16_t len)
{
    // 1. 检查 DMA 是否忙碌 (EN位)
    if ((DMA2_Stream7->CR & DMA_SxCR_EN) != 0)
    {
        return 1; // 忙碌，发送失败
    }

    // 2. 清除上次发送完成标志 (HIFCR)
    // Stream7 对应 High Interrupt Flag Clear Register
    DMA2->HIFCR |= (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7);

    // 3. 设置本次发送长度
    DMA2_Stream7->NDTR = len;

    // 4. 设置内存地址 (虽然通常不变，但为了安全建议每次重设)
    DMA2_Stream7->M0AR = (uint32_t)dma_tx_buffer;

    // 5. 开启 DMA
    DMA2_Stream7->CR |= DMA_SxCR_EN;

    return 0; // 成功
}

uint32_t DMA_Get_RX_Current_Pos(void)
{
    // NDTR 是倒计数的，所以用 总大小 - 剩余大小 = 当前位置
    return DMA_RX_BUFFER_SIZE - DMA2_Stream2->NDTR;
}
//==============================================================================
// USART2 相关的 DMA 配置函数
//============================================================================== 
void DMA_Config_USART2(void)
{
    // 1. 【关键】开启 DMA1 时钟 (注意是 DMA1)
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // --- TX 配置: DMA1 Stream6 Channel4 ---
    
    // 2.1 确保 Stream 关闭
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while(DMA1_Stream6->CR & DMA_SxCR_EN);

    // 2.2 清除 Stream6 所有中断标志
    // Stream6 在 HIFCR (High Interrupt Flag Clear Register)
    // 对应的位通常是 [21:16] 附近的区域，建议全清
    // Stream6 对应 TCIF6, HTIF6 等，位于 HIFCR 的 Bit 21, 20, 19, 18, 16
    DMA1->HIFCR |= (0x3F << 16); 

    // 2.3 设置外设地址 -> USART2 的数据寄存器
    DMA1_Stream6->PAR = (uint32_t)&(USART2->DR);

    // 2.4 设置内存地址 (缓冲区)
    DMA1_Stream6->M0AR = (uint32_t)dma_tx_buffer;

    // 2.5 配置控制寄存器 CR
    // Channel 4 (100) -> 查表得 DMA1 Stream6 Ch4 是 USART2_TX
    // Dir Mem2Periph (01)
    // Minc Enable (1)
    // PINC Disable (0)
    // Priority High (10)
    DMA1_Stream6->CR = (4U << 25) | (1U << 6) | (1U << 10) | (2U << 16);
    
    // 注意：此时不开启 EN，等发送函数来开
}
//===========================================================
//  填充数据并启动 DMA 发送
//===========================================================
void IMU_Send_Data(float gx, float gy, float gz, 
                   float ax, float ay, float az, 
                   float pitch, float roll, float yaw)
{
    int len;

    // 1. 检查 DMA1 Stream6 是否忙碌
    if (DMA1_Stream6->CR & 1) return;

    // 2. 清除 Stream6 传输完成标志 (HIFCR)
    // 0x3D << 16 是掩码，覆盖 Stream6 的所有标志位
    DMA1->HIFCR = (0x3F << 16);

    // 3. 格式化数据
    len = sprintf((char*)dma_tx_buffer, 
                  "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                  gx, gy, gz, ax, ay, az, pitch, roll, yaw);

    // 4. 设置长度
    DMA1_Stream6->NDTR = len;
    
    // 5. 确保地址正确
    DMA1_Stream6->M0AR = (uint32_t)dma_tx_buffer;

    // 6. 开启 DMA
    DMA1_Stream6->CR |= 1;
}




