#include "esp01s.h"
#include "usart.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>
#include "oled.h"
//////////////////////////////////////////////////////////////////////////////////	 
//
////////////////////////////////////////////////////////////////////////////////// 	
//-----------------------------------------------
// 外部函数声明
//-----------------------------------------------

//-----------------------------------------------
//外部变量声明

//-----------------------------------------------
// 变量声明
//-----------------------------------------------
//---------------------------------------------------------------
// ESP-01S 初始化序列
// return 0: 失败, 1: 成功
//---------------------------------------------------------------
uint8_t ESP01S_Init_UDP(void)
{ 
    // 测试指令
    OLED_ShowString(0, 32, "Step 1: AT...", 16);
    OLED_Refresh_Gram(); // 刷新OLED显示
    ESP_UART1_SendString("AT\r\n");
    ESP_Delay_ms(2000); // 增加延时，给模块反应时间
    
    // 修改波特率
    OLED_ShowString(0, 32, "Step 2: Baud..", 16);
    OLED_Refresh_Gram(); // 刷新OLED显示
    ESP_UART1_SendString("AT+UART_CUR=460800,8,1,0,0\r\n");
    ESP01S_Switch_Baud_Safe(BRR_460800); // 使用上面那个安全的切换函数
    ESP_Delay_ms(500); // 切换频率后必须停顿，等待电平稳定

    OLED_ShowString(0, 32, "Step 3: AT...", 16);
    OLED_Refresh_Gram(); // 刷新OLED显示
    ESP_UART1_SendString("AT\r\n"); 
    ESP_Delay_ms(500);

    ESP_UART1_SendString("AT+CWMODE=1\r\n");
    ESP_Delay_ms(500);

    // UDP 连接
    // 注意：确保 TARGET_IP 等宏是字符串格式
    OLED_ShowString(0, 32, "Step 3: UDP...", 16);
    OLED_Refresh_Gram(); // 刷新OLED显示
    ESP_UART1_SendString("AT+CIPSTART=\"UDP\",\"192.168.0.111\",12345,1111,0\r\n");
    ESP_Delay_ms(800); // 连接网络耗时较长，增加延时

    ESP_UART1_SendString("AT+CIPMODE=1\r\n");
    ESP_Delay_ms(500);

    OLED_ShowString(0, 32, "Step 4: Send..", 16);
    OLED_Refresh_Gram(); // 刷新OLED显示
    ESP_UART1_SendString("AT+CIPSEND\r\n");
    ESP_Delay_ms(500);
    
    return 1;
}
//---------------------------------------------------------------
// --- 寄存器辅助函数 ---
// 阻塞式发送字符串
//---------------------------------------------------------------
void ESP_UART1_SendString(char* str) 
{
    // 清除错误标志 (SR -> DR 顺序)
    // 即使现在不收数据，也要清除可能导致硬件锁死的 ORE/FE/NE 标志
    volatile uint32_t temp_sr = USART1->SR; 
    volatile uint32_t temp_dr = USART1->DR;
    (void)temp_sr; // 显式忽略未读取警告
    (void)temp_dr;

    while (*str) 
    {
        // 增加安全检查：如果 TXE 没等到，增加一个简单的超时或直接跳过，防止死循环
        uint32_t timeout = 10000; 
        while (!(USART1->SR & (1 << 7))) // 等待 TXE=1 (发送寄存器空)
        {
            if (--timeout == 0) return; // 硬件真坏了也不要卡死在这里
        }
        
        USART1->DR = (uint8_t)(*str++);
    }
}
//---------------------------------------------------------------
// --- 寄存器辅助函数 ---
// 等待发送完成 (TC位)
//---------------------------------------------------------------
void ESP_UART1_WaitTC(void) 
{
    while (!(USART1->SR & (1 << 6))); // 等待 TC=1
}
//---------------------------------------------------------------
// --- 寄存器辅助函数 ---
// 简单的延时函数
//---------------------------------------------------------------
void ESP_Delay_ms(uint32_t ms) 
{
    // 使用 volatile 确保循环不被删除
    volatile uint32_t i = ms * 16800; 
    while (i--);
}
//---------------------------------------------------------------
//
//---------------------------------------------------------------
void ESP01S_Switch_Baud_Safe(uint32_t new_brr) 
{
    // 1. 等待当前的发送完全结束 (TC=1)
    // 必须确保改波特率前，最后一个 bit 已经出了引脚
    while (!(USART1->SR & (1 << 6))); 

    // 2. 暂时关闭串口 (UE=0)
    // 虽然 F4 支持动态改 BRR，但关闭后再改能彻底防止硬件状态机产生毛刺
    USART1->CR1 &= ~(1 << 13); 

    // 3. 修改波特率寄存器
    USART1->BRR = new_brr; 

    // 4. 重新开启串口 (UE=1)
    USART1->CR1 |= (1 << 13); 
    
    // 5. 【关键修正】：必须先读 SR，后读 DR 才能清除 ORE 标志位
    // 即使你现在不打算接收数据，清除这些标志也能防止 CPU 被错误中断堵死
    volatile uint32_t dummy_sr = USART1->SR; // 先读 SR
    volatile uint32_t dummy_dr = USART1->DR; // 再读 DR
    (void)dummy_sr; // 显式告诉编译器这两个变量故意没使用
    (void)dummy_dr;

    // 6. 给硬件和 ESP-01S 一点点稳定时间
    ESP_Delay_ms(100);
}

