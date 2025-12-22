#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "key.h"
#include "dma_driver.h"
#include "mpu6500_driver.h"
#include "mpuiic.h"
//////////////////////////////////////////////////////////////////////////////////	

//---------------------------------------------
volatile uint32_t sys_ms_ticks = 0;      // 全局毫秒时间戳
volatile uint8_t g_mpu_read_ready = 0; // MPU读取触发标志


u16 u16EXIT0Count,u16EXIT1Count;						  
////////////////////////////////////////////////////////////////////////////////// 
//外部中断0服务程序
void EXTI0_IRQHandler(void)
{
	// 1. 核心计数：记录“传感器运行时间”
    sys_ms_ticks++; 
    
    // 2. 状态闪烁 (每250次采样翻转一次)
    u16EXIT0Count++;
    if(u16EXIT0Count >= 250) {
        u16EXIT0Count = 0;
        LED_MPU = !LED_MPU; 
    }

    // 3. 发送读取指令给主循环
    g_mpu_read_ready = 1; 

    // 4. 清除标志位
    EXTI->PR = 1 << 0;
}	
//外部中断1服务程序
void EXTI1_IRQHandler(void)
{
	EXTI->PR=1<<1;  //清除LINE0上的中断标志位  
	u16EXIT0Count++;
	if(u16EXIT1Count>=50)
	{
		u16EXIT1Count = 0;
	}
	
}
//外部中断2服务程序
void EXTI2_IRQHandler(void)
{
	delay_ms(10);	//消抖
	if(KEY1==0)	  
	{	 

	}		 
	EXTI->PR=1<<2;  //清除LINE2上的中断标志位  
}
//外部中断3服务程序
void EXTI3_IRQHandler(void)
{
	delay_ms(10);	//消抖
	if(KEY0==0)	 
	{
		LED_RAS = !LED_RAS;
	}		 
	EXTI->PR=1<<3;  //清除LINE3上的中断标志位  
}
//外部中断4服务程序
void EXTI4_IRQHandler(void)
{
	delay_ms(10);	//消抖
	if(KEY1==0)	 
	{		
		LED_RAS = !LED_RAS;
	}		 
	EXTI->PR=1<<4;  //清除LINE4上的中断标志位  
}
	   
//外部中断初始化程序
void EXTIX_Init(void)
{
	//KEY_Init(); 

	RCC->AHB1ENR|=1<<1;     //使能PORTB时钟
	GPIO_Set(GPIOB,PIN0|PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_PU);	//PB0~1设置上拉输入
	
	Ex_NVIC_Config(GPIO_B,0,FTIR); 	//下降沿触发 PB0
	MY_NVIC_Init(0,0,EXTI0_IRQn,2);	//抢占2，子优先级3，组2
	
	Ex_NVIC_Config(GPIO_B,1,FTIR); 	//下降沿触发 PB1
	//MY_NVIC_Init(2,3,EXTI1_IRQn,2);	//抢占2，子优先级3，组2

	Ex_NVIC_Config(GPIO_E,3,FTIR); 	//下降沿触发 PE3
	MY_NVIC_Init(2,3,EXTI3_IRQn,2);	//抢占2，子优先级3，组2

	Ex_NVIC_Config(GPIO_E,4,FTIR); 	//下降沿触发 PE4
	MY_NVIC_Init(2,4,EXTI4_IRQn,2);	//抢占2，子优先级3，组2
}

