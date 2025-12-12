#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"	  
//////////////////////////////////////////////////////////////////////////////////	   
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//串口1初始化 
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_TRANS_LEN     	200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define EN_USART2_RX            1       //使能串口2接收
#define EN_USART3_RX            1       //使能串口2接收
	  	

void uart_init1(u32 pclk2,u32 bound); 
void uart_init2(u32 pclk1,u32 bound); 
void uart_init3(u32 pclk1,u32 bound); 

void UART1ComReply(void);
void UART2ComReply(void);
void UART3ComReply(void);
#endif	   

