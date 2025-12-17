#ifndef __LED_H
#define __LED_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//						  
////////////////////////////////////////////////////////////////////////////////// 	
//---开发板LED端口定义---
//#define LED0 PAout(6)	// DS0
//#define LED1 PAout(7)	// DS1	 

//---PCB LED端口定义---
#define LED_MC PEout(2)    //PE2   LED_MC
#define LED_RAS PEout(8)   //PE8   LED_RAS
#define LED_MPU PEout(15)  //PE15  LED_MPU
#define LED_OLED PDout(11) //PD11  LED_OLED
#define LED_U3 PDout(12)   //PD12  LED_U3   
#define LED_U1 PDout(13)   //PD13  LED_U1
#define LED_MW PDout(14)   //PD14  LED_MW
#define LED_CAN PDout(15)  //PD15  LED_CAN
#define LED_U2 PCout(4)   //PC4   LED_U2
#define LED_MA PCout(5)   //PC5   LED_MA

//---Motor 方向控制端口定义---
#define MA_AIN1 PCout(11)	// 
#define MA_AIN2 PCout(10)	// 
#define MB_BIN1 PCout(12)	// 
#define MB_BIN2 PDout(0)	// 
#define MC_AIN1 PBout(9)	//
#define MC_AIN2 PBout(8)	// 	
#define MD_BIN1 PEout(0)	//
#define MD_BIN2 PEout(1)	//	


void LED_Init(void);//初始化		 				    
#endif


