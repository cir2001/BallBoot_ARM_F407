#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//
//
//
//********************************************************************************

////////////////////////////////////////////////////////////////////////////////// 	
#define ENCODER_TIM2_PERIOD  0xFFFFFFFF		//1322
#define ENCODER_TIM3_PERIOD  0xFFFF	//1322
#define ENCODER_TIM4_PERIOD  0xFFFF    //1043
#define ENCODER_TIM5_PERIOD  0xFFFF    //1043

#define COUNTER_RESET 0			//0
void TIM1_Int_Init(u16 arr,u16 psc);
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM3_Int_Init(u16 arr,u16 psc);

void TIM8_PWM_Init(u16 arr,u16 psc);

void TIM2_Encoder_Init(void);
void TIM3_Encoder_Init(void);
void TIM4_Encoder_Init(void);
void TIM5_Encoder_Init(void);

#endif
