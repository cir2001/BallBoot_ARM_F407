#ifndef __CONTROL_H
#define __CONTROL_H		
#include "sys.h"	 
//////////////////////////////////////////////////////////////////////////////////	
/****************************************************************************************************

**************************************************************************************************/	
void EXTI0_IRQHandler(void);

void Forward_Kinematics(float a,float b,float c);
void Inverse_Kinematics(float a,float b,float c);
void Control_Calculate(void);

	    															  					 
#endif  
	 


