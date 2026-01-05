#include "timer.h"
#include "led.h"
#include "usart.h" 
#include "can.h"
//////////////////////////////////////////////////////////////////////////////////	 
//
////////////////////////////////////////////////////////////////////////////////// 	
//-----------------------------------------------
// 外部函数声明
//-----------------------------------------------

//-----------------------------------------------
//外部变量声明
//-----------------------------------------------
extern u8 oled_tick;
extern u8 magic_tick;
//-----------------------------------------------
// 变量声明
//-----------------------------------------------
volatile int Target_Speed_M1 = 0; 
volatile int Target_Speed_M2 = 0;
volatile int Target_Speed_M3 = 0;

volatile uint32_t g_system_tick = 0; // 全局毫秒计数

u8 timer2_Counter;
//==============================================
// TIM1 更新中断服务函数
//==============================================
void TIM1_UP_TIM10_IRQHandler(void)
{
    if(TIM1->SR&0X0001) // 检查更新中断标志位
    {
		//LED_U1=!LED_U1;
        // 在这里执行您的代码
    }
	TIM1->SR&=~(1<<0); // 清除更新中断标志位
}
//==============================================
//定时器2中断服务程序
//==============================================	 
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//溢出中断
	{
		g_system_tick++;       // 5毫秒累加
		timer2_Counter++;
		oled_tick++;
		magic_tick++;
		if(timer2_Counter>=20) // 100ms
		{
			timer2_Counter=0;
			LED_RAS = !LED_RAS;
			
		}
		// 发给电机1 (ID: 0x201)
		CAN_Send_To_Motor(CAN_ID_TX_M1, Target_Speed_M1, 0);
		
		// 发给电机2 (ID: 0x301)
		CAN_Send_To_Motor(CAN_ID_TX_M2, Target_Speed_M2, 0);
		
		// 发给电机3 (ID: 0x401)
		CAN_Send_To_Motor(CAN_ID_TX_M3, Target_Speed_M3, 0);	

		TIM2->SR&=~(1<<0);//清除中断标志位 	 	   				     	    	
	}				      
}
//==============================================
//定时器3中断服务程序
//==============================================	 
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{
				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
//==============================================
//高级定时器TIM1初始化设置
//这里时钟选择为APB2 = 84MHz  APB2分频 = 2 
//TIM1时钟翻倍，fTIM1 = 168MHz
//fPWM = fTIM/(PSC+1)(ARR+1)
//arr：自动重装值。
//psc：时钟预分频数
//==============================================
void TIM1_Int_Init(u16 arr,u16 psc)
{
	RCC->APB2ENR |= 1 << 0;			//使能TIM1时钟

	TIM1->ARR=arr;          //设定计数器自动重装值
    TIM1->PSC=psc;          //预分频器
    TIM1->DIER|=1<<0;       //允许更新中断
    TIM1->CR1|=0x01;        //使能定时器1
    
    //STM32F407中，TIM1的更新中断与TIM10共享中断向量
    MY_NVIC_Init(1,2,TIM1_UP_TIM10_IRQn,2); //抢占1，子优先级2，组2
}
//==============================================
//通用定时器2中断初始化
//这里时钟选择为APB1的2倍，而APB1为42M
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
void TIM2_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<0;	//TIM3时钟使能    
 	TIM2->ARR=arr;  	//设定计数器自动重装值 
	TIM2->PSC=psc;  	//预分频器	  
	TIM2->DIER|=1<<0;   //允许更新中断	  
	TIM2->CR1|=0x01;    //使能定时器2
  	MY_NVIC_Init(2,0,TIM2_IRQn,2);	//抢占1，子优先级32，组2									 
}
//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为42M
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
 	TIM3->ARR=arr;  	//设定计数器自动重装值 
	TIM3->PSC=psc;  	//预分频器	  
	TIM3->DIER|=1<<0;   //允许更新中断	  
	TIM3->CR1|=0x01;    //使能定时器2
  	MY_NVIC_Init(1,3,TIM3_IRQn,2);	//抢占1，子优先级3，组2									 
}
//==============================================
//高级定时器TIM8初始化设置 PWM输出模式
//这里时钟选择为APB2 = 84MHz  APB2分频 = 2 
//TIM1时钟翻倍，fTIM1 = 168MHz
//fPWM = fTIM/(PSC+1)(ARR+1)
//arr：自动重装值。
//psc：时钟预分频数
//==============================================
void TIM8_PWM_Init(u16 arr,u16 psc)
{
	RCC->AHB1ENR |= 1 << 2;   		//使能PORTC时钟	
	RCC->APB2ENR |= 1 << 1;			//使能TIM8时钟
	GPIO_Set(GPIOC,PIN6|PIN7|PIN8|PIN9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//复用功能,上拉
	GPIO_AF_Set(GPIOC,6,3);		//PC6,AF3
	GPIO_AF_Set(GPIOC,7,3);		//PC7,AF3
	GPIO_AF_Set(GPIOC,8,3);		//PC8,AF3
	GPIO_AF_Set(GPIOC,9,3);		//PC9,AF3

	//设定计数器自动重装值及是否分频
	TIM8->ARR = arr;			//设定计数器自动重装值-决定PWM的频率
	TIM8->PSC = psc;			//预分频器0为不分频
	//ch1输出使能设置
	TIM8->CCMR1 &= ~(7 << 4);	//复位输出比较1模式
	TIM8->CCMR1 |= 6 << 4;		//CH1  PWM1模式
	TIM8->CCMR1 |= 1 << 3;		//CH1预装载使能
	//ch1输出使能设置
	TIM8->CCER  |= 1 << 0;		//输入/捕获1输出使能
	//-----------------
	TIM8->CCMR1 |= 6 << 12;//CH2  PWM1模式
	TIM8->CCMR1 |= 1 << 11;//CH2预装载使能
	//ch2输出使能设置
	TIM8->CCER  |= 1 << 4;//输入/捕获2输出使能
	//-----------------
	TIM8->CCMR2 |= 6 << 4;//CH3  PWM1模式
	TIM8->CCMR2 |= 1 << 3;//CH3预装载使能
	//ch3输出使能设置
	TIM8->CCER  |= 1 << 8;//输入/捕获3输出使能
	//ch4输出使能设置
	TIM8->CCMR2 |= 6 << 12;//CH4  PWM2模式
	TIM8->CCMR2 |= 1 << 11;//CH4预装载使能
	//ch4输出使能设置
	TIM8->CCER  |= 1 << 12;//输入/捕获4输出使能

    TIM8->BDTR |= (1 << 15);  // MOE = 1  使能主输出（高级定时器必须）

	//自动重装载预装载允许位ARPE及定时器使能
	TIM8->CR1 = 0X0080;	//ARPE使能
	TIM8->CR1 |= 0X01;	//使能定时器1
}

//-----------------------------------------------
// 获取当前时间戳
//-----------------------------------------------
uint32_t Get_System_Tick(void)
{
    return g_system_tick;
}

