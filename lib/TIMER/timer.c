#include "timer.h"
#include "led.h"
#include "usart.h" 
#include "mpu6500_driver.h"
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
extern u16 oled_tick;
//-----------------------------------------------
// 变量声明
//-----------------------------------------------
volatile int32_t Target_Speed_M1 = 100; 
volatile int32_t Target_Speed_M2 = 0;
volatile int32_t Target_Speed_M3 = 0;

volatile uint32_t g_system_tick = 0; // 全局毫秒计数

int16_t cnt;
int16_t iMotorA_Encoder,iMotorB_Encoder,iMotorC_Encoder;
long int iMotorAPulseTotle,iMotorBPulseTotle,iMotorCPulseTotle;

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
		TIM8->CCR1 = 16500;	//Motor A PWM Out	
		TIM8->CCR2 = 10500;	//Motor A PWM Out	
		TIM8->CCR3 = 8500;	//Motor A PWM Out
		TIM8->CCR4 = 15500;	//Motor A PWM Out
		cnt = TIM2->CNT;
		iMotorA_Encoder = cnt;
		iMotorAPulseTotle += iMotorA_Encoder;
		if(iMotorAPulseTotle>=5000)
		{
			TIM8->CCR1 = 0;	//Motor A PWM Out
			TIM8->CCR2 = 0;	//Motor A PWM Out
			TIM8->CCR3 = 0;	//Motor A PWM Out
			TIM8->CCR4 = 0;	//Motor A PWM Out
		}
		TIM2->CNT = 0;	
		iData2ASCII(iMotorAPulseTotle);		    				   				     	    	
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
  	MY_NVIC_Init(1,2,TIM2_IRQn,2);	//抢占1，子优先级32，组2									 
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
//===============================
//通用定时器2 Encoder初始化
//这里时钟选择为APB1 = 84M
//===============================
void TIM2_Encoder_Init(void)
{
	/************ 1. 开启时钟 ************/
	RCC->AHB1ENR |= 1<<0;   	//GPIOA时钟
	RCC->AHB1ENR |= 1<<1;   	//GPIOB时钟
	RCC->APB1ENR |= 1<<0;		//使能TIM2时钟

	/************ 2. GPIOA0/A1 配置为复用功能 AF1 ************/
	GPIO_Set(GPIOA,PIN15,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA15 编码器B相,复用功能,上拉
	GPIO_Set(GPIOB,PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB3 编码器A相,复用功能,上拉
	GPIO_AF_Set(GPIOA,15,1);		//PA15,AF1
	GPIO_AF_Set(GPIOB,3,1);			//PB3,AF1	

	/************ 3. TIM2 基本配置 ************/
	TIM2->PSC = 0x0;				//预分频器0为不分频
	TIM2->ARR = ENCODER_TIM2_PERIOD-1;//设定计数器自动重装值
	
	TIM2->CR1 &= ~(3<<8);			//选择时钟分频-不分频
	TIM2->CR1 &= ~(3<<5);			//选择计数模式 边沿对齐
	
	/************ 4. 输入捕获配置 ************/
	TIM2->CCMR1 |= 1<<0;		  	//CC1S='01' IC1FP1映射到TI1
	TIM2->CCMR1 |= 1<<8; 			//CC2S='01' IC2FP2映射到TI2

	TIM2->CCER &= ~(1<<1);  		//CC1P='0'  IC1FP1不反相,IC1FP1=TI1 上升沿
	TIM2->CCER &= ~(1<<5);  		//CC2P='0'  IC2FP2不反相,IC2FP2=TI2 上升沿

	//TIM3->CCER |= (1<<0);       	// CC1E = 1，使能通道1 
	//TIM3->CCER |= (1<<4);       	// CC2E = 1，使能通道2 

	TIM2->CCMR1 |= 3<<4; 			// IC1F='1000' 输入捕获1滤波器
	TIM2->CCMR1 |= 3<<12;    		// IC2F 输入捕获1滤波器

	TIM2->SMCR |= 3<<0;  			//SMS='011' 编码器模式 TI1+TI2 双边沿计数

	/************ 5. 清零并启动 ************/
	TIM2->CNT = COUNTER_RESET;
	TIM2->CR1 |= 0x01;   			//CEN=1,使能定时器

	/********** 6.Enable the TIM2 Update Interrupt *********/
	/*TIM3->DIER |= 1<<0;				//允许更新中断
	TIM3->DIER |= 1<<6;				//允许触发中断
	MY_NVIC_Init(1,3,TIM3_IRQn,2);//抢占1，子优先级1，组2*/
}

//===============================
//通用定时器3 Encoder初始化
//这里时钟选择为APB1 = 84M
//===============================
void TIM3_Encoder_Init(void)
{
	/************ 1. 开启时钟 ************/
	RCC->AHB1ENR |= 1<<1;   	//GPIOB时钟
	RCC->APB1ENR |= 1<<1;		//使能TIM3时钟

	/************ 2. GPIOA0/A1 配置为复用功能 AF1 ************/
	GPIO_Set(GPIOB,PIN4|PIN5,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB4 编码器B相,PB5 编码器A相,复用功能,上拉
	GPIO_AF_Set(GPIOB,4,2);		//PB4,AF2
	GPIO_AF_Set(GPIOB,5,2);		//PB5,AF2

	/************ 3. TIM2 基本配置 ************/
	TIM3->PSC = 0x0;				//预分频器0为不分频
	TIM3->ARR = ENCODER_TIM3_PERIOD-1;//设定计数器自动重装值
	
	TIM3->CR1 &= ~(3<<8);			//选择时钟分频-不分频
	TIM3->CR1 &= ~(3<<5);			//选择计数模式 边沿对齐
	
	/************ 4. 输入捕获配置 ************/
	TIM3->CCMR1 |= 1<<0;		  	//CC1S='01' IC1FP1映射到TI1
	TIM3->CCMR1 |= 1<<8; 			//CC2S='01' IC2FP2映射到TI2

	TIM3->CCER &= ~(1<<1);  		//CC1P='0'  IC1FP1不反相,IC1FP1=TI1 上升沿
	TIM3->CCER &= ~(1<<5);  		//CC2P='0'  IC2FP2不反相,IC2FP2=TI2 上升沿

	//TIM3->CCER |= (1<<0);       	// CC1E = 1，使能通道1 
	//TIM3->CCER |= (1<<4);       	// CC2E = 1，使能通道2 

	TIM3->CCMR1 |= 3<<4; 			// IC1F='1000' 输入捕获1滤波器
	TIM3->CCMR1 |= 3<<12;    		// IC2F 输入捕获1滤波器

	TIM3->SMCR |= 3<<0;  			//SMS='011' 编码器模式 TI1+TI2 双边沿计数

	/************ 5. 清零并启动 ************/
	TIM3->CNT = COUNTER_RESET;
	TIM3->CR1 |= 0x01;   			//CEN=1,使能定时器

	/********** 6.Enable the TIM2 Update Interrupt *********/
	/*TIM3->DIER |= 1<<0;				//允许更新中断
	TIM3->DIER |= 1<<6;				//允许触发中断
	MY_NVIC_Init(1,3,TIM3_IRQn,2);//抢占1，子优先级1，组2*/
}
//===============================
//通用定时器4 Encoder初始化
//这里时钟选择为APB1 = 84M
//===============================
void TIM4_Encoder_Init(void)
{
	/************ 1. 开启时钟 ************/
	RCC->AHB1ENR |= 1<<1;   	//GPIOB时钟
	RCC->APB1ENR |= 1<<2;		//使能TIM4时钟

	/************ 2. GPIOA0/A1 配置为复用功能 AF1 ************/
	GPIO_Set(GPIOB,PIN6|PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB6 编码器B相,PB7 编码器A相,复用功能,上拉
	GPIO_AF_Set(GPIOB,6,2);		//PB6,AF2
	GPIO_AF_Set(GPIOB,7,2);		//PB7,AF2	

	/************ 3. TIM2 基本配置 ************/
	TIM4->PSC = 0x0;				//预分频器0为不分频
	TIM4->ARR = ENCODER_TIM4_PERIOD-1;//设定计数器自动重装值
	
	TIM4->CR1 &= ~(3<<8);			//选择时钟分频-不分频
	TIM4->CR1 &= ~(3<<5);			//选择计数模式 边沿对齐
	
	/************ 4. 输入捕获配置 ************/
	TIM4->CCMR1 |= 1<<0;		  	//CC1S='01' IC1FP1映射到TI1
	TIM4->CCMR1 |= 1<<8; 			//CC2S='01' IC2FP2映射到TI2

	TIM4->CCER &= ~(1<<1);  		//CC1P='0'  IC1FP1不反相,IC1FP1=TI1 上升沿
	TIM4->CCER &= ~(1<<5);  		//CC2P='0'  IC2FP2不反相,IC2FP2=TI2 上升沿

	//TIM3->CCER |= (1<<0);       	// CC1E = 1，使能通道1 
	//TIM3->CCER |= (1<<4);       	// CC2E = 1，使能通道2 

	TIM4->CCMR1 |= 3<<4; 			// IC1F='1000' 输入捕获1滤波器
	TIM4->CCMR1 |= 3<<12;    		// IC2F 输入捕获1滤波器

	TIM4->SMCR |= 3<<0;  			//SMS='011' 编码器模式 TI1+TI2 双边沿计数

	/************ 5. 清零并启动 ************/
	TIM4->CNT = COUNTER_RESET;
	TIM4->CR1 |= 0x01;   			//CEN=1,使能定时器

	/********** 6.Enable the TIM2 Update Interrupt *********/
	/*TIM4->DIER |= 1<<0;				//允许更新中断
	TIM4->DIER |= 1<<6;				//允许触发中断
	MY_NVIC_Init(1,3,TIM4_IRQn,2);//抢占1，子优先级1，组2*/
}
//===============================
//通用定时器5 Encoder初始化
//这里时钟选择为APB1 = 84M
//===============================
void TIM5_Encoder_Init(void)
{
	/************ 1. 开启时钟 ************/
	RCC->AHB1ENR |= 1<<0;   	//GPIOA时钟
	RCC->APB1ENR |= 1<<3;		//使能TIM3时钟

	/************ 2. GPIOA0/A1 配置为复用功能 AF1 ************/
	GPIO_Set(GPIOA,PIN0|PIN1,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA0 编码器B相,PA1 编码器A相,复用功能,上拉
	GPIO_AF_Set(GPIOA,0,2);		//PA0,AF2
	GPIO_AF_Set(GPIOA,1,2);		//PA1,AF2	

	/************ 3. TIM2 基本配置 ************/
	TIM5->PSC = 0x0;				//预分频器0为不分频
	TIM5->ARR = ENCODER_TIM5_PERIOD-1;//设定计数器自动重装值
	
	TIM5->CR1 &= ~(3<<8);			//选择时钟分频-不分频
	TIM5->CR1 &= ~(3<<5);			//选择计数模式 边沿对齐
	
	/************ 4. 输入捕获配置 ************/
	TIM5->CCMR1 |= 1<<0;		  	//CC1S='01' IC1FP1映射到TI1
	TIM5->CCMR1 |= 1<<8; 			//CC2S='01' IC2FP2映射到TI2

	TIM5->CCER &= ~(1<<1);  		//CC1P='0'  IC1FP1不反相,IC1FP1=TI1 上升沿
	TIM5->CCER &= ~(1<<5);  		//CC2P='0'  IC2FP2不反相,IC2FP2=TI2 上升沿

	//TIM3->CCER |= (1<<0);       	// CC1E = 1，使能通道1 
	//TIM3->CCER |= (1<<4);       	// CC2E = 1，使能通道2 

	TIM5->CCMR1 |= 3<<4; 			// IC1F='1000' 输入捕获1滤波器
	TIM5->CCMR1 |= 3<<12;    		// IC2F 输入捕获1滤波器

	TIM5->SMCR |= 3<<0;  			//SMS='011' 编码器模式 TI1+TI2 双边沿计数

	/************ 5. 清零并启动 ************/
	TIM5->CNT = COUNTER_RESET;
	TIM5->CR1 |= 0x01;   			//CEN=1,使能定时器

	/********** 6.Enable the TIM2 Update Interrupt *********/
	/*TIM5->DIER |= 1<<0;				//允许更新中断
	TIM5->DIER |= 1<<6;				//允许触发中断
	MY_NVIC_Init(1,3,TIM5_IRQn,2);//抢占1，子优先级1，组2*/
}
//-----------------------------------------------
// 获取当前时间戳
//-----------------------------------------------
uint32_t Get_System_Tick(void)
{
    return g_system_tick;
}

