#include "sys.h"
#include "usart.h"
#include "led.h"	
#include <stdio.h>  
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	   
//
//********************************************************************************
//修改说明
//无
////////////////////////////////////////////////////////////////////////////////// 	
//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  0X08010000 	
u16 FalshSave[32];
//////////////////////////////////////////////////////////////////
u8  USART1_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
u8  USART1_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
u8  USART2_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
u8  USART2_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
u8  USART3_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
u8  USART3_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
u8  u8TxD1Busy,TxD1Num,TxD1pt,RxD1pt,RxD1Buf;
u8  u8TxD2Busy,TxD2Num,TxD2pt,RxD2pt,RxD2Buf;
u8  u8TxD3Busy,TxD3Num,TxD3pt,RxD3pt,RxD3Buf;
u8  u8ErrCode;
u8	UART1_RX_Flag;

u8  u8test1,u8test2,u8test3,u8test4,u8test5,u8test6,u8test7,u8test8;

u8  u8Uart2_flag,u8Uart2_flag_test;

int32_t recv_uart2_M1_val,recv_uart2_M2_val,recv_uart2_M3_val;

int32_t temp_val;
u8  rData1Temp,rData2Temp;
//=========================================================
//		 USART1 中断服务程序		  
//=========================================================
#if EN_USART1_RX   //如果使能了接收
//注意,读取USARTx->SR能避免莫名其妙的错误   	
void USART1_IRQHandler(void)
{
#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
	OSIntEnter();    
#endif
//========UART1 PE校验错误判断===============================
	if(USART1->SR&1)		//PE校验错误，软件清零，看说明书
	{			
		rData1Temp=USART1->DR;
		USART1->SR&=!1;	
	}				  
//========UART1 ORE过载错误判断===============================
	if(USART1->SR&(1<<3))
	{				
		rData1Temp=USART1->DR;	
		USART1->SR&=!(1<<3);				
	}	
//========UART1 FE帧错误判断===============================
	if(USART1->SR&(1<<1))		//FE帧错误，软件清零，看说明书
	{			
		USART1->SR&=!(1<<1);	
		rData1Temp=USART1->DR;		
	}			
//==== UART1 RxD ==========================================  	
	if(USART1->SR&(1<<5))//接收到数据
	{	 
		RxD1Buf=USART1->DR;
		USART1->SR&=!(1<<5);
		switch(RxD1Buf)	 {
			case '#':
				USART1_RX_BUF[0]='#';
				RxD1pt=1;
			break; 
			case '.':		// 连接PC机通讯，结束符<.>
				switch(USART1_RX_BUF[1])	{
			//==== A ====
					case 'A':	// 
							UART1ComReply();		// 指令回报
							break;
			//==== S ==== 
					case 'S':		// 
					
							break;
			//==== K ===== 
					case 'H':		//
							UART1_RX_Flag = 1;
							break;
			//==== G ===== 
					case 'G':		// 
							
							break;} 
			//=======defailt====
			default:
				USART1_RX_BUF[RxD1pt]=RxD1Buf;
				RxD1pt++;
				if(RxD1pt>=40) {RxD1pt=40; u8ErrCode=115;}	// 接收指令错误
			break;	} 		 									     
	}
//==== UART1 TxD ========================================== 
	if(USART1->SR&(1<<6))//Trasmission complete
	{
		if(TxD1pt < TxD1Num)
		{
		  USART1->DR = USART1_TX_BUF[TxD1pt];
		  TxD1pt ++;
		}else
		{
		  u8TxD1Busy = 0;		  //USART1发送结束标志
		  TxD1pt = TxD1Num;
		}
		USART1->SR&=!(1<<6);    //TC=0
	}  // */	 
#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
	OSIntExit();  											 
#endif
} 
#endif	
//=========================================================
// 		USART2 中断服务程序		
//=========================================================
void USART2_IRQHandler(void)
{
    u8 res;
    u32 sr = USART2->SR; // 这里的 SR 必须最先读取

    // 1. 强制错误处理：解决 ORE、FE、NE 锁死问题
    // 如果不清除这些标志，接收中断将永远停止触发
    if (sr & 0x0F) // 检查 ORE, NE, FE, PE 位
    {
        res = USART2->DR;     // 必须读 DR 才能配合 SR 的读取清除 ORE
        USART2->SR &= ~0x1F;  // 使用 ~ 符号正确清除所有状态位
        RxD2pt = 0;           // 发生硬件错误时重置指针，丢弃错误包
        return;
    }

    // 2. 正常数据接收
    if (sr & (1 << 5)) // RXNE: 接收缓冲区非空
    {
        res = USART2->DR;
        
        if (res == '#') // 帧起始符：强制重置，保证同步
        {
            RxD2pt = 0;
            USART2_RX_BUF[RxD2pt++] = res;
        }
        else if (res == '.') // 帧结束符
        {
            if (RxD2pt < USART_TRANS_LEN)
            {
                USART2_RX_BUF[RxD2pt] = '.'; // 保存结束符
                u8Uart2_flag = 1;            // 通知 main 循环解析处理
            }
        }
        else // 存入缓冲区
        {
            if (RxD2pt < USART_TRANS_LEN - 1)
            {
                USART2_RX_BUF[RxD2pt++] = res;
            }
            else // 缓冲区溢出保护
            {
                RxD2pt = 0;
            }
        }
    }

    // 3. 发送完成处理：修正 &= ~ 逻辑
    if (sr & (1 << 6)) // TC: 发送完成
    {
        if (TxD2pt < TxD2Num)
        {
            USART2->DR = USART2_TX_BUF[TxD2pt++];
        }
        else
        {
            u8TxD2Busy = 0;
        }
        USART2->SR &= ~(1 << 6); // 正确清除 TC 位
    }
}
//=========================================================
// 		USART3 中断服务程序		
//=========================================================
#if EN_USART3_RX   //如果使能了接收
void USART3_IRQHandler(void)
{
#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
	OSIntEnter();    
#endif
//==== UART3 RxD ==========================================  	
	if(USART3->SR&(1<<5))//接收到数据
	{	 
		RxD3Buf=USART3->DR;
		USART3->SR&=!(1<<5);
		switch(RxD3Buf)	 {
			case '#':
				USART3_RX_BUF[0]='#';
				RxD3pt=1;
			break; 
			case '.':		// 连接PC机通讯，结束符<.>
				switch(USART3_RX_BUF[1])	{
			//==== A ====
					case 'A':	// 
							UART3ComReply();		// 指令回报
							break;
			//==== S ==== 
					case 'S':		// 
					
							break;
			//==== K ===== 
					case 'K':		//
						
							break;
			//==== G ===== 
					case 'G':		// 
							
							break;} 
			//=======defailt====
			default:
				USART3_RX_BUF[RxD3pt]=RxD3Buf;
				RxD3pt++;
				if(RxD3pt>=40) {RxD3pt=40; u8ErrCode=115;}	// 接收指令错误
			break;	} 		 									     
	}
//==== UART3 TxD ========================================== 
	if(USART3->SR&(1<<6))//Trasmission complete
	{
		if(TxD3pt < TxD3Num)
		{
		  USART3->DR = USART3_TX_BUF[TxD3pt];
		  TxD3pt ++;
		}else
		{
		  u8TxD3Busy = 0;		  //USART1发送结束标志
		  TxD3pt = TxD3Num;
		}
		USART3->SR&=!(1<<6);    //TC=0
	}  // */	 
#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
	OSIntExit();  											 
#endif
} 
#endif	
//*************************************************************************************//
// 初始化IO 串口1 (DMA模式专用配置)
// pclk2: PCLK2时钟频率(84Mhz)
// bound: 波特率 
//*************************************************************************************//
void uart_init1(u32 pclk2, u32 bound)
{    
    float temp;
    u16 mantissa;
    u16 fraction;      
    
    // --- 1. 波特率计算 ---
    temp = (float)(pclk2 * 1000000) / (bound * 16);
    mantissa = temp;             
    fraction = (temp - mantissa) * 16; 
    mantissa <<= 4;
    mantissa += fraction; 

    // --- 2. 时钟与GPIO配置  ---
    RCC->AHB1ENR |= 1 << 0;     // 使能PORTA口时钟  
    RCC->APB2ENR |= 1 << 4;     // 使能串口1时钟 
    
    GPIO_Set(GPIOA, PIN9 | PIN10, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_50M, GPIO_PUPD_PU);
    GPIO_AF_Set(GPIOA, 9, 7);   // PA9 -> AF7 (USART1_TX)
    GPIO_AF_Set(GPIOA, 10, 7);  // PA10 -> AF7 (USART1_RX)
    
    // --- 3. 寄存器配置  ---
    USART1->BRR = mantissa;     // 设置波特率    
    USART1->CR1 &= ~(1 << 15);  // 设置OVER8=0 

    // 配置 CR3 寄存器，使能 DMA 请求
    // Bit 7: DMAT (DMA Enable Transmitter)
    // Bit 6: DMAR (DMA Enable Receiver)
    USART1->CR3 |= (1 << 7) | (1 << 6); 

#if EN_USART1_RX  // 如果使能了接收
    
    // 配置 CR1 寄存器，中断策略变更
    // Bit 3: TE (发送使能)
    // Bit 2: RE (接收使能)
    // Bit 4: IDLEIE (空闲中断使能) -> 关键！只在空闲时中断
    USART1->CR1 |= (1 << 3) | (1 << 2) | (1 << 4);

    // 移除旧的中断
    // 删除了 RXNEIE (Bit 5): 禁止接收缓冲区非空中断 (DMA会搬运，不需要CPU管)
    // 删除了 TCIE (Bit 6) / TXEIE (Bit 7): 禁止发送完成中断 (在主循环通过DMA寄存器判断)
    USART1->CR1 &= ~((1 << 5) | (1 << 6) | (1 << 7)); 

    // NVIC 配置 (优先级配置)
    // 这里的优先级建议设为 2 (高于主循环，但低于 5ms 定时器)
    MY_NVIC_Init(2, 2, USART1_IRQn, 2); 
    
#endif

    // --- 4. 使能串口 ---
    USART1->CR1 |= 1 << 13;     // UE (USART Enable)
}
/*************************************************************** */
//初始化IO 串口2
//pclk1:PCLK1时钟频率(42Mhz)
//bound:波特率 
/*************************************************************** */
void uart_init2(u32 pclk1,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk1*1000000)/(bound*16);//得到USARTDIV@OVER8=0
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<0;   	//使能PORTA口时钟  
	RCC->APB1ENR|=1<<17;  	//使能串口2时钟 
	GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA2,PA3,复用功能,上拉输出
 	GPIO_AF_Set(GPIOA,2,7);	//PA2,AF7
	GPIO_AF_Set(GPIOA,3,7);	//PA3,AF7  	   
	//波特率设置
 	USART2->BRR=mantissa; 	//波特率设置	 
	USART2->CR1&=~(1<<15); 	//设置OVER8=0 
	USART2->CR1|=1<<3;  	//串口发送使能 
#if EN_USART2_RX		  	//如果使能了接收
	//使能接收中断 
	USART2->CR1|=1<<2;  	//串口接收使能
	USART2->CR1|=1<<5;    	//接收缓冲区非空中断使
	USART2->CR1|=1<<6;    //发送缓冲区非空中断使能	    	
	MY_NVIC_Init(1,0,USART2_IRQn,2);//组2，最低优先级 
#endif
	USART2->CR1|=1<<13;  	//串口使能
}
//初始化IO 串口3
//pclk1:PCLK1时钟频率(42Mhz)
//bound:波特率 
void uart_init3(u32 pclk1,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk1*1000000)/(bound*16);//得到USARTDIV@OVER8=0
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<3;   	//使能PORTD口时钟  
	RCC->APB1ENR|=1<<18;  	//使能串口3时钟 
	GPIO_Set(GPIOD,PIN8|PIN9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PD8,PD9,复用功能,上拉输出
 	GPIO_AF_Set(GPIOD,8,7);	//PD8,AF7
	GPIO_AF_Set(GPIOD,9,7);	//PD9,AF7  	   
	//波特率设置
 	USART3->BRR=mantissa; 	//波特率设置	 
	USART3->CR1&=~(1<<15); 	//设置OVER8=0 
	USART3->CR1|=1<<3;  	//串口发送使能 
#if EN_USART3_RX		  	//如果使能了接收
	//使能接收中断 
	USART3->CR1|=1<<2;  	//串口接收使能
	USART3->CR1|=1<<5;    	//接收缓冲区非空中断使
	USART3->CR1|=1<<6;    //发送缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART3_IRQn,2);//组2，最低优先级 
#endif
	USART3->CR1|=1<<13;  	//串口使能
}
//=========================================================
// UART1 指令回报
//
//				
//=========================================================
void UART1ComReply(void)
{
	USART1_TX_BUF[1]='A';
	USART1_TX_BUF[2]=0x31;	
	USART1_TX_BUF[3]=0x32;
	
	USART1_TX_BUF[4]=0x0d;
	USART1_TX_BUF[5]=0x0a;	

	TxD1pt = 1;
	TxD1Num= 6;
	USART1->DR = '&';
}
//=========================================================
// UART2 指令回报
//
//				
//=========================================================
void UART2ComReply(void)
{
	if(u8TxD2Busy) return; // 如果正在发送，则退出，避免冲突

	USART2_TX_BUF[1]='A';
	USART2_TX_BUF[2]=0x35;	
	USART2_TX_BUF[3]=0x36;
	
	USART2_TX_BUF[4]=0x0d;
	USART2_TX_BUF[5]=0x0a;	

	TxD2pt = 1;
	TxD2Num= 6;
	u8TxD2Busy = 1;  // 置忙碌标志
	USART2->DR = '&';
}
//=========================================================
// UART3 指令回报
//
//				
//=========================================================
void UART3ComReply(void)
{
	USART3_TX_BUF[1]='A';
	USART3_TX_BUF[2]=0x37;	
	USART3_TX_BUF[3]=0x38;
	
	USART3_TX_BUF[4]=0x0d;
	USART3_TX_BUF[5]=0x0a;	

	TxD3pt = 1;
	TxD3Num= 6;
	USART3->DR = '&';
}


