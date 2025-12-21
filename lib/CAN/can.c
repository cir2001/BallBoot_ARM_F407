#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
//---------------------------------------------------------------
// 变量定义
//--------------------------------------------------------------
u16 CAN_TX_Count = 0;
//////////////////////////////////////////////////////////////////////////////////	 
//					  
////////////////////////////////////////////////////////////////////////////////// 	   
//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:1~3;
//tbs2:时间段2的时间单元.范围:1~8;
//tbs1:时间段1的时间单元.范围:1~16;
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//注意以上参数任何一个都不能设为0,否则会乱.
//波特率=Fpclk1/((tbs1+tbs2+1)*brp);
//mode:0,普通模式;1,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(1,6,7,6,1);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败;
u8 CAN1_Mode_Init()
{
	u16 i=0;
	u8 brp;
	RCC->AHB1ENR|=1<<0;  	//使能PORTA口时钟 
	GPIO_Set(GPIOA,PIN11|PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA11,PA12,复用功能,上拉输出
 	GPIO_AF_Set(GPIOA,11,9);//PA11,AF9
	GPIO_AF_Set(GPIOA,12,9);//PA12,AF9 	 

 	/*  使能 CAN1 时钟 */
	RCC->APB1ENR|=1<<25;//使能CAN1时钟 CAN1使用的是APB1的时钟(max:42M)

	/*  退出 SLEEP 模式 (若处于睡眠状态) */
	CAN1->MCR=0x0000;	//退出睡眠模式(同时设置所有位为0)

	/*  进入初始化模式 */
	CAN1->MCR|=1<<0;		//请求CAN进入初始化模式
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100)return 2;//进入初始化模式失败
	}
	// --- 核心设置修改 ---
    CAN1->MCR &= ~(1<<4);   // NART=0: 允许自动重传 (关键！没人应答时会自动死命重发)
    CAN1->MCR |= 1<<6;      // ABOM=1: 开启自动离线管理 (离线后自动恢复)
    CAN1->MCR |= 1<<5;      // AWUM=1: 自动唤醒模式
    
    CAN1->MCR &= ~(1<<7);   // 非时间触发模式
    CAN1->MCR &= ~(1<<3);   // 报文不锁定
    CAN1->MCR &= ~(1<<2);   // 优先级由ID决定

	/*  设置波特率  */
	/* Note: this calculations fit for PCLK1 = 42MHz */
  	brp  = (42000000 / 21) / 500000;                   
	/* set BTR register so that sample point is at about 72% bit time from bit start */
	/* TSEG1 = 15, TSEG2 = 4, SJW = 2 => 1 CAN bit = 18 TQ, sample at 72%    */
  	CAN1->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((         0x0F) << 16) | (          0x1FF)); 
	//--- 波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv) ---//
	CAN1->BTR |=  ((((2-1) & 0x03) << 24) | (((5-1) & 0x07) << 20) | (((15-1) & 0x0F) << 16) | ((brp-1) & 0x1FF)); 
	/*  退出初始化模式 */						
	CAN1->MCR&=~(1<<0);		//请求CAN退出初始化模式
	while((CAN1->MSR&1<<0)==1)
	{
		i++;
		if(i>0XFFF0)return 3;//退出初始化模式失败
	}

	/*  配置过滤器0 (可配置的过滤器0-13) */
	CAN1->FMR|=1<<0;		//过滤器组工作在初始化模式
	CAN1->FA1R&=~(1<<0);	//过滤器0不激活
	CAN1->FS1R|=1<<0; 		//过滤器位宽为32位.
	CAN1->FM1R|=0<<0;		//过滤器0工作在标识符屏蔽位模式
	CAN1->FFA1R|=0<<0;		//过滤器0关联到FIFO0
	CAN1->sFilterRegister[0].FR1=0X00000000;//32位ID
	CAN1->sFilterRegister[0].FR2=0X00000000;//32位MASK
	CAN1->FA1R|=1<<0;		//激活过滤器0
	CAN1->FMR&=0<<0;		//过滤器组进入正常模式

 	//使用中断接收
	CAN1->IER|=1<<1;		//FIFO0消息挂号中断允许.	    
	MY_NVIC_Init(1,0,CAN1_RX0_IRQn,2);//组2

	return 0;
}   
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
    // 1. 判断 FIFO0 是否真的有数据 (FMP0 位不为0)
    // RF0R 的低2位表示 FIFO 中的消息数量
    if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) 
    {
        uint32_t rx_id;
        int32_t rx_AS5600_val;
        int32_t rx_state;
        uint32_t current_time;
        
        // 获取标准帧 ID
        // RIR 寄存器的 [31:21] 位是标准 ID (STID)
        // [2] 是 IDE (扩展帧标志), [1] 是 RTR (远程帧标志)
        // 我们只看标准 ID，所以右移 21 位
        rx_id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;

        // 检查是否为标准数据帧 (可选)
        if (((CAN1->sFIFOMailBox[0].RIR & CAN_RI0R_RTR) == 0) && 
            ((CAN1->sFIFOMailBox[0].RIR & CAN_RI0R_IDE) == 0))
        {
            // 记录当前系统时间戳
            current_time = Get_System_Tick();
            //读取数据
            rx_AS5600_val     = (int32_t)(CAN1->sFIFOMailBox[0].RDLR);
            rx_state = (int32_t)(CAN1->sFIFOMailBox[0].RDHR);

            // 5. 根据 ID 存入数组
            switch (rx_id)
            {
                case CAN_ID_RX_M1: // 收到 0x101 -> 存入 Motor[0]
                Motors[0].AS5600_val = rx_AS5600_val;
                Motors[0].status = rx_state;
                Motors[0].last_tick = current_time;
                break;
            
				case CAN_ID_RX_M2: // 收到 0x102 -> 存入 Motor[1]
					Motors[1].AS5600_val = rx_AS5600_val;
					Motors[1].status = rx_state;
					Motors[1].last_tick = current_time;
					break;
				
				case CAN_ID_RX_M3: // 收到 0x103 -> 存入 Motor[2]
					Motors[2].AS5600_val = rx_AS5600_val;
					Motors[2].status = rx_state;
					Motors[2].last_tick = current_time;
					break;
				
				default:
					break;
            }
        }

        // --- 释放邮箱 (至关重要) ---
        
        // 6. 释放 FIFO0 输出邮箱 (RFOM0 置 1)
        // 如果不执行这步，FIFO 满了之后就再也收不到新数据了
        CAN1->RF0R |= CAN_RF0R_RFOM0; 
    }
}
//--------------------------------------------------------------------------
// 功能: 向指定ID的电机发送数据
// 参数: 
//   motor_id: 目标电机的CAN ID
//   real_speed: 速度数据
//   pos_error: 位置偏差
// 返回值: 0=发送失败(邮箱全满), 1=发送成功(已放入邮箱)
//--------------------------------------------------------------------------
uint8_t CAN_Send_To_Motor(uint32_t motor_id, int32_t real_speed, int32_t pos_error)
{
    CAN_TX_Count++;
    if(CAN_TX_Count>=500)
    {
        CAN_TX_Count=0;
        LED_CAN = !LED_CAN;
    }
	 
    // 1. 查找空闲邮箱
    uint8_t mbox_idx = 0xFF;
    if (CAN1->TSR & CAN_TSR_TME0)      mbox_idx = 0;
    else if (CAN1->TSR & CAN_TSR_TME1) mbox_idx = 1;
    else if (CAN1->TSR & CAN_TSR_TME2) mbox_idx = 2;

    if (mbox_idx == 0xFF) 
    {
        // 只有在全满时才尝试一次性清理
        CAN1->TSR |= (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2);
        return 0; // 本次放弃，等下个周期
    }

    // 2. 写入数据 (确保在写入 TIR 之前先写 TDLR 和 TDHR)
    CAN1->sTxMailBox[mbox_idx].TDTR = 8; 
    CAN1->sTxMailBox[mbox_idx].TDLR = (uint32_t)real_speed;
    CAN1->sTxMailBox[mbox_idx].TDHR = (uint32_t)pos_error;

    // 3. 写入 ID 并触发发送请求 (TXRQ)
    // 这一步必须是最后一步，且直接赋值 ID + TXRQ 标志
    CAN1->sTxMailBox[mbox_idx].TIR = (motor_id << 21) | CAN_TI0R_TXRQ; 

    return 1;
}


