#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "exti.h"
//---------------------------------------------------------------
// 变量定义
//--------------------------------------------------------------
//外部变量
extern volatile uint8_t write_index;       // 当前正在写入的 Ping-Pong 缓冲区索引
extern volatile uint8_t sample_in_buf_cnt; // 1ms 计数器 (0-19)
extern HybridPacket_t PingPongBuffer[2];   // 双缓冲区

//-------------
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
	MY_NVIC_Init(0,1,CAN1_RX0_IRQn,2);//组2

	return 0;
}   
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
    // 1. 直接读取寄存器快照，减少外设总线访问次数
    // 硬件寄存器访问比内存慢，读一次存入局部变量是最高效的
    uint32_t rir = CAN1->sFIFOMailBox[0].RIR;
    
    // 2. 快速检查：必须是标准帧且非远程帧
    // 使用位掩码一次性判断多个条件 (IDE=0 且 RTR=0)
    if ((rir & (CAN_RI0R_IDE | CAN_RI0R_RTR)) == 0)
    {
        uint32_t rx_id = (rir >> 21); // 提取 ID
        uint32_t m_idx = rx_id - CAN_ID_RX_M1; // 偏移计算索引

        // 3. 边界检查：只处理 M1, M2, M3
        if (m_idx < 3) 
        {
            // 缓存 1ms 计数器快照，防止读取过程中 sample_in_buf_cnt 被 1ms 中断修改
            uint32_t current_cnt = sample_in_buf_cnt;
            uint32_t slot = current_cnt / 5;
            if (slot > 3) slot = 3;

            // 4. 一次性读取数据寄存器
            uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR; // 编码器值
            uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR; // 状态值

            // 5. 指针偏移优化：直接定位到目标内存地址
            // 这种写法能让编译器生成最简练的汇编代码
            volatile Motor_Feedback_t *m_local = &Motors[m_idx];
            MotorFeedback_t *m_tele = &PingPongBuffer[write_index].motor_data[slot][m_idx];

            // 更新本地控制变量
            m_local->AS5600_val = (int32_t)rdlr;
            m_local->status     = (uint16_t)rdhr;
            m_local->last_tick  = Get_System_Tick(); // 使用全局毫秒计数器

            // 更新上位机上传变量 (使用精确的 float 常量)
            // 360/4096 = 0.087890625f
            m_tele->angle  = (float)((int32_t)rdlr) * 0.087890625f;
            m_tele->status = (uint16_t)rdhr;
        }
    }
        // --- 释放邮箱 (至关重要) ---
        
        // 6. 释放 FIFO0 输出邮箱 (RFOM0 置 1)
        // 如果不执行这步，FIFO 满了之后就再也收不到新数据了
        CAN1->RF0R |= CAN_RF0R_RFOM0; 
}
//--------------------------------------------------------------------------
// 功能: 向指定ID的电机发送数据
// 参数: 
//   motor_id: 目标电机的CAN ID
//   real_speed: 速度数据
//   pos_error: 位置偏差
// 返回值: 0=发送失败(邮箱全满), 1=发送成功(已放入邮箱)
//--------------------------------------------------------------------------
uint8_t CAN_Send_To_Motor(uint32_t motor_id, int real_speed, int pos_error)
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


