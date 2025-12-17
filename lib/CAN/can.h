#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//					  
////////////////////////////////////////////////////////////////////////////////// 	 
// -----------------------------------------------------------
// 1. F407 发送给电机的控制指令 ID (Tx)
//    方向: F407 -> 电机
// -----------------------------------------------------------
#define CAN_ID_TX_M1   0x201  // 发送给电机1
#define CAN_ID_TX_M2   0x301  // 发送给电机2
#define CAN_ID_TX_M3   0x401  // 发送给电机3

// -----------------------------------------------------------
// 2. 电机发送回 F407 的反馈数据 ID (Rx)
//    方向: 电机 -> F407
//    注意：F407 的接收过滤器必须能通过这些 ID
// -----------------------------------------------------------
#define CAN_ID_RX_M1   0x101  // 来自电机1的反馈
#define CAN_ID_RX_M2   0x102  // 来自电机2的反馈
#define CAN_ID_RX_M3   0x103  // 来自电机3的反馈

// 定义数据结构
typedef struct {
    int32_t speed;      // 实际速度
    int32_t pos_err;    // 位置误差
    uint8_t update_flag;// 收到数据标志
} Motor_Feedback_t;

// 全局数组，存储3个电机状态
volatile Motor_Feedback_t Motors[3];
	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE			1		 			//0,不使能;1,使能.								    
										 							 				    
u8 CAN1_Mode_Init(void);//CAN初始化
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);	//发送数据
u8 CAN1_Msg_Pend(u8 fifox);								//查询邮箱报文
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat);//接收数据
u8 CAN1_Tx_Staus(u8 mbox);  							//返回发送状态
u8 CAN1_Send_Msg(u8* msg,u16 id,u8 len);				//发送数据
u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
#endif

















