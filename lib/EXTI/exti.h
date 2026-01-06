#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"  	
//////////////////////////////////////////////////////////////////////////////////	 
//								  
////////////////////////////////////////////////////////////////////////////////// 	 
#define EXIT_PB0 PBout(0)   //

#define EN_MAHONY       1  // 1:使能 Mahony 算法 0:关闭
#define EN_MADGWICK     0   // 1:使能 Madgwick 算法 0:关闭
//-------------------------------------------------
// 数据处理结构体
//------------------------------------------------
#pragma pack(1)// 必须加在结构体定义最前面
typedef struct 
{
    float angle;  
    float speed;      //当前实时转速 (deg/s)    
    uint16_t status;     
} MotorFeedback_t;

// 1ms 原始采样数据点
typedef struct 
{
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];
} RawSample_t; // 12 bytes

// 10ms 控制状态反馈
typedef struct 
{
    float    euler[3];     // 姿态角 (Roll, Pitch, Yaw)
    int16_t  motor_out[3]; // 电机输出值
    uint16_t ctrl_dt;      // 算法实际耗时 (微秒)
} ControlStatus_t;

typedef struct      // 降低带宽的占用，每20ms发送一次
{
    uint8_t         head[2];            // 0xAA 0x55
    uint8_t         type;               // 0x01 占位，后续可以慢变量的上传
    uint16_t        packet_id;          // 包序号
    uint32_t        base_timestamp;     // 包首帧时间戳
    RawSample_t     raw_data[20];       // 20次原始采样
    ControlStatus_t ctrl_info[2];       // 2次控制解算信息
    MotorFeedback_t motor_data[4][3];   // 4个时间槽, 3个电机  F103 5ms发送间隔
    uint8_t         tail[2];            // 0x0D 0x0A
} HybridPacket_t;

//接收QT发送的指令
typedef struct {
    uint8_t  head[2];      // 0xAA, 0x55
    uint8_t  type;         // 0x01: PID, 0x02: Mode
    uint8_t  loop_type;    // 0: 平衡, 1: 速度
    float    p;
    float    i;
    float    d;
    uint8_t  tail[2];      // 0x0D, 0x0A
} CommandPacket_t;

#pragma pack()// 恢复默认对齐方式

void EXTIX_Init(void);	//外部中断初始化		
 					    
#endif

