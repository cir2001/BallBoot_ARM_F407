#ifndef __MAHONY_H__
#define __MAHONY_H__

#include <math.h>
// ================= 配置参数 =================
// 比例增益 (Kp)
#define MAHONY_KP_DEF  (2.0f * 5.0f) 
// 积分增益 (Ki)
#define MAHONY_KI_DEF  0.01f
// ================= 数据结构 =================
// 用于存储解算后的欧拉角 (单位：度)
typedef struct {
    float pitch; // 俯仰角
    float roll;  // 横滚角
    float yaw;   // 航向角
} IMU_Angle_t;

// ================= 函数声明 =================

//-------------------------------------------------
// @brief 初始化 Mahony 算法参数 (可用于复位)
//-------------------------------------------------
void AHRS_Init(void);

//-------------------------------------------------
// @brief 9轴核心解算函数 (增加磁力计)
// @param gx, gy, gz : 陀螺仪数据 (单位：rad/s)
// @param ax, ay, az : 加速度计数据 (单位：g 或 原始值)
// @param mx, my, mz : 磁力计数据 (单位：Gauss 或 原始值)
// @param dt         : 时间间隔 (单位：s)
//-------------------------------------------------
void AHRS_Update_9axis(float gx, float gy, float gz, float ax, float ay, float az, float mx_raw, float my_raw, float mz_raw, float dt);

void AHRS_Update_6axis(float gx, float gy, float gz, float ax, float ay, float az,float dt);

//-------------------------------------------------
//  @brief 获取当前的欧拉角
// @param angle : 指向 IMU_Angle_t 结构体的指针，用于接收结果
//-------------------------------------------------
void AHRS_GetEulerAngle(IMU_Angle_t *angle);

//-------------------------------------------------
// @brief 陀螺仪静态校准
// @param get_gyro_raw_func: 外部提供的读取原始数据的函数指针 (单位: rad/s)
//-------------------------------------------------
void AHRS_Calibrate(void);

static void AHRS_SetInitialQuaternion(float roll, float pitch, float yaw);
void AHRS_Init_Fast(float ax, float ay, float az, float mx, float my, float mz);


#endif /* __MAHONY_H__ */



