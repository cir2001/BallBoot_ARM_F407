/*
 * mahony.h
 *
 * Created on: Dec 9, 2025
 * Author: Gemini
 * Description: Mahony AHRS 姿态解算算法头文件
 */

#ifndef __MAHONY_H__
#define __MAHONY_H__

#include <math.h>

// ================= 配置参数 =================
// 比例增益 (Kp)
#define MAHONY_KP_DEF  (2.0f * 5.0f) 
// 积分增益 (Ki)
#define MAHONY_KI_DEF  0.005f

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
// @brief 核心解算函数
// @param gx, gy, gz : 陀螺仪数据 (单位：弧度/秒 rad/s) !!!重要!!!
// @param ax, ay, az : 加速度计数据 (单位任意，只要非零)
// @param dt         : 两次调用之间的时间间隔 (单位：秒)
//-------------------------------------------------
void AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

//-------------------------------------------------
//  @brief 获取当前的欧拉角
// @param angle : 指向 IMU_Angle_t 结构体的指针，用于接收结果
//-------------------------------------------------
void AHRS_GetEulerAngle(IMU_Angle_t *angle);

//-------------------------------------------------
// @brief 陀螺仪静态校准
// @param get_gyro_raw_func: 外部提供的读取原始数据的函数指针 (单位: rad/s)
//-------------------------------------------------
void AHRS_Calibrate(void (*get_gyro_raw_func)(float*, float*, float*));

void Read_MPU6500_Gyro(float *x, float *y, float *z);

#endif /* __MAHONY_H__ */



