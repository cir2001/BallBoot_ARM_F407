#ifndef __MADGWICK_H
#define __MADGWICK_H

#include <stdint.h>
#include <math.h>
#include "mahony.h"
// ================= 参数定义 =================
// Beta 增益: 0.05f ~ 0.5f 是常用范围
// 0.1f 适合大多数无人机/平衡车，既有一定抗噪性，回正速度也够
#define MADGWICK_BETA_DEF   0.1f    
// ================= 函数声明 =================

// 初始化
void Madgwick_Init(void);

// 快速初始化 (利用加速度计和磁力计立刻锁定姿态，消除启动收敛时间)
// ax, ay, az: g
// mx, my, mz: uT/Gauss
void Madgwick_Init_Fast(float ax, float ay, float az, float mx, float my, float mz);

// 设置磁力计校准参数 (硬铁偏移 + 软铁比例)
void Madgwick_Set_MagCalib_Data(float offset_x, float offset_y, float offset_z, 
                                float scale_x,  float scale_y,  float scale_z);

// 陀螺仪静态校准 (需保持静止)
void Madgwick_Gyro_Calibrate_Run(void);

// 9轴融合更新 (含磁力计)
// dt: 采样周期 (秒)
void Madgwick_Update_9axis(float gx, float gy, float gz, 
                           float ax, float ay, float az, 
                           float mx_raw, float my_raw, float mz_raw, 
                           float dt);

// 6轴融合更新 (无磁力计)
void Madgwick_Update_6axis(float gx, float gy, float gz, 
                           float ax, float ay, float az, 
                           float dt);

// 获取解算后的欧拉角
void Madgwick_GetEulerAngle(IMU_Angle_t *angle);

void Madgwick_Calibrate(void);

#endif // __MADGWICK_H


