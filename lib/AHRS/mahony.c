//*****************************************************
// mahony.c
// Created on: Dec 9, 2025
//*****************************************************
#include "mahony.h"
#include "delay.h"
#include "math.h"
// ================= 私有变量 (Static) =================
// 使用 static 关键字，确保这些变量只能在本文件内部被访问
static volatile float twoKp = MAHONY_KP_DEF; 
static volatile float twoKi = MAHONY_KI_DEF;
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

static volatile float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static volatile int is_calibrated = 0;
// ================= 内部辅助函数 =================
// 快速倒数平方根 (STM32F4 若开启 FPU，也可用 1.0f/sqrtf(x))
static float invSqrt(float x) 
{
    return 1.0f / sqrtf(x); // F407 开启 FPU 后，sqrtf 非常快
}
// ================= 接口函数实现 =================
// 初始化函数
void AHRS_Init(void)
{
    twoKp = MAHONY_KP_DEF;
    twoKi = MAHONY_KI_DEF;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
}

// 核心更新函数
void AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 应用校准值
    if(is_calibrated) {
        gx -= gyro_bias[0];
        gy -= gyro_bias[1];
        gz -= gyro_bias[2];
    }
    //  静态死区过滤 (防止极小的噪声引起积分漂移)
    // 对于 MPU6500，0.005 rad/s 是一个比较保守的阈值
    if(fabs(gx) < 0.005f) gx = 0.0f;
    if(fabs(gy) < 0.005f) gy = 0.0f;
    if(fabs(gz) < 0.005f) gz = 0.0f;

    // 1. 如果加速度计数据无效（全0），则无法修正，直接返回
    if(!((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f))) return;

    // 2. 加速度数据归一化
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 3. 估计重力方向
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // 4. 计算误差 (测量重力 与 估计重力 的叉积)
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // 5. 积分误差 (Ki)
    if(twoKi > 0.0f) {
        integralFBx += twoKi * halfex * dt;
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // 6. 比例补偿 (Kp)
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;

    // 7. 四元数微分方程积分
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 8. 四元数归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// 获取欧拉角
void AHRS_GetEulerAngle(IMU_Angle_t *angle)
{
    if (angle == 0) return; // 空指针检查

    // 俯仰角 (Pitch)
    angle->pitch = asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29578f; 
    
    // 横滚角 (Roll)
    angle->roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29578f;
    
    // 航向角 (Yaw)
    angle->yaw   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29578f;
}

//-------------------------------------------------------------------
// @brief 陀螺仪静态校准
// @param get_gyro_raw_func: 外部提供的读取原始数据的函数指针 (单位: rad/s)
//-------------------------------------------------------------------
void AHRS_Calibrate(void (*get_gyro_raw_func)(float*, float*, float*))
{
    float gx, gy, gz;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    const int sample_count = 1000;
    
    for(int i = 0; i < sample_count; i++) {
        get_gyro_raw_func(&gx, &gy, &gz);
        sum[0] += gx;
        sum[1] += gy;
        sum[2] += gz;
        delay_ms(1); // 采样间隔，F407 建议使用系统延时
    }
    
    gyro_bias[0] = sum[0] / sample_count;
    gyro_bias[1] = sum[1] / sample_count;
    gyro_bias[2] = sum[2] / sample_count;
    is_calibrated = 1;
}
