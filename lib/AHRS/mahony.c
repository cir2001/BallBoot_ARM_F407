#include "mahony.h"
#include "delay.h"
#include "math.h"
#include "icm42688.h"
#include "mmc5603.h"
#include <string.h>        // 用于 memset (修复了之前的 >> 错误)
// ================= 私有变量 =================
static volatile float twoKp = MAHONY_KP_DEF; 
static volatile float twoKi = MAHONY_KI_DEF;
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

static volatile float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float mag_bias[3] = {-0.164917f, 0.040692f, -0.647417f};
static volatile int is_calibrated = 0;

// 磁场参考方向（西安地区磁倾角 ≈54°）
static float mag_bx = 0.5878f;  // cos(54°)
static float mag_bz = 0.8090f;  // sin(54°)

// ================= 内部辅助函数 =================
static float invSqrt(float x) 
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// ================= 接口函数实现 =================

// 初始化函数
void AHRS_Init(void)
{
    twoKp = MAHONY_KP_DEF;
    twoKi = MAHONY_KI_DEF;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;

    // 计算磁倾角参考方向（西安 54°）
    float mag_inclination_rad = MAG_INCLINATION_DEG * 3.141592653589793f / 180.0f;
    mag_bx = cosf(mag_inclination_rad);
    mag_bz = sinf(mag_inclination_rad);
}

// 核心更新函数
void AHRS_Update_9axis(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    int useMag = 0;

    // 扣除陀螺仪零偏
    if (is_calibrated) {
        gx -= gyro_bias[0];
        gy -= gyro_bias[1];
        gz -= gyro_bias[2];
    }

    mx -= mag_bias[0];
    my -= mag_bias[1];
    mz -= mag_bias[2];

    // 传感器异常过滤
    if (fabsf(ax) > 40000.0f || fabsf(ay) > 40000.0f || fabsf(az) > 40000.0f ||
        fabsf(ax) < 1000.0f && fabsf(ay) < 1000.0f && fabsf(az) < 1000.0f) {  // 防止全小（如自由落体）
        return;  // 直接跳过此帧
    }
    if (fabsf(gx) > 1000.0f || fabsf(gy) > 1000.0f || fabsf(gz) > 1000.0f) {
        return;  // Gyro野值过滤
    }

    // 加速度归一化
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm < 1e-6f) return;
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    // 磁力计归一化
    float mag_norm_sq = mx * mx + my * my + mz * mz;
    if (mag_norm_sq < 1e-6f) {
        useMag = 0;
    } else {
        recipNorm = invSqrt(mag_norm_sq);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
    }

    // 辅助变量
    q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
    q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

    // 重力参考方向
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 磁场参考方向（正确使用磁倾角）
    if (useMag) {
        wx = 2.0f * (mag_bx * (0.5f - q2q2 - q3q3) + mag_bz * (q1q3 - q0q2));
        wy = 2.0f * (mag_bx * (q1q2 - q0q3) + mag_bz * (q0q1 + q2q3));
        wz = 2.0f * (mag_bx * (q1q3 + q0q2) + mag_bz * (0.5f - q1q1 - q2q2));
    }

    // 误差叉积
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    if (useMag) 
    {
        ex += 0.15f * (my * wz - mz * wy);  // 原为 1.0，降到 0.1~0.3
        ey += 0.15f * (mz * wx - mx * wz);
        ez += 0.15f * (mx * wy - my * wx);
    }

    // PI 控制
    if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * dt;
        integralFBy += twoKi * ey * dt;
        integralFBz += twoKi * ez * dt;

        // 积分限幅
        if (fabsf(integralFBx) > 1.0f) integralFBx = 0.0f;
        if (fabsf(integralFBy) > 1.0f) integralFBy = 0.0f;
        if (fabsf(integralFBz) > 1.0f) integralFBz = 0.0f;

        gx += twoKp * ex + integralFBx;
        gy += twoKp * ey + integralFBy;
        gz += twoKp * ez + integralFBz;
    } else {
        gx += twoKp * ex;
        gy += twoKp * ey;
        gz += twoKp * ez;
    }

    // 四元数积分
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 四元数归一化 + 防崩溃
    float norm_sq = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if (norm_sq < 1e-6f || isnan(norm_sq)) {
        q0 = 1.0f; q1 = q2 = q3 = 0.0f;
        return;
    }
    recipNorm = invSqrt(norm_sq);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}
// 核心更新函数
void AHRS_Update_6axis(float gx, float gy, float gz, float ax, float ay, float az, float dt)
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
    if (angle == NULL || isnan(q0 + q1 + q2 + q3)) {
        angle->pitch = angle->roll = angle->yaw = 0.0f;
        return;
    }

    float sinp = -2.0f * (q1 * q3 - q0 * q2);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    angle->pitch = asinf(sinp) * 57.2957795f;
    angle->roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * 57.2957795f;
    angle->yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.2957795f;
}

// 陀螺仪校准
void AHRS_Calibrate(void)
{
    ICM_Data raw_data; 
    float gx, gy, gz;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    const int sample_count = 1000;
    
    const float lsb_to_rads = (1.0f / 16.4f) * (3.141592653589793f / 180.0f);
    
    for(int i = 0; i < sample_count; i++) 
    {
        ICM42688_ReadData(&raw_data);
        gx = (float)raw_data.gyro_x * lsb_to_rads;
        gy = (float)raw_data.gyro_y * lsb_to_rads;
        gz = (float)raw_data.gyro_z * lsb_to_rads;
        sum[0] += gx; sum[1] += gy; sum[2] += gz;
        delay_ms(1); 
    }
    
    gyro_bias[0] = sum[0] / sample_count;
    gyro_bias[1] = sum[1] / sample_count;
    gyro_bias[2] = sum[2] / sample_count;
    is_calibrated = 1;
}