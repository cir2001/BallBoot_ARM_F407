#include "mahony.h"
#include "delay.h"
#include "math.h"
#include "icm42688.h"
#include "mmc5603.h"
#include <string.h>        // 用于 memset (修复了之前的 >> 错误)
#include <math.h>
#include "madgwick.h"
#include "exti.h"
// ================= 私有变量 =================
static volatile float twoKp = MAHONY_KP_DEF; 
static volatile float twoKi = MAHONY_KI_DEF;
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

static volatile float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
//static float mag_bias[3] = {-0.164917f, 0.040692f, -0.647417f};
static float mag_bias[3] = {0.0f, 0.0f, 0.0f};
// ================= 内部辅助函数 =================
static float invSqrt(float x) 
{
    float halfx = 0.5f * x;
    float y = x;
    uint32_t i; // 使用 uint32_t 确保在 32 位系统上准确匹配 float 长度

    // 使用 memcpy 代替指针强制转换，规避 Strict-Aliasing 警告
    memcpy(&i, &y, sizeof(i));
    
    i = 0x5f3759df - (i >> 1);
    
    memcpy(&y, &i, sizeof(y));

    y = y * (1.5f - (halfx * y * y));
    // y = y * (1.5f - (halfx * y * y)); // 如果需要更高精度，可以取消注释第二项迭代
    
    return y;
}
// 静态辅助函数：欧拉角转四元数初始化
static void Mahony_SetInitialQuaternion(float roll, float pitch, float yaw) 
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
}
// ================= 接口函数实现 =================
// 初始化函数
void Mahony_Init(void)
{
    twoKp = MAHONY_KP_DEF;
    twoKi = MAHONY_KI_DEF;
}

// 核心更新函数
void Mahony_Update_9axis(float gx, float gy, float gz, float ax, float ay, float az, float mx_raw, float my_raw, float mz_raw, float dt)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    // 1. 先扣除偏置 (确保与 Update 一致)
    float mx_adj = mx_raw - mag_bias[0];
    float my_adj = my_raw - mag_bias[1];
    float mz_adj = mz_raw - mag_bias[2];

    // 2. 唯一映射 (IMU_X = -Mag_Y, IMU_Y = Mag_X)
    float mx = -my_adj; 
    float my = mx_adj;
    float mz = mz_adj;

    // 1. 扣除陀螺仪零偏 (确保 bias 单位与 gx,gy,gz 一致)
    // 如果 gx 是 rad/s，bias 也必须是 rad/s
    gx -= gyro_bias[0];
    gy -= gyro_bias[1];
    gz -= gyro_bias[2];

    // 2. 预计算四元数乘积
    q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
    q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

    // 3. 加速度计处理：修正 Pitch 和 Roll
    float a_norm_sq = ax * ax + ay * ay + az * az;
    // 如果模长太小（例如小于 0.0001，意味着几乎为0），则跳过本次重力修正
    if (a_norm_sq < 1e-6f) {
    // 传感器数据无效，只进行陀螺仪积分，或者直接返回
    return; 
    }

    if (a_norm_sq > 0.001f) {
        recipNorm = invSqrt(a_norm_sq);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        vx = 2.0f * (q1q3 - q0q2);
        vy = 2.0f * (q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);
    }

    // 4. 磁力计处理：修正 Yaw (这是解决漂移的关键)
    float m_norm_sq = mx * mx + my * my + mz * mz;
    if (m_norm_sq > 0.001f) 
    {
        recipNorm = invSqrt(m_norm_sq);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // 将测量磁场转到地理坐标系 (h = q * m * q')
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        
        bx = sqrtf(hx * hx + hy * hy);
        bz = hz;

        // 将地理参考磁场转回机体坐标系 (w = q' * b * q)
        wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
        wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
        wz = 2.0f * (bx * (q1q3 + q0q2) + bz * (0.5f - q1q1 - q2q2));

        // 累加磁场误差项
        ex += (my * wz - mz * wy);
        ey += (mz * wx - mx * wz);
        ez += (mx * wy - my * wx);
    }

    // 5. PI 控制
    if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * dt;
        integralFBy += twoKi * ey * dt;
        integralFBz += twoKi * ez * dt;
        gx += integralFBx; gy += integralFBy; gz += integralFBz;
    }
    gx += twoKp * ex;
    gy += twoKp * ey;
    gz += twoKp * ez;

    // 6. 积分并归一化
    float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    float dq1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
    float dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
    float dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;
    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}
// 核心更新函数
void Mahony_Update_6axis(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 应用校准值
    gx -= gyro_bias[0];
    gy -= gyro_bias[1];
    gz -= gyro_bias[2];

    //  静态死区过滤 (防止极小的噪声引起积分漂移)
    // 对于 MPU6500，0.005 rad/s 是一个比较保守的阈值
    if(fabs(gx) < 0.002f) gx = 0.0f;
    if(fabs(gy) < 0.002f) gy = 0.0f;
    if(fabs(gz) < 0.002f) gz = 0.0f;

    // 1. 如果加速度计数据无效（全0），则无法修正，直接返回
    if(!((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f))) return;

    // 2. 加速度数据归一化
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

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
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 8. 四元数归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}
// 获取欧拉角
void Mahony_GetEulerAngle(IMU_Angle_t *angle)
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
void Mahony_Calibrate(void)
{
    ICM_Data raw_data; 
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;

    float ax_cal = 0.0f, ay_cal = 0.0f, az_cal = 0.0f;
    float mx_cal = 0.0f, my_cal = 0.0f, mz_cal = 0.0f;

    double sum_g[3] = {0.0f, 0.0f, 0.0f};
    double sum_a[3] = {0.0f, 0.0f, 0.0f};
    
    const int sample_count = 1000;
    
    const float gy_lsb_to_rads = (1.0f / 16.4f) * (3.141592653589793f / 180.0f);
    const float acc_lsb_to_g = (1.0f / 16384.0f); // ±2g range
    
    for(int i = 0; i < sample_count; i++) 
    {
        ICM42688_ReadData(&raw_data);
        gx = (float)raw_data.gyro_x * gy_lsb_to_rads;
        gy = (float)raw_data.gyro_y * gy_lsb_to_rads;
        gz = (float)raw_data.gyro_z * gy_lsb_to_rads;
        sum_g[0] += gx; sum_g[1] += gy; sum_g[2] += gz;

        ax = (float)raw_data.acc_x * acc_lsb_to_g;
        ay = (float)raw_data.acc_y * acc_lsb_to_g;  
        az = (float)raw_data.acc_z * acc_lsb_to_g;
        sum_a[0] += ax; sum_a[1] += ay; sum_a[2] += az;

        delay_ms(5); 
    }

    MMC5603_ReadData(&mx, &my, &mz);

    gyro_bias[0] = sum_g[0] / sample_count;
    gyro_bias[1] = sum_g[1] / sample_count;
    gyro_bias[2] = sum_g[2] / sample_count;

    ax_cal = sum_a[0] / sample_count;
    ay_cal = sum_a[1] / sample_count;
    az_cal = sum_a[2] / sample_count;

    mx_cal = mx;
    my_cal = my;
    mz_cal = mz;

    Mahony_Init();

    Mahony_Init_Fast(ax_cal, ay_cal, az_cal, mx_cal, my_cal, mz_cal);
}

//------------------------------------------------------------
//  @brief 快速对准初始化 (消除启动爬升曲线)
//  @param ax, ay, az : 初始加速度
//  @param mx, my, mz : 初始磁力计 (Gauss)
//------------------------------------------------------------
void Mahony_Init_Fast(float ax, float ay, float az, float mx, float my, float mz) 
{
    // 1. 重置积分项
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;

    // 1. 先扣除偏置 (确保与 Update 一致)
    float mx_adj = mx - mag_bias[0];
    float my_adj = my - mag_bias[1];
    float mz_adj = mz - mag_bias[2];

    // 2. 唯一映射 (IMU_X = -Mag_Y, IMU_Y = Mag_X)
    float mx_imu = -my_adj; 
    float my_imu = mx_adj;
    float mz_imu = mz_adj;

    // 2. 加速度计归一化
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    // 3. 计算初始 Roll 和 Pitch (基于重力矢量)
    float initialRoll = atan2f(ay, az);
    float initialPitch = asinf(-ax);

    // 5. 倾斜补偿 (计算初始 Yaw)
    float cosRoll = cosf(initialRoll);
    float sinRoll = sinf(initialRoll);
    float cosPitch = cosf(initialPitch);
    float sinPitch = sinf(initialPitch);

    float mag_x = mx_imu * cosPitch + my_imu * sinRoll * sinPitch + mz_imu * cosRoll * sinPitch;
    float mag_y = my_imu * cosRoll - mz_imu * sinRoll;

    float initialYaw = atan2f(-mag_y, mag_x);

    // 6. 将计算结果直接赋值给四元数
    Mahony_SetInitialQuaternion(initialRoll, initialPitch, initialYaw);
}


