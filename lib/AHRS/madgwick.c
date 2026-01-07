#include "madgwick.h"
#include "delay.h"     // 延时函数
#include "icm42688.h"  // 传感器驱动
#include "mmc5603.h"   // 传感器驱动
#include <string.h>   
#include <math.h>
// ================= 私有变量 =================
volatile float beta = MADGWICK_BETA_DEF;
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// 传感器校准参数
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float mag_bias[3] = {0.0565f, 0.0540f, -0.5500f};
static float mag_scale[3] = {1.0f, 0.99f, 1.0f}; // Y轴稍微压缩一点点
// ================= 内部辅助函数 =================
// 快速平方根倒数
static float invSqrt(float x) 
{
    float halfx = 0.5f * x;
    float y = x;
    uint32_t i; 
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
// 欧拉角转四元数 (用于快速初始化)
static void Madgwick_SetInitialQuaternion(float roll, float pitch, float yaw) 
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

void Madgwick_Init(void)
{
    //beta = MADGWICK_BETA_DEF;
    beta = 1.5f;
}

void Madgwick_Set_MagCalib_Data(float offset_x, float offset_y, float offset_z, 
                                float scale_x,  float scale_y,  float scale_z)
{
    mag_bias[0] = offset_x; mag_bias[1] = offset_y; mag_bias[2] = offset_z;
    mag_scale[0] = scale_x;   mag_scale[1] = scale_y;   mag_scale[2] = scale_z;
}
// ---------------------------------------------------------------------------------------------------
// Madgwick 9轴融合算法
// ---------------------------------------------------------------------------------------------------
void Madgwick_Update_9axis(float gx, float gy, float gz, float ax, float ay, float az, float mx_raw, float my_raw, float mz_raw, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    // =================================================================
    // 1. 硬铁 (Hard Iron) 去偏
    // =================================================================
    float mx_adj = mx_raw - mag_bias[0];
    float my_adj = my_raw - mag_bias[1];
    float mz_adj = mz_raw - mag_bias[2];
    // =================================================================
    // 2. 软铁 (Soft Iron) 拉圆
    // =================================================================
    mx_adj *= mag_scale[0];
    my_adj *= mag_scale[1];
    mz_adj *= mag_scale[2];

    // 2. 唯一映射 (IMU_X = -Mag_Y, IMU_Y = Mag_X)
    float mx = -my_adj; 
    float my = -mx_adj;
    float mz =  mz_adj;

    // 3. 陀螺仪零偏补偿
    gx -= gyro_bias[0];
    gy -= gyro_bias[1];
    gz -= gyro_bias[2];

    // 4. 计算反馈增益 (梯度下降)
    // 如果加速度计无效，无法校正
    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        return;
    }

    float a_norm_sq = ax * ax + ay * ay + az * az;

    // 如果模长太小（例如小于 0.0001，意味着几乎为0），则跳过本次重力修正
    if (a_norm_sq < 1e-6f) {
        // 传感器数据无效，只进行陀螺仪积分，或者直接返回
        return; 
    }

    // 归一化加速度
    recipNorm = invSqrt(a_norm_sq);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    // 归一化磁力计
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

    // 辅助变量计算
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
    q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

    // 参考磁场计算 (Reference direction)
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // 梯度下降法计算误差方向 (Gradient decent algorithm corrective step)
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    // 归一化梯度
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    // 计算四元数导数 (Gyro term - Beta * Gradient term)
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    // 四元数积分
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 归一化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}
// ---------------------------------------------------------------------------------------------------
// Madgwick 6轴融合算法
// ---------------------------------------------------------------------------------------------------
void Madgwick_Update_6axis(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx -= gyro_bias[0];
    gy -= gyro_bias[1];
    gz -= gyro_bias[2];

    // 计算四元数导数 (基于陀螺仪)
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // 如果加速度计有效，计算梯度下降修正
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        // 梯度下降
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        // 应用 Beta 修正
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 积分
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}
// 获取欧拉角 (Roll, Pitch, Yaw)
// 单位：度 (Degree)
void Madgwick_GetEulerAngle(IMU_Angle_t *angle)
{
    if (angle == NULL) return;

    // 57.29578f = 180 / PI
    const float radToDeg = 57.2957795f;

    // 1. 计算 Roll (横滚角, 绕 X 轴)
    // atan2(2(q0q1 + q2q3), 1 - 2(q1^2 + q2^2))
    angle->roll = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * radToDeg;

    // 2. 计算 Pitch (俯仰角, 绕 Y 轴)
    // 注意：这里的符号定义与 Mahony 保持一致。如果发现 Pitch 方向反了，可以去掉负号。
    float sinp = -2.0f * (q1 * q3 - q0 * q2);
    
    // 防止 asin 超出定义域 [-1, 1] 导致计算结果为 NaN
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    
    angle->pitch = asinf(sinp) * radToDeg;

    // 3. 计算 Yaw (航向角, 绕 Z 轴)
    // atan2(2(q1q2 + q0q3), 1 - 2(q2^2 + q3^2))
    angle->yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * radToDeg;
}
// 陀螺仪校准
void Madgwick_Calibrate(void)
{
    ICM_Data raw_data; 
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;

    float ax_cal = 0.0f, ay_cal = 0.0f, az_cal = 0.0f;
    float mx_cal = 0.0f, my_cal = 0.0f, mz_cal = 0.0f;

    double sum_g[3] = {0.0, 0.0, 0.0};
    double sum_a[3] = {0.0, 0.0, 0.0};
    double sum_m[3] = {0.0, 0.0, 0.0};
    
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

        MMC5603_ReadData(&mx, &my, &mz);
        sum_m[0] += mx; sum_m[1] += my; sum_m[2] += mz;

        delay_ms(2); 
    }
    gyro_bias[0] = (float)(sum_g[0] / sample_count);
    gyro_bias[1] = (float)(sum_g[1] / sample_count);
    gyro_bias[2] = (float)(sum_g[2] / sample_count);

    ax_cal = (float)(sum_a[0] / sample_count);
    ay_cal = (float)(sum_a[1] / sample_count);
    az_cal = (float)(sum_a[2] / sample_count);

    mx_cal = (float)(sum_m[0] / sample_count);
    my_cal = (float)(sum_m[1] / sample_count);
    mz_cal = (float)(sum_m[2] / sample_count);

    Madgwick_Init();
    
    Madgwick_Init_Fast(ax_cal, ay_cal, az_cal, mx_cal, my_cal, mz_cal);
}
//------------------------------------------------------------
//  @brief 快速对准初始化 (消除启动爬升曲线)
//  @param ax, ay, az : 初始加速度
//  @param mx, my, mz : 初始磁力计 (Gauss)
//------------------------------------------------------------
void Madgwick_Init_Fast(float ax, float ay, float az, float mx, float my, float mz) 
{
    // =================================================================
    // 1. 硬铁 (Hard Iron) 去偏
    // =================================================================
    float mx_adj = mx - mag_bias[0];
    float my_adj = my - mag_bias[1];
    float mz_adj = mz - mag_bias[2];
    // =================================================================
    // 2. 软铁 (Soft Iron) 拉圆
    // =================================================================
    mx_adj *= mag_scale[0];
    my_adj *= mag_scale[1];
    mz_adj *= mag_scale[2];
    
    // 唯一映
    float mx_imu = -my_adj; 
    float my_imu = -mx_adj;
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
    Madgwick_SetInitialQuaternion(initialRoll, initialPitch, initialYaw);
}

