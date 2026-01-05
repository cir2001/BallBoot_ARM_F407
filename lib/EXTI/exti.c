#include "exti.h"
#include "led.h" 
#include "key.h"
#include "dma_uart1.h"
#include "icm42688.h"
#include "mahony.h"
//////////////////////////////////////////////////////////////////////////////////	
// 定义一个结构体变量存放角度
IMU_Angle_t current_angle;
//---------------------------------------------
volatile uint32_t sys_ms_ticks = 0;      // 全局毫秒时间戳
//volatile uint8_t g_mpu_read_ready = 0; // MPU读取触发标志
extern HybridPacket_t PingPongBuffer[2];
extern volatile uint8_t write_index;
extern volatile uint8_t sample_in_buf_cnt;
extern volatile uint8_t g_send_data_flag;

extern volatile int Target_Speed_M1; 
extern volatile int Target_Speed_M2;
extern volatile int Target_Speed_M3;

extern volatile uint8_t g_last_send_index; // 记录待发送的缓冲区索引

extern float g_mag_x, g_mag_y, g_mag_z; // 主循环读到的校准后的磁力计数据

u16 u16EXIT0Count,u16EXIT1Count;	

volatile ICM_Data g_imu_data;
////////////////////////////////////////////////////////////////////////////////// 
//外部中断0服务程序
void EXTI0_IRQHandler(void)
{
	uint32_t start_count, stop_count; // 用于记录 DWT 计数值
	// 立即读取 MPU 数据
	//int16_t r_gx, r_gy, r_gz, r_ax, r_ay, r_az;
	ICM42688_ReadData((ICM_Data*)&g_imu_data);

	float gx_rad = (float)g_imu_data.gyro_x / 16.4f * (3.1415926f / 180.0f);
	float gy_rad = (float)g_imu_data.gyro_y / 16.4f * (3.1415926f / 180.0f);
	float gz_rad = (float)g_imu_data.gyro_z / 16.4f * (3.1415926f / 180.0f);

	float acc_x_g = (float)g_imu_data.acc_x /16384.0f;
	float acc_y_g = (float)g_imu_data.acc_y /16384.0f;
	float acc_z_g = (float)g_imu_data.acc_z /16384.0f;
	// --- 开始计时 ---
    start_count = DWT->CYCCNT;

	// 姿态解算 (Mahony 只有 1ms 积分，F407 开启 FPU 后仅需约 50us)
	// AHRS_Update_9axis(gx_rad, gy_rad, gz_rad, 
	//                acc_x_g, acc_y_g, acc_z_g, 
	//                g_mag_x, g_mag_y, g_mag_z, 0.001f);

	// AHRS_Update_9axis(gx_rad, gy_rad, gz_rad, 
	//                acc_x_g, acc_y_g, acc_z_g, 
	//                0, 0, 0, 0.001f);

	AHRS_Update_6axis(gx_rad, gy_rad, gz_rad,
					  acc_x_g, acc_y_g, acc_z_g, 0.001f);

	// --- 结束计时 ---
    stop_count = DWT->CYCCNT;

	// 填充缓冲区 (此时主循环正在忙别的事也不怕，中断会准时填表)
	HybridPacket_t* p_buf = &PingPongBuffer[write_index];
	
	if (sample_in_buf_cnt == 0) p_buf->base_timestamp = sys_ms_ticks;
	
	p_buf->raw_data[sample_in_buf_cnt].accel[0] = g_imu_data.acc_x; 
    p_buf->raw_data[sample_in_buf_cnt].accel[1] = g_imu_data.acc_y;
    p_buf->raw_data[sample_in_buf_cnt].accel[2] = g_imu_data.acc_z;
    p_buf->raw_data[sample_in_buf_cnt].gyro[0]  = g_imu_data.gyro_x;
    p_buf->raw_data[sample_in_buf_cnt].gyro[1]  = g_imu_data.gyro_y;
    p_buf->raw_data[sample_in_buf_cnt].gyro[2]  = g_imu_data.gyro_z;

	// 10ms 和 20ms 记录姿态
	if (sample_in_buf_cnt == 9 || sample_in_buf_cnt == 19)
	{
		uint8_t c_idx = (sample_in_buf_cnt == 19) ? 1 : 0;
		AHRS_GetEulerAngle(&current_angle);
		p_buf->ctrl_info[c_idx].euler[0] = current_angle.roll;
		p_buf->ctrl_info[c_idx].euler[1] = current_angle.pitch;
		p_buf->ctrl_info[c_idx].euler[2] = current_angle.yaw;

		// 计算耗时：(结束值 - 开始值) / (SystemCoreClock / 1,000,000)
        // F407 运行在 168MHz，所以直接除以 168 即可得到微秒
        uint32_t diff = stop_count - start_count;
        p_buf->ctrl_info[c_idx].ctrl_dt = (uint16_t)(diff / 168);

		p_buf->ctrl_info[c_idx].motor_out[0] = (int16_t)Target_Speed_M1;
		p_buf->ctrl_info[c_idx].motor_out[1] = (int16_t)Target_Speed_M2;
		p_buf->ctrl_info[c_idx].motor_out[2] = (int16_t)Target_Speed_M3;
	}

	sample_in_buf_cnt++;
	if (sample_in_buf_cnt >= 20)
	{
		g_last_send_index = write_index; // 记录当前填满的缓冲区
		write_index = !write_index;      // 立即切换到另一个缓冲区
		sample_in_buf_cnt = 0;           // 重置计数，下一毫秒立刻开始填新表
		g_send_data_flag = 1;            // 仅通知主循环：有一包数据可以发了
	}
	
    // 状态闪烁 (每250次采样翻转一次)
    u16EXIT0Count++;
    if(u16EXIT0Count >= 250) {
        u16EXIT0Count = 0;
        LED_MPU = !LED_MPU; 
    }

	//  核心计数：记录“传感器运行时间”
    sys_ms_ticks++; 
    // 4. 清除标志位
    EXTI->PR = 1 << 0;
}	

//外部中断初始化程序
void EXTIX_Init(void)
{
	//KEY_Init(); 
	RCC->AHB1ENR|=1<<1;     //使能PORTB时钟
	GPIO_Set(GPIOB,PIN0|PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_PU);	//PB0~1设置上拉输入
	
	Ex_NVIC_Config(GPIO_B,0,FTIR); 	//下降沿触发 PB0
	//Ex_NVIC_Config(GPIO_B, 0, RTIR);  // 上升沿触发，数据一产生立即进入 ISR
	MY_NVIC_Init(0,0,EXTI0_IRQn,2);	//抢占2，子优先级3，组2
}

