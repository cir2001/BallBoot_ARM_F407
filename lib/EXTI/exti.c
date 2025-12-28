#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "key.h"
#include "dma_uart1.h"
#include "mpu6500_driver.h"
#include "mpuiic.h"
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

extern volatile uint8_t g_last_send_index; // 记录待发送的缓冲区索引
u16 u16EXIT0Count,u16EXIT1Count;						  
////////////////////////////////////////////////////////////////////////////////// 
//外部中断0服务程序
void EXTI0_IRQHandler(void)
{
	// 立即读取 MPU 数据
	int16_t r_gx, r_gy, r_gz, r_ax, r_ay, r_az;
	MPU6500_Get_Gyroscope(&r_gx, &r_gy, &r_gz);
	MPU6500_Get_Accelerometer(&r_ax, &r_ay, &r_az);

	// 姿态解算 (Mahony 只有 1ms 积分，F407 开启 FPU 后仅需约 50us)
	AHRS_Update(r_gx * 0.0010653f, r_gy * 0.0010653f, r_gz * 0.0010653f, 
				(float)r_ax, (float)r_ay, (float)r_az, 0.001f);

	// 填充缓冲区 (此时主循环正在忙别的事也不怕，中断会准时填表)
	HybridPacket_t* p_buf = &PingPongBuffer[write_index];
	
	if (sample_in_buf_cnt == 0) p_buf->base_timestamp = sys_ms_ticks;
	
	p_buf->raw_data[sample_in_buf_cnt].accel[0] = r_ax;
	p_buf->raw_data[sample_in_buf_cnt].accel[1] = r_ay;
	p_buf->raw_data[sample_in_buf_cnt].accel[2] = r_az;
	p_buf->raw_data[sample_in_buf_cnt].gyro[0]  = r_gx;
	p_buf->raw_data[sample_in_buf_cnt].gyro[1]  = r_gy;
	p_buf->raw_data[sample_in_buf_cnt].gyro[2]  = r_gz;

	// 10ms 和 20ms 记录姿态
	if (sample_in_buf_cnt == 9 || sample_in_buf_cnt == 19)
	{
		uint8_t c_idx = (sample_in_buf_cnt == 19) ? 1 : 0;
		AHRS_GetEulerAngle(&current_angle);
		p_buf->ctrl_info[c_idx].euler[0] = current_angle.roll;
		p_buf->ctrl_info[c_idx].euler[1] = current_angle.pitch;
		p_buf->ctrl_info[c_idx].euler[2] = current_angle.yaw;
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

    // 发送读取指令给主循环
    //g_mpu_read_ready = 1; 

	//  核心计数：记录“传感器运行时间”
    sys_ms_ticks++; 
    // 4. 清除标志位
    EXTI->PR = 1 << 0;
}	
//外部中断1服务程序
void EXTI1_IRQHandler(void)
{
	EXTI->PR=1<<1;  //清除LINE0上的中断标志位  
	u16EXIT0Count++;
	if(u16EXIT1Count>=50)
	{
		u16EXIT1Count = 0;
	}
	
}
//外部中断2服务程序
void EXTI2_IRQHandler(void)
{
	delay_ms(10);	//消抖
	if(KEY1==0)	  
	{	 

	}		 
	EXTI->PR=1<<2;  //清除LINE2上的中断标志位  
}
//外部中断3服务程序
void EXTI3_IRQHandler(void)
{
	delay_ms(10);	//消抖
	if(KEY0==0)	 
	{
		LED_RAS = !LED_RAS;
	}		 
	EXTI->PR=1<<3;  //清除LINE3上的中断标志位  
}
//外部中断4服务程序
void EXTI4_IRQHandler(void)
{
	delay_ms(10);	//消抖
	if(KEY1==0)	 
	{		
		LED_RAS = !LED_RAS;
	}		 
	EXTI->PR=1<<4;  //清除LINE4上的中断标志位  
}
	   
//外部中断初始化程序
void EXTIX_Init(void)
{
	//KEY_Init(); 

	RCC->AHB1ENR|=1<<1;     //使能PORTB时钟
	GPIO_Set(GPIOB,PIN0|PIN1,GPIO_MODE_IN,0,0,GPIO_PUPD_PU);	//PB0~1设置上拉输入
	
	Ex_NVIC_Config(GPIO_B,0,FTIR); 	//下降沿触发 PB0
	MY_NVIC_Init(0,0,EXTI0_IRQn,2);	//抢占2，子优先级3，组2
	
	Ex_NVIC_Config(GPIO_B,1,FTIR); 	//下降沿触发 PB1
	//MY_NVIC_Init(2,3,EXTI1_IRQn,2);	//抢占2，子优先级3，组2

	Ex_NVIC_Config(GPIO_E,3,FTIR); 	//下降沿触发 PE3
	MY_NVIC_Init(2,3,EXTI3_IRQn,2);	//抢占2，子优先级3，组2

	Ex_NVIC_Config(GPIO_E,4,FTIR); 	//下降沿触发 PE4
	MY_NVIC_Init(2,4,EXTI4_IRQn,2);	//抢占2，子优先级3，组2
}

