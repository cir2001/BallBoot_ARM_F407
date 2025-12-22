//*****************************************************************
//  Project Name: 球形机器人F407
//	File Name: BallBot_Arm.C	 
//  芯片：STM32F407VGT6 PCB 版本:V0.0
//  version: V0.0
//	Author: ZJ
//	At:	Xi'an China
//*****************************************************************
//	功能：
//		1. 0.96寸OLED显示屏 SPI接口
//		2. USART1,USART2,USART3同时使用
//		3. mpu6500 DMP V6.12 移植 对内容进行了针对stm32F407的修改
//		   模拟I2C通讯方式 如果MPU初始化无法正确读出地址值，考虑连接线屏蔽问题
//		   采用绞线方式连接
//		4. TIM8的CH1~CH4通道输出PWM波，驱动电机 （减速电机方案，暂未使用）
//		5. TIM2,TIM3，TIM4,TIM5作为编码器接口使用 （减速电机方案，暂未使用）
//		6. CAN总线通讯，波特率500Kbps
//		7. FreeRTOS实时操作系统移植
//   	8. 预留SPI1接口，可连接树莓派等主控
//  	9. TIM1作为中断定时器使用 （减速电机方案，暂未使用）
// 		10.KEY按键扫描
//*****************************************************************	
//=================================================================		
//		date		comment
//	2025-12-02   	初始化
//  2025-12-17      PlatformIO 移植
//  2025-12-17      F407 F103 联机测试 串口2接收指令，下发3个F103，指令格式: #S+20000,-20000,+20000.
//*****************************************************************
//==== 引脚连接表 ============
//---- MPU6500 ----------
//	mpu6500（I2C）				stm32f407(模拟I2C)
//		3.3v					3.3v
//		GND						GND
//		SCL						PB10(I2C2)	PB8(I2C1)	PB6
//		SDA						PB11(I2C2)	PB9(I2C1)	PB7 
//		AD0/SDO					GND(addr 0x70)
//		INT1					PB0(EXIT0)
//    	INT2					PB1(EXIT1)
// 		FSYNC					PB2
//---- 0.96寸 OLED --------
//	0.96寸OLED(SD1306)			stm32f407(软件模拟SPI)
//		GND										GND
//		Vcc										3.3V
//		SCK	（SPI时钟线）		OLED_SCL		 PB13
//		SDA	（SPI数据线）		OLED_SDA		 PB15
//		RES （复位）			OLED_RST		 PB12
//		DC  （数据与命令切换）	 OLED_RS		  PB14
//		CS   (片选)				OLED_CS			 PD10	
//----- CAN1 总线 -------
//		CAN_TX					PA12
//		CAN_RX					PA11
//-------- 电机驱动  减速电机方案（暂未使用）---------------------------
//		Motor A PWM Out		TIM8_CH1		PC6
//		Motor B PWM Out		TIM8_CH2		PC7
//		Motor C PWM Out		TIM8_CH3		PC8
//		Motor D PWM Out		TIM8_CH4		PC9
//----- 编码器接口 -------
//		Motor A Encoder B			TIM2_CH1		PA15
//		Motor A Encoder A			TIM2_CH2		PB3
//		Motor B Encoder B			TIM3_CH1		PB4
//		Motor B Encoder A			TIM3_CH2		PB5
//      Motor C Encoder B			TIM4_CH1		PB6
//		Motor C Encoder A			TIM4_CH2		PB7
//		Motor D Encoder B			TIM5_CH1		PA0
//		Motor D Encoder A			TIM5_CH2		PA1
//----- 电机方向 -------
//		Motor A AIN2			PC10
//		Motor A AIN1			PC11
//		Motor B BIN1			PC12
//		Motor B BIN2			PD0
//		Motor C AIN2			PB8
//		Motor C AIN1			PB9
//		Motor D BIN1			PE0
//		Motor D BIN2			PE1
//---------------- USART接口  --------------------------------------- 
//--- ESP-01S  DMA方式 ---
//	UART1   115200，1start，8bit，1stop，no parity 
//		USART1_TX			PA9
//		USART1_RX			PA10
//--- 中断方式 ---
//	UART2   115200，1start，8bit，1stop，no parity 
//		USART2_TX			PA2
//		USART2_RX			PA3
//--- 蓝牙模块 ---
// 	UART3   115200，1start，8bit，1stop，no parity 
//		USART3_TX			PD8
//		USART3_RX			PD9
//-------------- 树莓派SPI接口 ---------------------------------------
//  	SPI1_MOSI           PA7
// 	 	SPI1_MISO           PA6
//  	SPI1_SCK            PA5
//  	SPI1_NSS            PA4
//=========================================================
//说明：
//====LED===============================================
//		LED_MC	<------->		PE2   
//		LED_U2	<------->		PC4
//		LED_MA	<------->		PC5
//      LED_RAS	<------->		PE8
//	  	LED_MPU	<------->		PE15
//   	LED_OLED<------->		PD11
// 		LED_U3	<------->		PD12
// 		LED_U1	<------->		PD13
//    	LED_MW	<------->		PD14
//		LED_CAN	<------->		PD15
//=== KEY ===============================================
//		KEY0	<------->		PE3
//		KEY1	<------->		PE4
//=========================================================
#include "sys.h"
#include "usart.h" 
#include "delay.h" 
#include "led.h" 
#include "spi.h"
#include "math.h"
#include "mpu6500_driver.h"
#include "mpuiic.h"
#include "exti.h" 
#include "stdint.h"			//定义bool变量需要加入这行
#include "stdbool.h"		//定义bool变量需要加入这行
#include "timer.h"
#include "can.h"
#include "oled.h"
#include "key.h"
#include "dma_driver.h"
#include "mahony.h"
#include "esp01s.h"

#include "FreeRTOS.h"
#include "task.h"   
#include "queue.h"

#include <string.h>  // 必须包含，否则无法识别 memset
#include <math.h>    // 必须包含，否则无法识别 NAN 常量
//=================================================
#define LPF_ALPHA 0.3f
//**************************************************
// 函数声明
//**************************************************
void Task1(void *pvParameters);
void Task2(void *pvParameters);
void OLED_Refresh(void);
uint8_t DMA_Submit_And_Switch_Buffer(void);
void Buffer_Clear_And_Init(HybridPacket_t *buf);

// 定义两个缓冲区：Ping 和 Pong
HybridPacket_t PingPongBuffer[2]; 
volatile uint8_t  write_index = 0;       // 当前写入缓冲区的索引
volatile uint8_t  sample_in_buf_cnt = 0; // 当前缓冲区已存样本数 (0-9)
volatile uint8_t  g_control_logic_flag = 0; // 10ms控制触发标志
volatile uint8_t g_send_data_flag = 0; // 20ms发送触发标志
volatile uint8_t ctrl_index = 0; 		// 用于指示当前是 20ms 里的第几次控制

extern volatile uint32_t sys_ms_ticks;      // 全局毫秒时间戳
extern volatile uint8_t g_mpu_read_ready; // 引入外部标志位
//-----------------------------------------------
// 外部变量声明
//-----------------------------------------------
extern	u8  USART1_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
extern	u8  USART1_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
extern	u8  USART2_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
extern	u8  USART2_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
extern	u8  USART3_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
extern	u8  USART3_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.

extern  u8  mpu_data_ready,UART1_RX_Flag,u8Uart2_flag;
extern  u8  RxD2pt;

extern int32_t recv_uart2_M1_val,recv_uart2_M2_val,recv_uart2_M3_val;


extern int32_t Target_Speed_M1,Target_Speed_M2,Target_Speed_M3;   // 设定的目标速度
//-----------------------------------------------
// 变量声明
//-----------------------------------------------
/* --- 系统状态变量  --- */
static uint16_t global_packet_id = 0; // 仅在 main 使用，普通 static 即可

int16_t gy_X,gy_Y,gy_Z;
int16_t ac_X,ac_Y,ac_Z;

float f_acc[3], f_gyro[3]; // 滤波变量

u8 i,res,res1;

u8 res,u8Led0_Counter;
u8 mpu_int_count;

u32 u16counter;

u16 oled_tick;

// 定义一个结构体变量存放角度
IMU_Angle_t current_angle;
//int16_t cnt;
//int16_t iMotorA_Encoder,iMotorB_Encoder,iMotorC_Encoder;
//long int iMotorAPulseTotle,iMotorBPulseTotle,iMotorCPulseTotle;
//======= 主程序 Main ==================================
int main(void)
{ 
    Stm32_Clock_Init(336,8,2,7);//设置时钟,168Mhz
	delay_init(168);			//初始化延时函数

    // 1. 使能 Trace 单元 (必须先开启才能使用 DWT)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // 2. DWT 计数器清零
    DWT->CYCCNT = 0;
    // 3. 使能 DWT 周期计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    KEY_Init();
	LED_Init();

	// --- OLED 最先启动，方便观察 ---
    SPI2_Init();
    delay_us(10);
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0,(u8*)"System Booting...", 16);
    OLED_Refresh_Gram(); // 第一次刷新
    delay_ms(3000); // 保持 2 秒

    // --- 初始化传感器，但不开启定时器发送 ---
    OLED_ShowString(0, 16, (u8*)"Init MPU...", 16);
    OLED_Refresh_Gram();
    delay_ms(2000); // 保持 2 秒
    if(MPU6500_Init() != 0) 
    {
        // 如果初始化失败，让一个 LED 长亮或快闪报警
        while(1) { LED_MA = !LED_MA; delay_ms(100); }
    }
    delay_ms(100); // 等待 MPU6500 准备好

    // --- 初始化 AHRS 算法 ---
    AHRS_Init();

    // --- 初始化串口 ---
    uart_init1(84,115200);
    uart_init2(42,115200);
    uart_init3(42,115200);
    delay_ms(100); // 等待串口初始化完成

    //--- 初始化 CAN ---
    CAN1_Mode_Init();
    delay_ms(100); // 等待 CAN 初始化完成

	//--- 初始化 ESP-01S ---
    OLED_ShowString(0, 16, (u8*)"Init ESP...", 16);
    OLED_Refresh_Gram();
    ESP01S_Init_UDP();

	// 初始化 TIM1，设定为 200ms 中断一次
	// 参数1 (ARR): 1999  -> 计数 2000 次
	// 参数2 (PSC): 16799 -> 预分频 16800 (时钟变为 10kHz, 0.1ms)
	//TIM1_Int_Init(1999, 16799);
	TIM2_Int_Init(500-1,840-1); //5ms
//	TIM3_Int_Init(500-1,840-1);//10Khz的计数频率，计数5K次为500ms 

    // 配置 DMA 1 用于 USART1 发送接收 
    OLED_ShowString(0, 16, (u8*)"Init DMA...", 16);
    OLED_Refresh_Gram();
    delay_ms(100); // 等待 CAN 初始化完成
    DMA_Config_USART1();    //  应在ESP-01S 初始化后进行
    delay_ms(100); // 等待 初始化完成

    // 启动外部中断
    OLED_ShowString(0, 16, (u8*)"Init EXTI...", 16);
    OLED_Refresh_Gram();
    delay_ms(100); // 等待 CAN 初始化完成
    EXTIX_Init();

    OLED_Clear();
    OLED_ShowString(0, 0, (u8*)"System Online", 16);
    OLED_Refresh_Gram(); // 刷新
    delay_ms(1000);
    //--- OLED 进入主界面 ---
    OLED_Clear();
    OLED_ShowString(0, 0, (u8*)" Tar    AS   Sta", 16);
    OLED_Refresh_Gram(); // 刷新 
	
//------------------ FreeRTOS 任务创建 -------------------
    // 创建两个LED任务  FreeREOS部分
	//xTaskCreate(Task1, "LED1_Task", 128, NULL, 1, NULL); 
	//xTaskCreate(Task2, "LED2_Task", 128, NULL, 1, NULL); 
	// 启动调度器 
	//vTaskStartScheduler();
//------------------ FreeRTOS 任务创建结束 -------------------
    // 初始化所有电机状态
    for(int i=0; i<3; i++)
    {
        Motors[i].last_tick = 0; // 初始为 0
        Motors[i].status = 0x80;  // 初始状态设为错误/离线
    }
//******** 主循环程序 ***********//
	while(1)
	{
        // ==========================================================
        // 任务 1：MPU6500 数据采集与缓冲管理 (原 EXTI0 逻辑)
        // ==========================================================
        if (g_mpu_read_ready) 
        {
            g_mpu_read_ready = 0;
            // 1. 读取 MPU6500 原始数据 (1ms 一次)
	        // 注意：此处读取必须足够快，建议使用硬件 I2C 或高速 SPI
	        int16_t r_ax, r_ay, r_az, r_gx, r_gy, r_gz;
	        MPU6500_Get_Gyroscope(&r_gx, &r_gy, &r_gz);
	        MPU6500_Get_Accelerometer(&r_ax, &r_ay, &r_az);

            // 2. 一阶低通滤波
            f_acc[0] = LPF_ALPHA * r_ax + (1.0f - LPF_ALPHA) * f_acc[0];
            f_acc[1] = LPF_ALPHA * r_ay + (1.0f - LPF_ALPHA) * f_acc[1];
            f_acc[2] = LPF_ALPHA * r_az + (1.0f - LPF_ALPHA) * f_acc[2];
            f_gyro[0] = LPF_ALPHA * r_gx + (1.0f - LPF_ALPHA) * f_gyro[0];
            f_gyro[1] = LPF_ALPHA * r_gy + (1.0f - LPF_ALPHA) * f_gyro[1];
            f_gyro[2] = LPF_ALPHA * r_gz + (1.0f - LPF_ALPHA) * f_gyro[2];

            // 3. 压入当前双缓冲区
            if (sample_in_buf_cnt < 20)
            {
                if (sample_in_buf_cnt == 0) 
                {
                    PingPongBuffer[write_index].base_timestamp = sys_ms_ticks;
                }
                RawSample_t *s = &PingPongBuffer[write_index].raw_data[sample_in_buf_cnt];
                s->accel[0] = (int16_t)f_acc[0]; s->accel[1] = (int16_t)f_acc[1]; s->accel[2] = (int16_t)f_acc[2];
                s->gyro[0]  = (int16_t)f_gyro[0]; s->gyro[1]  = (int16_t)f_gyro[1]; s->gyro[2]  = (int16_t)f_gyro[2];
                
                sample_in_buf_cnt++;
            
                // 3. 触发 10ms 控制逻辑 (100Hz)
                if (sample_in_buf_cnt == 10) {
                    ctrl_index = 0;           // 第一个 10ms 周期
                    g_control_logic_flag = 1;
                } 
                else if (sample_in_buf_cnt == 20) {
                    ctrl_index = 1;           // 第二个 10ms 周期
                    g_control_logic_flag = 1;
                    g_send_data_flag = 1;     // 同时触发 20ms 发送
                }
            } else {
                // 【调试用】如果运行到这里，说明 while(1) 没能及时重置计数器
                // 可以在这里点亮一个错误灯，代表系统实时性存在风险
            }
        }
        // ==========================================================
        // 任务 2：串口 2 指令解析 (#S+20000,-20000,+20000.)
        // ==========================================================
		// 检查是否有新指令
        if (u8Uart2_flag == 1)
        {
            // 1. 基本校验：确保包头和类型正确 (#S)
            if (USART2_RX_BUF[0] == '#' && USART2_RX_BUF[1] == 'S')
            {
                // 2. 使用局部变量临时存储解析结果，防止中途出错影响电机运行
                int32_t val1 = 0, val2 = 0, val3 = 0;
                
                // 3. 安全解析逻辑 (格式: #S+20000,-20000,+20000.)
                // 解析 Motor 1
                val1 = (USART2_RX_BUF[3]-'0')*10000 + (USART2_RX_BUF[4]-'0')*1000 + 
                    (USART2_RX_BUF[5]-'0')*100   + (USART2_RX_BUF[6]-'0')*10 + (USART2_RX_BUF[7]-'0');
                if(USART2_RX_BUF[2] == '-') val1 = -val1;

                // 解析 Motor 2
                val2 = (USART2_RX_BUF[10]-'0')*10000 + (USART2_RX_BUF[11]-'0')*1000 + 
                    (USART2_RX_BUF[12]-'0')*100   + (USART2_RX_BUF[13]-'0')*10 + (USART2_RX_BUF[14]-'0');
                if(USART2_RX_BUF[9] == '-') val2 = -val2;

                // 解析 Motor 3
                val3 = (USART2_RX_BUF[17]-'0')*10000 + (USART2_RX_BUF[18]-'0')*1000 + 
                    (USART2_RX_BUF[19]-'0')*100   + (USART2_RX_BUF[20]-'0')*10 + (USART2_RX_BUF[21]-'0');
                if(USART2_RX_BUF[16] == '-') val3 = -val3;

                // 4. 范围限幅与生效
                Target_Speed_M1 = (val1 > 25000) ? 25000 : ((val1 < -25000) ? -25000 : val1);
                Target_Speed_M2 = (val2 > 25000) ? 25000 : ((val2 < -25000) ? -25000 : val2);
                Target_Speed_M3 = (val3 > 25000) ? 25000 : ((val3 < -25000) ? -25000 : val3);
                
                UART2ComReply(); // 反馈解析成功
            }
            
            // 5. 解析完成，清零标志位并重置指针
            u8Uart2_flag = 0;
            RxD2pt = 0; 
        }
        // ==========================================================
        // 任务 3：姿态解算 (Mahony) 与 控制计算
        // ==========================================================
        // 获取当前正在处理的缓冲区指针
        HybridPacket_t* p_buf = &PingPongBuffer[write_index];
        // 1. 检查是否凑齐了 10ms 数据
        if(g_control_logic_flag)
        {
            g_control_logic_flag = 0;
            uint32_t start_cycles = DWT->CYCCNT; 

            // 执行姿态解算 (Mahony)
            // --- 1. 高频姿态更新循环 ---
            // 根据 ctrl_index 确定读取 raw_data 的起始位置
            // ctrl_index=0 -> offset=0, 读取 0-9ms 数据
            // ctrl_index=1 -> offset=10, 读取 10-19ms 数据
            int offset = ctrl_index * 10;

            // 虽然我们在 10ms 的逻辑里，但我们要把过去 10ms 的轨迹“补”回来
            for(int i = 0; i < 10; i++) 
            {
                // 提取对应毫秒的数据 (注意 i + offset)
                float gx = p_buf->raw_data[i + offset].gyro[0] * 0.0010653f;
                float gy = p_buf->raw_data[i + offset].gyro[1] * 0.0010653f;
                float gz = p_buf->raw_data[i + offset].gyro[2] * 0.0010653f;
                float ax = p_buf->raw_data[i + offset].accel[0];
                float ay = p_buf->raw_data[i + offset].accel[1];
                float az = p_buf->raw_data[i + offset].accel[2];
            
                // 核心：调用 10 次更新，每次 dt 必须是 0.001f (1ms)
                // 这样 AHRS 内部的四元数或旋转矩阵是连续积分的
                AHRS_Update(gx, gy, gz, ax, ay, az, 0.001f); 
            }

            // --- 2. 此时获取的欧拉角是融合了 10 组数据后的最新姿态 ---
            AHRS_GetEulerAngle(&current_angle);

            // --- 3. 执行一次控制计算 (PID/LQR) ---
            // 控制不需要 1ms 一次，因为电机的物理响应没那么快
            //BallBot_Control_Calculate(current_angle);

            // 3. 填充控制量到当前正在写入的缓冲区
            p_buf->ctrl_info[ctrl_index].euler[0] = current_angle.roll;
            p_buf->ctrl_info[ctrl_index].euler[1] = current_angle.pitch;
            p_buf->ctrl_info[ctrl_index].euler[2] = current_angle.yaw;
            
            p_buf->ctrl_info[ctrl_index].motor_out[0] = Target_Speed_M1;
            p_buf->ctrl_info[ctrl_index].motor_out[1] = Target_Speed_M2;
            p_buf->ctrl_info[ctrl_index].motor_out[2] = Target_Speed_M3;

            // --- 5. 统计耗时并提交发送 ---
            // 耗时计算 (us): (结束周期 - 开始周期) / (主频MHz)
            p_buf->ctrl_info[ctrl_index].ctrl_dt = (DWT->CYCCNT - start_cycles) / 168;

        }
        // ==========================================================
        // 任务 4：数据外发 (DMA 提交)
        // ==========================================================
        if (g_send_data_flag) 
        {
            g_send_data_flag = 0;
         
            p_buf->packet_id = global_packet_id++;

            // 切换缓冲区、重置计数器、触发 DMA
            DMA_Submit_And_Switch_Buffer(); 
        }
        // ==========================================================
        // 任务 5：低频任务 (OLED 刷新等)
        // ==========================================================
        // 每 100ms 刷新一次屏幕 (1ms * 100 = 100ms)
        if (oled_tick >= 50)
        {
            oled_tick = 0;
            //LED_MA = !LED_MA;
            OLED_Refresh();
            //UART2ComReply();
        }
	}
}
//------------------------------------------------------
// OLED 主界面刷新
//------------------------------------------------------
void OLED_Refresh(void)
{
    char buf[20]; 
    uint32_t current_time = Get_System_Tick();
    // --- 第一行 (Y=16) ---
    sprintf(buf, "%6ld %5ld", Target_Speed_M1, Motors[0].AS5600_val);
    OLED_ShowString(0, 16, (u8*)buf, 16);
    // 1. 判断物理层是否掉线 (超时检测)
    // 如果当前时间比上次收到回包的时间大 100ms
    if (current_time - Motors[0].last_tick > 20) //100ms timer2 5ms 
    {
        OLED_ShowString(104, 16, (u8*)"OFF", 16); 
    }
    // 2. 判断协议层是否报错 (F103 发来的心跳保护位)
    else if (Motors[0].status & 0x80) 
    {
        OLED_ShowString(104, 16, (u8*)"ERR", 16); 
    }
    // 3. 正常运行
    else 
    {
        OLED_ShowString(104, 16, (u8*)" OK", 16); 
    }

    // --- 第二行 (Y=32) ---
    sprintf(buf, "%6ld %5ld", Target_Speed_M2, Motors[1].AS5600_val);
    OLED_ShowString(0, 32, (u8*)buf, 16);
    // 1. 判断物理层是否掉线 (超时检测)
    // 如果当前时间比上次收到回包的时间大 100ms
    if (current_time - Motors[1].last_tick > 20) //100ms timer2 5ms 
    {
        OLED_ShowString(104, 32, (u8*)"OFF", 16); 
    }
    // 2. 判断协议层是否报错 (F103 发来的心跳保护位)
    else if (Motors[1].status & 0x80) 
    {
        OLED_ShowString(104, 32, (u8*)"ERR", 16); 
    }
    // 3. 正常运行
    else 
    {
        OLED_ShowString(104, 32, (u8*)" OK", 16); 
    }

    // --- 第三行 (Y=48) ---
    sprintf(buf, "%6ld %5ld", Target_Speed_M3, Motors[2].AS5600_val);
    OLED_ShowString(0, 48, (u8*)buf, 16);
    // 1. 判断物理层是否掉线 (超时检测)
    // 如果当前时间比上次收到回包的时间大 100ms
    if (current_time - Motors[2].last_tick > 20) //100ms timer2 5ms 
    {
        OLED_ShowString(104, 48, (u8*)"OFF", 16); 
    }
    // 2. 判断协议层是否报错 (F103 发来的心跳保护位)
    else if (Motors[2].status & 0x80) 
    {
        OLED_ShowString(104, 48, (u8*)"ERR", 16); 
    }
    // 3. 正常运行
    else 
    {
        OLED_ShowString(104, 48, (u8*)" OK", 16); 
    }

    OLED_Refresh_Gram();
}
//------------------------------------------------
// 裸机环境下的双缓冲切换与提交函数
//------------------------------------------------
uint8_t DMA_Submit_And_Switch_Buffer(void) 
{
    uint8_t last_write_index;

    // --- 进入临界区：关闭全局中断，防止 EXTI 中断在此期间修改指针 ---
    __disable_irq(); 

    last_write_index = write_index; // 记录刚刚写满的索引
    write_index = !write_index;     // 切换到另一个缓冲区
    sample_in_buf_cnt = 0;          // 重置采样计数器

    // --- 退出临界区：恢复全局中断 ---
    __enable_irq(); 

    // 触发异步 DMA 发送
    // 确保 DMA 当前空闲
    if((DMA2_Stream7->CR & 0x01) == 0) 
    {
        DMA_USART1_Start_TX_DoubleBuf((uint32_t)&PingPongBuffer[last_write_index], sizeof(HybridPacket_t));
    }

    // 立刻把下一轮要写的这一桶 (新 write_index) 填满无效值
    // 这样接下来的 20ms 内，没收到的 CAN 报文位就会保持 NAN
    Buffer_Clear_And_Init(&PingPongBuffer[write_index]);

    return last_write_index;
}
//--------------------------------------------------------
// @brief 将指定的缓冲区初始化为无效状态
// @param buf 指向需要清理的结构体指针
//--------------------------------------------------------
void Buffer_Clear_And_Init(HybridPacket_t *buf) 
{
    //清理即将使用的缓冲区
    // 使用 memset 将整个结构体清零。这会清除旧的 raw_data 和 motor_data
    // sizeof(HybridPacket_t) 约为 379 字节，F407 跑 memset 极快（< 1us）
    memset(buf, 0, sizeof(HybridPacket_t));

    // 填充固定协议头尾 (提前填好，发包时就不用管了)
    buf->head[0] = 0xAA;
    buf->head[1] = 0x55;
    buf->type = 0x01;       //前期不区分快慢帧，统一20ms间隔发送
    buf->tail[0] = 0x0D;
    buf->tail[1] = 0x0A;

    // 3. 填充浮点数无效值 (使用 NAN)
    // 传感器原始数据通常是 int16，直接清零即可，但电机角度是 float
    for (int t = 0; t < 4; t++) {
        for (int m = 0; m < 3; m++) {
            buf->motor_data[t][m].angle = 0xFFFF;    // 设为无效状态码
            buf->motor_data[t][m].status = 0xFFFF;  // 设为无效状态码
        }
    }

    // 4. 填充控制数据无效值
    for (int i = 0; i < 2; i++) {
        buf->ctrl_info[i].euler[0] = NAN;
        buf->ctrl_info[i].euler[1] = NAN;
        buf->ctrl_info[i].euler[2] = NAN;
        buf->ctrl_info[i].ctrl_dt  = 0; // 耗时清零
    }
}
//===========================================================================
//
//  FreeRtos function
//
//===========================================================================
void vApplicationIdleHook(void)
{
}

void vApplicationTickHook(void)
{
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for( ;; );
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for( ;; );
}

/* 任务1：PD12闪烁 */
void Task1(void *pvParameters)
{
while (1)
{
	//LED1=!LED1;
	vTaskDelay(pdMS_TO_TICKS(500)); // 延时500ms
}
}

/* 任务2：PD13闪烁 */
void Task2(void *pvParameters)
{
while (1)
{
	//LED0=!LED0;
	vTaskDelay(pdMS_TO_TICKS(1000)); // 延时1000ms
}
}
