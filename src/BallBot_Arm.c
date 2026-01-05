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
//  芯片说明
//--- MMC5603C 3轴数字磁力计----
// 国产芯片的ADDR因版本不同，为0x30-0x38，需问卖家
// 芯片版本不同，器件ID返回值不定，因此无需进行ID号读取
//=========================================================
#include "sys.h"
#include "usart.h" 
#include "delay.h" 
#include "led.h" 
#include "spi.h"
#include "math.h"
#include "mpuiic.h"
#include "exti.h" 
#include "stdint.h"			//定义bool变量需要加入这行
#include "stdbool.h"		//定义bool变量需要加入这行
#include "timer.h"
#include "can.h"
#include "oled.h"
#include "key.h"
#include "dma_uart1.h"
#include "dma_uart2.h"
#include "mahony.h"
#include "esp01s.h"
#include "icm42688.h"
#include "mmc5603.h" 

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
void OLED_Refresh_Sliced(void);
uint8_t DMA_Submit_And_Switch_Buffer(void);
void Buffer_Clear_And_Init(HybridPacket_t *buf);
int fast_itoa(int32_t val, char* buf, uint8_t width, char pad);
//-----------------------------------------------
// 缓冲区管理变量
HybridPacket_t PingPongBuffer[2]; 
volatile uint8_t  write_index = 0;       // 当前写入缓冲区的索引
volatile uint8_t  sample_in_buf_cnt = 0; // 当前缓冲区已存样本数 (0-19)
volatile uint8_t  g_send_data_flag = 0;  // 20ms发送触发标志
volatile uint8_t  g_control_logic_flag = 0; // 10ms控制触发标志
volatile uint8_t ctrl_index = 0; 		// 用于指示当前是 20ms 里的第几次控制

volatile uint8_t g_last_send_index; // 记录待发送的缓冲区索引

extern volatile uint32_t sys_ms_ticks;      // 全局毫秒时间戳
extern volatile uint8_t g_mpu_read_ready; // 引入外部标志位

extern volatile uint32_t DMA_Busy_Drop_Count;
//-----------------------------------------------
// 外部变量声明
//-----------------------------------------------
extern	u8  USART1_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
extern	u8  USART1_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
extern	u8  USART2_RX_BUF[USART2_MAX_RX_LEN];     //接收缓冲,最大128个字节.
extern	u8  USART2_TX_BUF[USART2_MAX_RX_LEN];     //发送缓冲,最大128个字节.
extern	u8  USART3_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
extern	u8  USART3_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.

extern  u8  mpu_data_ready,UART1_RX_Flag,u8Uart2_rx_flag;
extern  u8  RxD2pt;

extern int32_t recv_uart2_M1_val,recv_uart2_M2_val,recv_uart2_M3_val;


extern int32_t Target_Speed_M1,Target_Speed_M2,Target_Speed_M3;   // 设定的目标速度
//-----------------------------------------------
// 变量声明
//-----------------------------------------------
/* --- 系统状态变量  --- */
static uint16_t global_packet_id = 0; // 仅在 main 使用，普通 static 即可
volatile float g_mag_x = 0.0f, g_mag_y = 0.0f, g_mag_z = 0.0f; // 主循环读到的校准后的磁力计数据

int16_t gy_X,gy_Y,gy_Z;
int16_t ac_X,ac_Y,ac_Z;

float f_acc[3], f_gyro[3]; // 滤波变量


u8 i,res,res1;

u8 oled_tick;
u8 magic_tick;

// 定义一个结构体变量存放角度
IMU_Angle_t current_angle;
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
    SPI1_Init();
    delay_ms(100);

	// --- OLED 最先启动，方便观察 ---
    SPI2_Init();
    delay_us(10);
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0,(u8*)"System Booting..  ", 16);
    OLED_Refresh_Gram(); // 第一次刷新
    delay_ms(3000); // 保持 2 秒

    
    // --- 初始化传感器，但不开启定时器发送 ---
    OLED_ShowString(0, 16, (u8*)"Init IMU ICM  ", 16);
    OLED_Refresh_Gram();
    delay_ms(2000); // 保持 2 秒
    if(ICM42688_Init() == 0) 
    {
        OLED_ShowString(0, 16, (u8*)"ICM Failed!  ", 16);
        OLED_Refresh_Gram();
        delay_ms(1000);
        // 如果初始化失败，让一个 LED 长亮或快闪报警
        while(1) { LED_MA = !LED_MA; delay_ms(100); }
    }else if(ICM42688_Init() == 1)
    {
        // 执行校准 (此时必须保证板子静止！)
        OLED_ShowString(0, 16, (u8*)"Success!    ", 16);
        OLED_Refresh_Gram();
        delay_ms(1000);
        OLED_ShowString(0, 16, (u8*)"Calibrate..  ", 16);
        OLED_Refresh_Gram();
        delay_ms(500); // 保持 1 秒
        // --- 初始化 AHRS 算法 ---
        AHRS_Init();
        AHRS_Calibrate(); // 启动时自动校准，需保持机器人静止
        OLED_ShowString(0, 16, (u8*)"ICM Done!   ", 16);
        OLED_Refresh_Gram();
        delay_ms(1000);
    }

    OLED_ShowString(0, 16, (u8*)"Init IMU MMC   ", 16);
    OLED_Refresh_Gram();
    delay_ms(2000); // 保持 2 秒
    if(MMC5603_Init() == 0) 
    {
        OLED_ShowString(0, 16, (u8*)"MMC Failed!   ", 16);
        OLED_Refresh_Gram();
        delay_ms(1000);
        // 如果初始化失败，让一个 LED 长亮或快闪报警
        while(1) { LED_MA = !LED_MA; delay_ms(100); }
    }else if(MMC5603_Init() == 1)
    {
        // 执行校准 (此时必须保证板子静止！)
        OLED_ShowString(0, 16, (u8*)"MMC Success!  ", 16);
        OLED_Refresh_Gram();
        delay_ms(1000);
    }

    // --- 初始化串口 ---
    // 串口1
    OLED_ShowString(0, 16, (u8*)"Init UART1...   ", 16);
    OLED_Refresh_Gram();
    delay_ms(500); // 保持 1 秒
    uart_init1(84,921600);
    // 串口2
    OLED_ShowString(0, 16, (u8*)"Init UART2...   ", 16);
    OLED_Refresh_Gram();
    delay_ms(500); // 保持 1 秒
    DMA_Config_USART2(115200);
    //uart_init3(42,115200);
    //delay_ms(100); // 等待串口初始化完成

    //--- 初始化 CAN ---
    OLED_ShowString(0, 16, (u8*)"Init CAN...   ", 16);
    OLED_Refresh_Gram();
    delay_ms(500); // 保持 1 秒
    CAN1_Mode_Init();
    delay_ms(100); // 等待 CAN 初始化完成

	//--- 初始化 ESP-01S ---
    //OLED_ShowString(0, 16, (u8*)"Init ESP...", 16);
    //OLED_Refresh_Gram();
    //ESP01S_Init_UDP();

	// 初始化 TIM1，设定为 200ms 中断一次
	// 参数1 (ARR): 1999  -> 计数 2000 次
	// 参数2 (PSC): 16799 -> 预分频 16800 (时钟变为 10kHz, 0.1ms)
	//TIM1_Int_Init(1999, 16799);
    OLED_ShowString(0, 16, (u8*)"Init TIMER...   ", 16);
    OLED_Refresh_Gram();
    delay_ms(500); // 保持 1 秒
	TIM2_Int_Init(500-1,840-1); //5ms
//	TIM3_Int_Init(500-1,840-1);//10Khz的计数频率，计数5K次为500ms 

    // 配置 DMA 1 用于 USART1 发送接收 
    OLED_ShowString(0, 16, (u8*)"Init DMA...   ", 16);
    OLED_Refresh_Gram();
    delay_ms(500); // 等待 CAN 初始化完成
    DMA_Config_USART1();    //  应在ESP-01S 初始化后进行
    delay_ms(100); // 等待 初始化完成

    // 在开启采样中断前，预填两个缓冲区的固定字节
    for(int i = 0; i < 2; i++)
    {
        PingPongBuffer[i].head[0] = 0xAA;
        PingPongBuffer[i].head[1] = 0x55;
        PingPongBuffer[i].type    = 0x01;
        PingPongBuffer[i].tail[0] = 0x0D;
        PingPongBuffer[i].tail[1] = 0x0A;
        
        // 初始化时清零数据区
        memset(PingPongBuffer[i].raw_data, 0, sizeof(PingPongBuffer[i].raw_data));
        memset(PingPongBuffer[i].ctrl_info, 0, sizeof(PingPongBuffer[i].ctrl_info));
    }

    // 启动外部中断
    OLED_ShowString(0, 16, (u8*)"Init EXTI...   ", 16);
    OLED_Refresh_Gram();
    delay_ms(500); // 等待 CAN 初始化完成
   
    EXTIX_Init();

    OLED_Clear();
    OLED_ShowString(0, 0, (u8*)"System Online    ", 16);
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
        // 任务1. 数据外发 (20ms 一次，由 DMA 异步完成)
        // ==========================================================
        if (g_send_data_flag) 
        {
            g_send_data_flag = 0;
            
            // 增加包序号 (对记录的旧索引操作)
            PingPongBuffer[g_last_send_index].packet_id = global_packet_id++;

            // 检查 DMA 是否忙碌并发送
            if((DMA2_Stream7->CR & 0x01) == 0) 
            {
                DMA_USART1_Start_TX_DoubleBuf((uint32_t)&PingPongBuffer[g_last_send_index], sizeof(HybridPacket_t));
            }
            else
            {
                DMA_Busy_Drop_Count++; // 依然记录忙碌状态
            }

            
        }
        // ==========================================================
        // 任务2：串口 2 指令解析 (#S+20000,-20000,+20000.)
        // ==========================================================
		// 检查是否有新指令
        if (u8Uart2_rx_flag) 
        {
            int val1 = 0, val2 = 0, val3 = 0;
            
            // 1. 解析指令 (例如发送 #S,120,50\n)
            if (sscanf((char*)USART2_RX_BUF, "#S%d,%d,%d", &val1, &val2, &val3) == 3) 
            {
                //Target_Speed_M1 = (val1 > 25000) ? 25000 : ((val1 < -25000) ? -25000 : val1);
                //Target_Speed_M2 = (val2 > 25000) ? 25000 : ((val2 < -25000) ? -25000 : val2);
                //Target_Speed_M3 = (val3 > 25000) ? 25000 : ((val3 < -25000) ? -25000 : val3);

                Target_Speed_M1 = val1;
                Target_Speed_M2 = val2;
                Target_Speed_M3 = val3;
                
                // 2. 数据回报：确认修改成功
                USART2_Printf("%ld %ld %ld\r\n", Target_Speed_M1,Target_Speed_M2,Target_Speed_M3);
            } 
            else if (strstr((char*)USART2_RX_BUF, "#R")) 
            {
                // 3. 数据回报：读取当前状态
                //USART2_Printf("STATE: Roll=%.2f Pitch=%.2f\r\n", current_angle.roll, current_angle.pitch);
            }

            // 处理完毕，清空标志
            u8Uart2_rx_flag = 0;
            memset(USART2_RX_BUF, 0, USART2_MAX_RX_LEN);
        }
        // ==========================================================
        // 任务 低频任务 (Magic 刷新)
        // ==========================================================
        // 每 50ms 刷新一次
        if (magic_tick >= 10)
        {
            magic_tick = 0;
            //MMC5603_ReadData(&g_mag_x, &g_mag_y, &g_mag_z);
            //printf("X: %.3f G   Y: %.3f G   Z: %.3f G\r\n", g_mag_x, g_mag_y, g_mag_z);
           //delay_ms(100);  // 10Hz打
        }
        // ==========================================================
        // 任务 低频任务 (OLED 刷新等)
        // ==========================================================
        // 每 100ms 刷新一次屏幕 (1ms * 100 = 100ms)
        if (oled_tick >= 10)
        {
            oled_tick = 0;
            //LED_MA = !LED_MA;
            OLED_Refresh_Sliced(); // 时间切片  降低刷新过程cpu的负载
            //UART2ComReply();
        }
	}
}
//---------------------------------------------------------------------
// @brief 分步刷新 OLED 界面 (状态机模式)
// @note 建议每 10ms-20ms 调用一次，完成一次全屏刷新需要 4 次调用
//--------------------------------------------------------------------
void OLED_Refresh_Sliced(void)
{
    static uint8_t refresh_step = 0;
    char display_buf[24]; 
    char* p = display_buf;
    int32_t target_speeds[3] = {Target_Speed_M1, Target_Speed_M2, Target_Speed_M3};
    //int32_t target_speeds[3] = {DMA_Busy_Drop_Count, DMA_Busy_Drop_Count, DMA_Busy_Drop_Count};

    switch (refresh_step) 
    {
        case 0: 
        case 1: 
        case 2: 
        {
            uint8_t i = refresh_step;
            
            // 使用 fast_itoa 替换 sprintf
            // 渲染目标速度，宽度 6，空格填充
            p += fast_itoa(target_speeds[i], p, 6, ' ');
            *p++ = '|'; // 分隔符
            // 显示下位机传回的实际转速 (取整显示，或者用浮点转换)
            //  显示整数转速
            p += fast_itoa((int32_t)Motors[i].speed, p, 5, ' ');
            OLED_ShowString(0, 16 + (i * 16), (u8*)display_buf, 16);
            
            // 状态逻辑判定保持不变...
            const char* status_str = (Get_System_Tick() - Motors[i].last_tick > 20) ? "OFF" : 
                                    (Motors[i].status & 0x80 ? "ERR" : " OK");
            
            OLED_ShowString(104, 16 + (i * 16), (u8*)status_str, 16);
            break;
        }
        case 3: 
            OLED_Refresh_Gram(); 
            break;
    }

    if (++refresh_step >= 4) refresh_step = 0;
}
//--------------------------------------------------------
//@brief 极简整数转字符串 (支持负数和固定宽度填充)
//@param val: 待转换的 32 位整数
//@param buf: 目标缓冲区
//@param width: 最小显示宽度 (0 表示不填充)
//@param pad: 填充字符 (通常为 ' ' 或 '0')
//@return 返回生成的字符串长度
//-----------------------------------------------------------
int fast_itoa(int32_t val, char* buf, uint8_t width, char pad) {
    char temp[12];
    uint8_t i = 0, j = 0;
    uint32_t abs_val;
    uint8_t negative = 0;

    // 1. 处理负数
    if (val < 0) {
        negative = 1;
        abs_val = -val;
    } else {
        abs_val = val;
    }

    // 2. 逐位取余 (逆序)
    if (abs_val == 0) {
        temp[i++] = '0';
    } else {
        while (abs_val > 0) {
            temp[i++] = (abs_val % 10) + '0';
            abs_val /= 10;
        }
    }

    // 3. 计算符号和填充
    uint8_t actual_width = i + negative;
    while (actual_width < width) {
        buf[j++] = pad;
        actual_width++;
    }

    if (negative) buf[j++] = '-';

    // 4. 逆序拷贝到输出缓冲区
    while (i > 0) {
        buf[j++] = temp[--i];
    }

    buf[j] = '\0'; // 结束符
    return j;
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
    }else
    {
        DMA_Busy_Drop_Count++; // 如果 DMA 还没发完，说明波特率还是不够快
    }
    // 立刻把下一轮要写的这一桶 (新 write_index) 填满无效值
    // 这样接下来的 20ms 内，没收到的 CAN 报文位就会保持 NAN
    //Buffer_Clear_And_Init(&PingPongBuffer[write_index]);

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
    memset(buf->raw_data, 0, sizeof(buf->raw_data));
    memset(buf->ctrl_info, 0, sizeof(buf->ctrl_info));

    // 填充固定协议头尾 (提前填好，发包时就不用管了)
    buf->head[0] = 0xAA;
    buf->head[1] = 0x55;
    buf->type = 0x01;       //前期不区分快慢帧，统一20ms间隔发送
    buf->tail[0] = 0x0D;
    buf->tail[1] = 0x0A;
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



