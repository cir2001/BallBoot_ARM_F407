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
//**************************************************
// 函数声明
//**************************************************
void Task1(void *pvParameters);
void Task2(void *pvParameters);
void OLED_Refresh(void);
//-------------------------------------------------
// 数据处理结构体
//------------------------------------------------
#pragma pack(1)// 必须加在结构体定义最前面
typedef struct {
    uint32_t timestamp;
    int16_t  accel[3];
    int16_t  gyro[3];
    int32_t  motor_spd[3];
} SampleData_t;

typedef struct {
    uint8_t      head[2];   // 0xAA 0x55
    SampleData_t samples[20];
    uint8_t      tail[2];   // 0x0D 0x0A
} BatchPacket_t;
#pragma pack()// 恢复默认对齐方式
// 定义两个缓冲区：Ping 和 Pong
BatchPacket_t PingPongBuffer[2]; 
volatile uint8_t write_index = 0; // 当前 CPU 正在写入的缓冲区索引 (0 或 1)
volatile uint8_t sample_in_buf_cnt = 0;    // 当前缓冲区内已存的样本数 (0-19)
extern  uint32_t sys_ms_ticks; // 全局毫秒时间戳
//-----------------------------------------------
// 外部变量声明
//-----------------------------------------------
extern	u8  USART1_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
extern	u8  USART1_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
extern	u8  USART2_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
extern	u8  USART2_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
extern	u8  USART3_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
extern	u8  USART3_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.

extern  u8  mpu_data_ready,UART1_RX_Flag;

extern int32_t recv_uart2_M1_val,recv_uart2_M2_val,recv_uart2_M3_val;

extern u8 u8Uart2_flag;
extern int32_t Target_Speed_M1,Target_Speed_M2,Target_Speed_M3;   // 设定的目标速度
//-----------------------------------------------
// 变量声明
//-----------------------------------------------
int16_t gy_X,gy_Y,gy_Z;
int16_t ac_X,ac_Y,ac_Z;

float fpitch,froll,fyaw;

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
        //LED_MA = !LED_MA;
        //UART1ComReply();
        // ============================
        // 主循环：处理紧急任务
        // ============================
		// 检查是否有新指令
        if (u8Uart2_flag == 1)
        {
			// 从串口2接收新指令
			Target_Speed_M1 = recv_uart2_M1_val;//设定范围：-25000~25000
            Target_Speed_M2 = recv_uart2_M2_val;
            Target_Speed_M3 = recv_uart2_M3_val;
            UART2ComReply();
            // 清除标志位
            u8Uart2_flag = 0;
        }
        // MPU data process
        if(mpu_data_ready)
		{
			mpu_data_ready = 0;
            //---- 读取 MPU6500 数据 ----
            
            //--- 准备数据 (单位转换) ---
            // 假设 Gyro 灵敏度为 16.4 LSB/(deg/s) (即量程 2000)
            // 必须转为 rad/s:  Raw / 16.4 * (PI / 180) ≈ Raw * 0.0010653
            float gx = gy_X * 0.0010653f;
            float gy = gy_Y * 0.0010653f;
            float gz = gy_Z * 0.0010653f;
            //--- 更新算法 ---
            AHRS_Update(gx, gy, gz, ac_X, ac_Y, ac_Z, 0.001f);

            // 5. 获取结果
            AHRS_GetEulerAngle(&current_angle);    
            
            // 1. 定位当前使用的缓冲区指针
            BatchPacket_t* p_buf = &PingPongBuffer[write_index];

            // 2. 填充数据并打上时间戳
            p_buf->samples[sample_in_buf_cnt].timestamp = sys_ms_ticks;
            // ... 此处填充 MPU6500 数据和电机转速 ...
            p_buf->samples[sample_in_buf_cnt].accel[0] = 0x01; 
            p_buf->samples[sample_in_buf_cnt].accel[1] = 0x02; 
            p_buf->samples[sample_in_buf_cnt].accel[2] = 0x03; 
            p_buf->samples[sample_in_buf_cnt].gyro[0] = 0x11;
            p_buf->samples[sample_in_buf_cnt].gyro[1] = 0x12;
            p_buf->samples[sample_in_buf_cnt].gyro[2] = 0x13;
            p_buf->samples[sample_in_buf_cnt].motor_spd[0] = 0x21;
            p_buf->samples[sample_in_buf_cnt].motor_spd[1] = 0x22;
            p_buf->samples[sample_in_buf_cnt].motor_spd[2] = 0x23;

            sample_in_buf_cnt++;

            // 3. 满 20 组数据 (20ms 周期到)
            if(sample_in_buf_cnt >= 20)
            {
                // 封装包头包尾
                p_buf->head[0] = 0xAA; p_buf->head[1] = 0x55;
                p_buf->tail[0] = 0x0D; p_buf->tail[1] = 0x0A;

                // 调试用：如果发送失败（DMA忙），翻转另一个 LED 报警 如果上电有问题，使用这段代码
                if((DMA2_Stream7->CR & 0x01) == 0) 
                {
                    DMA_USART1_Start_TX_DoubleBuf((uint32_t)p_buf, sizeof(BatchPacket_t));
                } else 
                {
                    // 如果进到这里，说明 460800 竟然还没发完，这绝对不正常
                    // 可能是串口死锁了
                    LED_MA = !LED_MA; // 让一个 LED 快速闪烁作为报警
                }
                //DMA_USART1_Start_TX_DoubleBuf((uint32_t)p_buf, sizeof(BatchPacket_t));

                // 5. 【关键】立即切换索引，下个 1ms 采样将写到另一个缓冲区
                write_index = !write_index; 
                sample_in_buf_cnt = 0;
            }
       }
        // ============================
        // 主循环：处理非紧急任务
        // ============================
        // 每 100ms 刷新一次屏幕 (1ms * 100 = 100ms)
        if (oled_tick >= 50)
        {
            oled_tick = 0;
            //LED_MA = !LED_MA;
            OLED_Refresh();
            //UART2ComReply();
        }
        //UART1ComReply();
        //UART2ComReply();
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
