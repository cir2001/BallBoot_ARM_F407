#ifndef __ESP01S_H
#define __ESP01S_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//					  
////////////////////////////////////////////////////////////////////////////////// 	 

// --- wifi配置 ---
#define WIFI_SSID     "DDWW_Nwpu"    // WiFi名称
#define WIFI_PWD      "acq902m3"     // WiFi密码
#define TARGET_IP     "192.168.0.111"    // 上位机 QT 所在的 IP
#define TARGET_PORT   "8080"              // 上位机 UDP 端口
#define LOCAL_PORT    "1234"              // ESP-01S 本地端口

// 寄存器波特率计算值 (84MHz)
/* 适用于 USART1 (总线频率 84MHz) */
#define BRR_115200          0x2D9   // 84000000 / (16 * 115200) = 45.57 -> 45(0x2D) , 0.57*16=9(0x9)
#define BRR_460800          0x0B6   // 84000000 / (16 * 460800) = 11.39 -> 11(0x0B) , 0.39*16=6(0x6)
#define BRR_921600          0x05B   // 84000000 / (16 * 921600) = 5.69  -> 5(0x05)  , 0.69*16=11(0xB)

#define BRR_500000          0x0A8  // 84,000,000 / (16 * 500,000) = 10.5 -> 10(0xA) , 0.5*16=8(0x8)  
#define BRR_600000         0x08C   // 84,000,000 / (16 * 600,000) = 8.75 -> 8(0x08) , 0.75*16=12(0xC)
#define BRR_750000         0x070   // 84,000,000 / (16 * 750,000) = 7.0 -> 7(0x07) , 0.0*16=0(0x0)

uint8_t ESP01S_Init_UDP(void);
void ESP_UART1_SendString(char* str);
void ESP_UART1_WaitTC(void);
void ESP_Delay_ms(uint32_t ms);

#endif
