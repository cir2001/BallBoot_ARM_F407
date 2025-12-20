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
#define BRR_115200       0x2D9
#define BRR_460800       0x0B6

uint8_t ESP01S_Init_UDP(void);
void ESP_UART1_SendString(char* str);
void ESP_UART1_WaitTC(void);
void ESP_Delay_ms(uint32_t ms);

#endif
