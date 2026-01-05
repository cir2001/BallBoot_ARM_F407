#ifndef __MMC5603_H
#define __MMC5603_H	 
#include "sys.h" 
#include "mpuiic.h"
//////////////////////////////////////////////////////////////////////////////////	 
//						  
////////////////////////////////////////////////////////////////////////////////// 	
#define MMC5603_ADDR          0x30      // 7位I2C地址

#define MMC5603_REG_DATA      0x00      // 数据起始寄存器
#define MMC5603_REG_ID        0x39      // 产品ID (0x10)
#define MMC5603_REG_STATUS    0x18      // 状态寄存器
#define MMC5603_REG_ODR       0x1A      // ODR设置寄存器（新版专用）
#define MMC5603_REG_CTRL0     0x1B      // 控制寄存器0（SET/RESET、Auto SR等）
#define MMC5603_REG_CTRL1     0x1C      // 控制寄存器1（BW、软复位）
#define MMC5603_REG_CTRL2     0x1D      // 控制寄存器2（连续模式等）


u8 MMC5603_Init(void);
void MMC5603_ReadData(float *mx, float *my, float *mz);
u8 I2C_Write_Byte(u8 reg, u8 data);
u8 MMC5603_Read_Reg(u8 reg); // 读取寄存器值
 				    
#endif


