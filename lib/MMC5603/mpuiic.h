#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"

// I2C2 硬件接口定义
// SCL -> PB10
// SDA -> PB11

void I2C_Hardware_Init(void);
u8 I2C_Write_Reg(u8 addr, u8 reg, u8 data);
u8 I2C_Read_Regs(u8 addr, u8 reg, u8 *buf, u8 len);
u8 I2C_Read_Reg(u8 addr, u8 reg);

#endif