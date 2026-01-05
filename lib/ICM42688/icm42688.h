#ifndef __ICM42688_H
#define __ICM42688_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//						  
////////////////////////////////////////////////////////////////////////////////// 	
#define ICM_REG_BANK_SEL      0x76
#define ICM_REG_DEVICE_CONFIG 0x11
#define ICM_REG_DRIVE_CONFIG  0x13
#define ICM_REG_INT_CONFIG    0x14
#define ICM_REG_FIFO_CONFIG   0x16

#define ICM_REG_PWR_MGMT_0    0x4E
#define ICM_REG_ACCEL_DATA_X1 0x1F  // 连续读取起始地址
#define ICM_REG_GYRO_DATA_X1  0x25  // 连续读取起始地址
#define ICM_REG_WHO_AM_I      0x75

#define ICM_REG_INT_CONFIG    0x14
#define ICM_REG_INT_SOURCE0   0x65

#define ICM_REG_GYRO_CONFIG0  0x4F  // 正确地址
#define ICM_REG_ACCEL_CONFIG0 0x50
//---------------------------------------------
typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
} ICM_Data;
//---------------------------------------------
// 函数声明
//---------------------------------------------
void ICM42688_ReadData(ICM_Data* data) ;
uint8_t ICM42688_Init(void);
uint8_t ICM_ReadReg(uint8_t reg);
void ICM_WriteReg(uint8_t reg, uint8_t data);
		 				    
#endif


