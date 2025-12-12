#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
	   		   
//IO方向设置
//#define MPU_SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PB7输入模式
//#define MPU_SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PB7输出模式

//IO操作函数	 
//#define MPU_I2C_SCL    PBout(6) 		//SCL
//#define MPU_I2C_SDA    PBout(7) 		//SDA	 
//#define MPU_READ_SDA   PBin(7) 		//输入SDA 

//IO方向设置
#define MPU_SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}	//PB11输入模式
#define MPU_SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;} //PB11输出模式

//IO操作函数	 
#define MPU_I2C_SCL    PBout(10) 		//SCL
#define MPU_I2C_SDA    PBout(11) 		//SDA	 
#define MPU_READ_SDA   PBin(11) 		//输入SDA 


//IIC所有操作函数
void MPU_I2C_Delay(void);				//MPU IIC延时函数
void MPU_I2C_Init(void);                //初始化IIC的IO口				 
void MPU_I2C_Start(void);				//发送IIC开始信号
void MPU_I2C_Stop(void);	  			//发送IIC停止信号
void MPU_I2C_Send_Byte(u8 txd);			//IIC发送一个字节
u8 MPU_I2C_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 MPU_I2C_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_I2C_Ack(void);					//IIC发送ACK信号
void MPU_I2C_NAck(void);				//IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif

