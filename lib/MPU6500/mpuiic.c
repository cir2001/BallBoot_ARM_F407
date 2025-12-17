#include "mpuiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//							  
//////////////////////////////////////////////////////////////////////////////////

//MPU IIC 延时函数
void MPU_I2C_Delay(void)
{
	delay_us(2);
}
//初始化IIC
void MPU_I2C_Init(void)
{					     
	RCC->AHB1ENR|=1<<1;    //使能PORTB时钟	   	  
	GPIO_Set(GPIOB,PIN10|PIN11,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);	//PB10/PB11设置 
	//GPIO_Set(GPIOB,PIN6|PIN7,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);	//PB6/PB7设置 
	MPU_I2C_SCL=1;
	MPU_I2C_SDA=1;
}
//产生IIC起始信号
void MPU_I2C_Start(void)
{
	MPU_SDA_OUT();     //sda线输出
	MPU_I2C_SDA=1;	  	  
	MPU_I2C_SCL=1;
	MPU_I2C_Delay();
 	MPU_I2C_SDA=0;//START:when CLK is high,DATA change form high to low 
	MPU_I2C_Delay();
	MPU_I2C_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void MPU_I2C_Stop(void)
{
	MPU_SDA_OUT();//sda线输出
	MPU_I2C_SCL=0;
	MPU_I2C_SDA=0;//STOP:when CLK is high DATA change form low to high
 	MPU_I2C_Delay();
	MPU_I2C_SCL=1; 
	MPU_I2C_SDA=1;//发送I2C总线结束信号
	MPU_I2C_Delay();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU_I2C_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA设置为输入  
	MPU_I2C_SDA=1;MPU_I2C_Delay();	   
	MPU_I2C_SCL=1;MPU_I2C_Delay();	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_I2C_Stop();
			return 1;
		}
	}
	MPU_I2C_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void MPU_I2C_Ack(void)
{
	MPU_I2C_SCL=0;
	MPU_SDA_OUT();
	MPU_I2C_SDA=0;
	MPU_I2C_Delay();
	MPU_I2C_SCL=1;
	MPU_I2C_Delay();
	MPU_I2C_SCL=0;
	MPU_I2C_Delay();
	MPU_I2C_SDA=1;  		//20240503
	MPU_I2C_Delay();
}
//不产生ACK应答		    
void MPU_I2C_NAck(void)
{
	MPU_I2C_SCL=0;
	MPU_SDA_OUT();
	MPU_I2C_SDA=1;
	MPU_I2C_Delay();
	MPU_I2C_SCL=1;
	MPU_I2C_Delay();
	MPU_I2C_SCL=0;
	MPU_I2C_Delay();
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_I2C_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_I2C_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        MPU_I2C_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		MPU_I2C_SCL=1;
		MPU_I2C_Delay(); 
		MPU_I2C_SCL=0;	
		MPU_I2C_Delay();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU_I2C_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        MPU_I2C_SCL=0; 
        MPU_I2C_Delay();
		MPU_I2C_SCL=1;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		MPU_I2C_Delay(); 
    }					 
    if (!ack)
        MPU_I2C_NAck();//发送nACK
    else
        MPU_I2C_Ack(); //发送ACK   
    return receive;
}


