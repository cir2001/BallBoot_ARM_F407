#include "mpu6500_driver.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"   

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "math_fun.h"
#include "math.h"


/*******************************************/
extern u8 USART1_TX_BUF[128],USART2_TX_BUF[128];
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;
//////////////////////////////////////////////////////////////////////////////////	 
//
/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};
 
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
		
		 .orientation = {0, -1, 0,
                     1, 0, 0,
                     0, 0, 1}
};
 
////////////////////////////////////////////////////////////////////////////////// 
//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU6500_Init(void)
{ 
	u8 res = 0;
		
	MPU_I2C_Init();//初始化IIC总线
	
	MPU6500_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
  	delay_ms(100);
	MPU6500_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	
	MPU6500_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU6500_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU6500_Set_Rate(1000);						//设置采样率50Hz
	MPU6500_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU6500_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU6500_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU6500_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	
	res=MPU6500_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR_ID)//器件ID正确
	{
		MPU6500_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU6500_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU6500_Set_Rate(1000);						//设置采样率为1000Hz
		MPU6500_Write_Byte(MPU_INT_EN_REG,0X01);	//开启Data ready中断
 	}else 	return 1;
	
	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU6500_Set_Gyro_Fsr(u8 fsr)
{
	return MPU6500_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU6500_Set_Accel_Fsr(u8 fsr)
{
	return MPU6500_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU6500_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU6500_Write_Byte(MPU_CFG_REG,0x01);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU6500_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU6500_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU6500_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU6500_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU6500_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU6500_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU6500_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//ax,ay,az:加速度计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU6500_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU6500_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU6500_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_I2C_Start(); 
	MPU_I2C_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_I2C_Wait_Ack())	//等待应答
	{
		MPU_I2C_Stop();		 
		return 1;		
	}
    MPU_I2C_Send_Byte(reg);	//写寄存器地址
    MPU_I2C_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_I2C_Send_Byte(buf[i]);	//发送数据
		if(MPU_I2C_Wait_Ack())		//等待ACK
		{
			MPU_I2C_Stop();	 
			return 1;		 
		}		
	}    
    MPU_I2C_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU6500_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_I2C_Start(); 
	MPU_I2C_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_I2C_Wait_Ack())	//等待应答
	{
		MPU_I2C_Stop();		 
		return 1;		
	}
    MPU_I2C_Send_Byte(reg);	//写寄存器地址
    MPU_I2C_Wait_Ack();		//等待应答
    MPU_I2C_Start();
	MPU_I2C_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    MPU_I2C_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=MPU_I2C_Read_Byte(0);//读数据,发送nACK 
		else *buf=MPU_I2C_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    MPU_I2C_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU6500_Write_Byte(u8 reg,u8 data) 				 
{ 
    MPU_I2C_Start(); 
	MPU_I2C_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(MPU_I2C_Wait_Ack())	//等待应答
	{
		MPU_I2C_Stop();		 
		return 1;		
	}
    MPU_I2C_Send_Byte(reg);	//写寄存器地址
    MPU_I2C_Wait_Ack();		//等待应答 
	MPU_I2C_Send_Byte(data);//发送数据
	if(MPU_I2C_Wait_Ack())	//等待ACK
	{
		MPU_I2C_Stop();	 
		return 1;		 
	}		 
    MPU_I2C_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU6500_Read_Byte(u8 reg)
{
	u8 res;
    MPU_I2C_Start(); 
	MPU_I2C_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	MPU_I2C_Wait_Ack();		//等待应答 
    MPU_I2C_Send_Byte(reg);	//写寄存器地址
    MPU_I2C_Wait_Ack();		//等待应答
    MPU_I2C_Start();
	MPU_I2C_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    MPU_I2C_Wait_Ack();		//等待应答 
	res=MPU_I2C_Read_Byte(0);//读取数据,发送nACK 
    MPU_I2C_Stop();			//产生一个停止条件 
	return res;		
}

//MPU6050自测试
//返回值:0,正常
//    其他,失败
u8 MPU6500_run_self_test(void)
{
	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3]; 
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x07) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
		float gyro_sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long)(gyro[0] * gyro_sens);
		gyro[1] = (long)(gyro[1] * gyro_sens);
		gyro[2] = (long)(gyro[2] * gyro_sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
				
		return 0;
	}else return 1;
}
//
//
//
//=================================
u8 MPU6500_DMP_Init(void)
{
	struct int_param_s int_param;
	int result;
	
	result=mpu_init(&int_param);
	if(result) return 1;
	
	result=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
	if(result) return 2;
	
	result=mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置FIFO
	if(result) return 3;
	
	result=mpu_set_sample_rate(DEFAULT_MPU_HZ);//设置采样率
	if(result) return 4;
	
	result=dmp_load_motion_driver_firmware();//加载dmp固件
	if(result) return 5;
	
	result=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));//设置陀螺仪方向
	if(result) return 6;
	
	result=dmp_enable_feature(DMP_FEATURE_TAP|DMP_FEATURE_ANDROID_ORIENT|
														DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_GYRO_CAL|
														DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_RAW_GYRO);//设置dmp功能
	if(result) return 7;
	
	result=dmp_set_fifo_rate(DEFAULT_MPU_HZ);//设置DMP输出速率(最大不超过200Hz)
	if(result) return 8;
	
	result=MPU6500_run_self_test();		//自检
	if(result) return 9;
	
	result=mpu_set_dmp_state(1);//使能DMP
	if(result) return 10;
	
	result=dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);//设置中断产生方式
	if(result) return 11;
	
	return 0;
}
//得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
u8 MPU6500_dmp_get_euler_angle(short *accel,short *gyro,float *pitch,float *roll,float *yaw)
{
	
//q30格式,long转float时的除数.
#define Q30  ((1<<30)*1.0f)
	
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short sensors;
	unsigned char more;
	long quat[4]; 
	u8 result=0;
	result=dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);
	if(result)return 1;	 
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if(sensors&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] / Q30;	//q30格式转换为浮点数
		q1 = quat[1] / Q30;
		q2 = quat[2] / Q30;
		q3 = quat[3] / Q30; 
		//计算得到俯仰角/横滚角/航向角
		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	}else return 2;
	return 0;
}

//***********************************************
//函数功能：float数据类型输出，6个
//入口参数：6个float
//返回值：无
//***********************************************
void fData2ASCII6(float f1,float f2,float f3,float f4,float f5,float f6)
{
	u8 t1,t2,t3,t4,t5;
	int i1;

	//f1--------
	if(f1>=0)
	{
		USART1_TX_BUF[0] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[0] = 0x2d;
		f1 = -f1;
	}
	
	i1 = (int)f1;	//取整数部分
	
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[1] = t1 + 0x30;
	USART1_TX_BUF[2] = t2 + 0x30;
	USART1_TX_BUF[3] = t3 + 0x30;
	USART1_TX_BUF[4] = t4 + 0x30;
	USART1_TX_BUF[5] = t5 + 0x30;
	USART1_TX_BUF[6] = ',';
	//f2---------
	if(f2>=0)
	{
		USART1_TX_BUF[7] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[7] = 0x2d;
		f2 = -f2;
	}
	
	i1 = (int)f2;	//取整数部分
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[8] = t1 + 0x30;
	USART1_TX_BUF[9] = t2 + 0x30;
	USART1_TX_BUF[10] = t3 + 0x30;
	USART1_TX_BUF[11] = t4 + 0x30;
	USART1_TX_BUF[12] = t5 + 0x30;
	USART1_TX_BUF[13] = ',';
	//f3---------
	if(f3>=0)
	{
		USART1_TX_BUF[14] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[14] = 0x2d;
		f3 = -f3;
	}
	
	i1 = (int)f3;	//取整数部分
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[15] = t1 + 0x30;
	USART1_TX_BUF[16] = t2 + 0x30;
	USART1_TX_BUF[17] = t3 + 0x30;
	USART1_TX_BUF[18] = t4 + 0x30;
	USART1_TX_BUF[19] = t5 + 0x30;
	USART1_TX_BUF[20] = ',';
	//f4---------
	if(f4>=0)
	{
		USART1_TX_BUF[21] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[21] = 0x2d;
		f4 = -f4;
	}
	
	i1 = (int)f4;	//取整数部分
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[22] = t1 + 0x30;
	USART1_TX_BUF[23] = t2 + 0x30;
	USART1_TX_BUF[24] = t3 + 0x30;
	USART1_TX_BUF[25] = t4 + 0x30;
	USART1_TX_BUF[26] = t5 + 0x30;
	USART1_TX_BUF[27] = ',';
	//f5---------
	if(f5>=0)
	{
		USART1_TX_BUF[28] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[28] = 0x2d;
		f5 = -f5;
	}
	
	i1 = (int)f5;	//取整数部分
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[29] = t1 + 0x30;
	USART1_TX_BUF[30] = t2 + 0x30;
	USART1_TX_BUF[31] = t3 + 0x30;
	USART1_TX_BUF[32] = t4 + 0x30;
	USART1_TX_BUF[33] = t5 + 0x30;
	USART1_TX_BUF[34] = ',';
	//f6---------
	if(f6>=0)
	{
		USART1_TX_BUF[35] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[35] = 0x2d;
		f6 = -f6;
	}
	
	i1 = (int)f6;	//取整数部分
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[36] = t1 + 0x30;
	USART1_TX_BUF[37] = t2 + 0x30;
	USART1_TX_BUF[38] = t3 + 0x30;
	USART1_TX_BUF[39] = t4 + 0x30;
	USART1_TX_BUF[40] = t5 + 0x30;
	USART1_TX_BUF[41] = ',';
	USART1_TX_BUF[42]=0x0d;
	USART1_TX_BUF[43]=0x0a;	

	TxD1pt = 1;
	TxD1Num= 44;
	USART1->DR = USART1_TX_BUF[0];
}

//***********************************************
//函数功能：float数据类型输出，3个
//入口参数：3个float
//返回值：无
//***********************************************
void fData2ASCII(float fx,float fy,float fz)
{
	u8 t1,t2,t3,t4,t5;
	int i1;

	//--------
	if(fx>=0)
	{
		USART1_TX_BUF[0] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[0] = 0x2d;
		fx = -fx;
	}
	
	i1 = (int)fx;	//取整数部分
	
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[1] = t1 + 0x30;
	USART1_TX_BUF[2] = t2 + 0x30;
	USART1_TX_BUF[3] = t3 + 0x30;
	USART1_TX_BUF[4] = t4 + 0x30;
	USART1_TX_BUF[5] = t5 + 0x30;
	USART1_TX_BUF[6] = ',';
	//---------
	if(fy>=0)
	{
		USART1_TX_BUF[7] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[7] = 0x2d;
		fy = -fy;
	}
	
	i1 = (int)fy;	//取整数部分
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[8] = t1 + 0x30;
	USART1_TX_BUF[9] = t2 + 0x30;
	USART1_TX_BUF[10] = t3 + 0x30;
	USART1_TX_BUF[11] = t4 + 0x30;
	USART1_TX_BUF[12] = t5 + 0x30;
	USART1_TX_BUF[13] = ',';
	//------
	if(fz>=0)
	{
		USART1_TX_BUF[14] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[14] = 0x2d;
		fz = -fz;
	}
	
	i1 = (int)fz;	//取整数部分
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[15] = t1 + 0x30;
	USART1_TX_BUF[16] = t2 + 0x30;
	USART1_TX_BUF[17] = t3 + 0x30;
	USART1_TX_BUF[18] = t4 + 0x30;
	USART1_TX_BUF[19] = t5 + 0x30;
	USART1_TX_BUF[20]=0x0d;
	USART1_TX_BUF[21]=0x0a;	

	TxD1pt = 1;
	TxD1Num= 22;
	USART1->DR = USART1_TX_BUF[0];
}
//***********************************************
//函数功能：int 数据类型输出，UART1输出
//入口参数：1个int 
//返回值：无
//***********************************************
void iData2ASCII(int16_t icnt)
{
	u8 t1,t2,t3,t4,t5;
	int i1;

	//--------
	if(icnt>=0)
	{
		USART1_TX_BUF[0] = 0x2b;
	}
	else
	{
		USART1_TX_BUF[0] = 0x2d;
		icnt = -icnt;
	}
	i1 = icnt;	
	t1 = i1/10000;
	t2 = (i1-t1*10000)/1000;
	t3 = (i1-t1*10000-t2*1000)/100;
	t4 = (i1-t1*10000-t2*1000-t3*100)/10;
	t5 = i1-t1*10000-t2*1000-t3*100-t4*10;
	
	USART1_TX_BUF[1] = t1 + 0x30;
	USART1_TX_BUF[2] = t2 + 0x30;
	USART1_TX_BUF[3] = t3 + 0x30;
	USART1_TX_BUF[4] = t4 + 0x30;
	USART1_TX_BUF[5] = t5 + 0x30;

	USART1_TX_BUF[6]=0x0d;
	USART1_TX_BUF[7]=0x0a;	

	TxD1pt = 1;
	TxD1Num= 8;
	USART1->DR = USART1_TX_BUF[0];
}

