#include "control.h"
#include <stm32f10x.h>

#include "usart.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "timer.h"
/************************************************************************************


*************************************************************************************/
u16 u16Exti0Count;

#define MotorA_PWM_Scale 7000
#define MotorB_PWM_Scale 7000
#define MotorC_PWM_Scale 7000

//#define MOTOR_OUT_MAX 6000
//#define MOTOR_OUT_MIN	-6000

#define BOT_SPEED_SET 0
#define BOT_POSITION_SET 0

//#define BOT_POSITION_MAX 5000
//#define BOT_POSITION_MIN -5000
//---------------------------------------------
float BOT_POSITION_MAX=500;
float BOT_POSITION_MIN=-500;

u8  u8UartSendCount;
u8  u8RotationCount;


u16 u16_Delayms,u16CtriCount;

int iMotorA_Encoder,iMotorB_Encoder,iMotorC_Encoder;
int iMotorA_Err,iMotorB_Err,iMotorC_Err;
int iMotorA_Integral,iMotorB_Integral,iMotorC_Integral;
int iMotorA_Err_Last,iMotorB_Err_Last,iMotorC_Err_Last;

long int iMotorAPulseTotle,iMotorBPulseTotle,iMotorCPulseTotle;

int iTIM2Temp,iTIM3Temp,iTIM4Temp;

int iPitch_Bias,iRoll_Bias,iYaw_Bias;

float fPitch_Bias,fRoll_Bias,fYaw_Bias;

float fPitch_Set=0;
float fRoll_Set=0;
float fYaw_Set=0;

int MOTOR_OUT_MAX =  6000;
int MOTOR_OUT_MIN = -6000;

float fMotor_A_Ctrl,fMotor_B_Ctrl,fMotor_C_Ctrl;

float fMotor_A_Cal,fMotor_B_Cal,fMotor_C_Cal;
float fMotor_A_PWM,fMotor_B_PWM,fMotor_C_PWM;

float fAngle_X_Ctrl,fAngle_Y_Ctrl,fAngle_Z_Ctrl;

float fAngle_Pitch_Integral,fAngle_Roll_Integral,fAngle_Yaw_Integral;
float	fAngle_Pitch_Err,fAngle_Roll_Err,fAngle_Yaw_Err;


float fSpeedVx=0,fSpeedVy=0,fSpeedVz=0;
float fSpeed_X_Ctrl=0,fSpeed_Y_Ctrl=0,fSpeed_Z_Ctrl=0;
float fSpeedOldX=0,fPositionX=0;
float fSpeedOldY=0,fPositionY=0;
float fSpeedOldZ=0,fPositionZ=0;

float fMove_X_Out=0,fMove_Y_Out=0,fMove_Z_Out=0;
//-------------------------------
extern short gyro[3], accel[3], sensors;
extern float fAngle_Pitch,fAngle_Roll,fAngle_Yaw;
extern s16 s16MotorA_Set_Input;

extern float fGyro_X,fGyro_Y,fGyro_Z;

extern float fGyro_Pitch,fGyro_Roll,fGyro_Yaw;

extern u8 u8RecBack;
extern u8 u8Key1_State;

extern void SetMotorPWM(float MotorAPWM,float MotorBPWM,float MotorCPWM);

extern u8 u8Move_Front,u8Move_Back,u8Move_Right,u8Move_Left,u8Move_Pos,u8Move_Neg,u8Move_Zero;
//===============================


//==================================================================
//外部中断0服务程序
void EXTI0_IRQHandler(void)
{
	EXTI->PR=1<<0;  //清除LINE0上的中断标志位  
	u16Exti0Count++;
	if(u16Exti0Count>=50)
	{
		u16Exti0Count = 0;
		LED1 = !LED1;
	}
	if((INT == 0) && (u8Key1_State == 1))
	{
		u16CtriCount = 0;
		u8UartSendCount++;
		//-- 读取编码器 ------
		iTIM2Temp = (short)TIM2->CNT;
		iTIM3Temp = (short)TIM3->CNT;
		iTIM4Temp = (short)TIM4->CNT;

		TIM2->CNT = 0; //计数清零 TIM2->CNT = 0
		TIM3->CNT = 0; //计数清零 TIM3->CNT = 0
		TIM4->CNT = 0; //计数清零 TIM4->CNT = 0
		
		iMotorA_Encoder = iTIM4Temp;
		iMotorB_Encoder = iTIM3Temp;
		iMotorC_Encoder = iTIM2Temp;
					
		iMotorAPulseTotle += iMotorA_Encoder;	//速度累加
		iMotorBPulseTotle += iMotorB_Encoder;
		iMotorCPulseTotle += iMotorC_Encoder;	
		
		Forward_Kinematics(iMotorA_Encoder,iMotorB_Encoder,iMotorC_Encoder);
		//Forward_Kinematics(-20,-20,-20);
		Read_DMP();
		
		Control_Calculate();	
		Inverse_Kinematics(fMove_X_Out,fMove_Y_Out,fMove_Z_Out);//坐标转换 逆运动学
		//Inverse_Kinematics(0,0,200);//坐标转换 逆运动学
		
		/*fMotor_A_Ctrl = (float)s16MotorA_Set_Input/7;	//转速测试
		fMotor_B_Ctrl = (float)s16MotorA_Set_Input/7;
		fMotor_C_Ctrl = (float)s16MotorA_Set_Input/7;
		
		fMotor_A_Ctrl = -5;	//转速测试
		fMotor_B_Ctrl = -5;
		fMotor_C_Ctrl = -5;
		
		fMotor_A_PWM = 100;
		fMotor_B_PWM = 100;
		fMotor_C_PWM = 100;*/
		
		
		fMotor_A_Ctrl = fMotor_A_Cal/7;
		fMotor_B_Ctrl = fMotor_B_Cal/7;
		fMotor_C_Ctrl = fMotor_C_Cal/7;
		
		
		iMotorA_Err =	 (int)fMotor_A_Ctrl - iMotorA_Encoder;
		iMotorB_Err =  (int)fMotor_B_Ctrl - iMotorB_Encoder;
		iMotorC_Err =  (int)fMotor_C_Ctrl - iMotorC_Encoder;

		iMotorA_Integral += iMotorA_Err;
		iMotorB_Integral += iMotorB_Err;
		iMotorC_Integral += iMotorC_Err;
		
		fMotor_A_PWM = -(iMotorA_Err * 1000.0f + (iMotorA_Err - iMotorA_Err_Last) * 100.0f + (iMotorA_Integral) * 20.0f);
		fMotor_B_PWM = -(iMotorB_Err * 1000.0f + (iMotorB_Err - iMotorB_Err_Last) * 100.0f + (iMotorB_Integral) * 20.0f);
		fMotor_C_PWM = -(iMotorC_Err * 1000.0f + (iMotorC_Err - iMotorC_Err_Last) * 100.0f + (iMotorC_Integral) * 20.0f);
	
		
		iMotorA_Err_Last = iMotorA_Err;
		iMotorB_Err_Last = iMotorB_Err;
		iMotorC_Err_Last = iMotorC_Err;
		
		//--- 限幅 -----
		if((int)fMotor_A_PWM > MOTOR_OUT_MAX) fMotor_A_PWM = MOTOR_OUT_MAX;
		if((int)fMotor_A_PWM < MOTOR_OUT_MIN) fMotor_A_PWM = MOTOR_OUT_MIN;

		if((int)fMotor_B_PWM > MOTOR_OUT_MAX) fMotor_B_PWM = MOTOR_OUT_MAX;
		if((int)fMotor_B_PWM < MOTOR_OUT_MIN) fMotor_B_PWM = MOTOR_OUT_MIN;

		if((int)fMotor_C_PWM > MOTOR_OUT_MAX) fMotor_C_PWM = MOTOR_OUT_MAX;
		if((int)fMotor_C_PWM < MOTOR_OUT_MIN) fMotor_C_PWM = MOTOR_OUT_MIN;
		//======  电机PWM输出 =========
		SetMotorPWM(fMotor_A_PWM,fMotor_B_PWM,fMotor_C_PWM);	

		if(u8UartSendCount>=10)
		{
			switch(u8RecBack)	{
				case 0:
					fData2ASCII9(fAngle_Pitch,fAngle_Roll,fAngle_Yaw,fMotor_A_PWM,fMotor_B_PWM,fMotor_C_PWM,iMotorA_Encoder,iMotorB_Encoder,iMotorC_Encoder);
					//fData2ASCII9(fAngle_Pitch,fAngle_Roll,fAngle_Pitch_Err,fAngle_Roll_Err,fAngle_Pitch_Integral,fAngle_Roll_Integral,fAngle_X_Ctrl,fAngle_Y_Ctrl,fMotor_A_Ctrl);
					//fData2ASCII(iMotorA_Encoder,-fMotor_A_PWM,iMotorA_Err); // motor-A
					//fData2ASCII(iMotorB_Encoder,-fMotor_B_PWM,iMotorB_Err);	// motor-B
					//fData2ASCII(iMotorC_Encoder,-fMotor_C_PWM,iMotorC_Err);	// motor-C
				break;
				case 1:
					fData2ASCII9(fAngle_X_Ctrl,fAngle_Y_Ctrl,fAngle_Z_Ctrl,fSpeed_X_Ctrl,fSpeed_Y_Ctrl,fSpeed_Z_Ctrl,fMove_X_Out,fMove_Y_Out,fMove_Z_Out);
				break;
				case 2:
					fData2ASCII9(fSpeedOldX,fSpeedOldY,fSpeedOldZ,fPositionX,fPositionY,fPositionZ,iMotorA_Encoder,iMotorB_Encoder,iMotorC_Encoder);
					//fData2ASCII9(fAngle_Pitch,fGyro_Y,fAngle_X_Ctrl,fSpeedOldX,fPositionX,fSpeed_X_Ctrl,fMove_X_Out,iMotorB_Encoder,iMotorC_Encoder);
				break;
				case 3:
					fData2ASCII9(fSpeedVx,fSpeedVy,fSpeedVz,fMotor_A_Ctrl,fMotor_B_Ctrl,fMotor_C_Ctrl,fMotor_A_PWM,fMotor_B_PWM,fMotor_C_PWM);
				break;
				case 4:
					fData2ASCII9(fAngle_Pitch,fAngle_Roll,fAngle_Yaw,fMotor_A_PWM,fMotor_B_PWM,fMotor_C_PWM,iMotorA_Encoder,iMotorB_Encoder,iMotorC_Encoder);
				break;
				default:
					fData2ASCII9(fAngle_Pitch,fAngle_Roll,fAngle_Yaw,fGyro_X,fGyro_Y,fGyro_Z,iMotorA_Encoder,iMotorB_Encoder,iMotorC_Encoder);
				break;}
			u8UartSendCount = 0;
		}
	}

}
//****************************************************
//函数功能：正运动学计算-坐标变换
//入口函数：a,b,c
//返回值：无
//****************************************************
void Forward_Kinematics(float a,float b,float c)
{
	fSpeedVy = (2.0f * a - b - c)/3;
	fSpeedVx = ((c-b) * sqrt(3))/3;
	fSpeedVz = (a + b + c)/3;
}
//****************************************************
//函数功能：逆运动学计算-坐标变换
//入口函数：x,y,z
//返回值：无
//****************************************************
void Inverse_Kinematics(float x,float y,float z)
{
	fMotor_A_Cal = (y + 1.0f * z)*1.14;
	fMotor_B_Cal = (-(0.5f) * y - (sqrt(3)/2.0f) * x + 1.0f * z)*1.14;
	fMotor_C_Cal = (-(0.5f) * y + (sqrt(3)/2.0f) * x + 1.0f * z)*1.14;
}
//****************************************************
//函数功能：控制律解算
//入口函数：void
//返回值：无
//****************************************************
void Control_Calculate(void)
{
		/*fAngle_X_Ctrl = (fAngle_Pitch+0.05f)* 14.0f + fGyro_Y * 0.001f;
		fAngle_Y_Ctrl = (fAngle_Roll-0.05f) * 14.0f + fGyro_X * 0.001f;
		//fAngle_Z_Ctrl = (float)fSpeedVz * 300.0f + fGyro_Z * 0.2f;
		fAngle_Z_Ctrl=0;*/
		//-------------------
		fAngle_Pitch_Err = -0.75f - fAngle_Pitch;
		fAngle_Roll_Err  = -0.75f - fAngle_Roll;
	
		//fAngle_Yaw_Err = 0.0f - fAngle_Yaw;
		if(u8Move_Pos == 1)
		{
			u8RotationCount++;
			if(u8RotationCount == 50)
			{
				u8RotationCount = 0;
				fYaw_Set += 5.0f;
			}
		}
		else if(u8Move_Neg == 1)
		{
			u8RotationCount++;
			if(u8RotationCount == 50)
			{
				u8RotationCount = 0;
				fYaw_Set -= 5.0f;
			}
		}
		else if((u8Move_Zero == 1) && (fYaw_Set != 0.0f))
		{
			if(fYaw_Set>0) fYaw_Set -= 0.1f;
			if(fYaw_Set<0) fYaw_Set += 0.1f;
		}
		fAngle_Yaw_Err = fYaw_Set - fAngle_Yaw;
		
		fAngle_Pitch_Integral += fAngle_Pitch_Err;
		fAngle_Roll_Integral += fAngle_Roll_Err;
		//fAngle_Yaw_Integral += fAngle_Yaw_Err;
	
		fAngle_X_Ctrl = -(fAngle_Pitch_Err * 20.0f + fAngle_Pitch_Integral * 0.46f);
		fAngle_Y_Ctrl = -(fAngle_Roll_Err * 20.0f + fAngle_Roll_Integral * 0.46f);
		//fAngle_Z_Ctrl = 0;
		fAngle_Z_Ctrl = -(fAngle_Yaw_Err * 2.0f + fGyro_Z * 0.002f);;
	
		//---speed_X------
		fSpeedOldX *= 0.95f;						//一阶低通滤波器
		fSpeedOldX += fSpeedVx * 0.05f;	//一阶低通滤波器
		
		fPositionX += fSpeedOldX;  //路程 速度的积分
		
		//积分上限
		if(fPositionX > BOT_POSITION_MAX)		fPositionX = BOT_POSITION_MAX;
		if(fPositionX < BOT_POSITION_MIN)		fPositionX = BOT_POSITION_MIN;
		
		fSpeed_X_Ctrl = fSpeedOldX * 0.3f + fPositionX * 0.0015f;			
		//---speed_Y-------------
		fSpeedOldY *= 0.95f;						//一阶低通滤波器
		fSpeedOldY += fSpeedVy * 0.05f;	//一阶低通滤波器
		fPositionY += fSpeedOldY;  //路程 速度的积分
		//积分上限
		if(fPositionY > BOT_POSITION_MAX)		fPositionY = BOT_POSITION_MAX;
		if(fPositionY < BOT_POSITION_MIN)		fPositionY = BOT_POSITION_MIN;
		
		fSpeed_Y_Ctrl = fSpeedOldY * 0.3f + fPositionY * 0.0015f;
		//--speed_Z-----------
		fSpeedOldZ *= 0.95f;						//一阶低通滤波器
		fSpeedOldZ += fSpeedVz * 0.05f;	//一阶低通滤波器
		
		fPositionZ += fSpeedOldZ;  //路程 速度的积分
		
		//积分上限
		if(fPositionZ > BOT_POSITION_MAX)		fPositionZ = BOT_POSITION_MAX;
		if(fPositionZ < BOT_POSITION_MIN)		fPositionZ = BOT_POSITION_MIN;
		fSpeed_Z_Ctrl = 0;		
		
		//====== 角度 + 速度 =========
		fMove_X_Out = fAngle_X_Ctrl + fSpeed_X_Ctrl;
		fMove_Y_Out = fAngle_Y_Ctrl + fSpeed_Y_Ctrl;
		fMove_Z_Out = fAngle_Z_Ctrl + fSpeed_Z_Ctrl;
		//======= 角度 ==========
		/*fMove_X_Out = fAngle_X_Ctrl;
		fMove_Y_Out = fAngle_Y_Ctrl;
		fMove_Z_Out = fAngle_Z_Ctrl;*/

}
