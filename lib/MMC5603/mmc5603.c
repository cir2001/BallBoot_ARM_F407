#include "mmc5603.h" 
#include "mpuiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//	STM32F407开发板 LED0 LED1配置						  
////////////////////////////////////////////////////////////////////////////////// 	 
/**
 * @brief MMC5603 初始化
 * @return 1: 成功, 0: 失败
 */
u8 MMC5603_Init(void)
{
    u8 res;

    I2C_Init();
    delay_ms(10);

    // 1. 软件复位（必须，清空寄存器）
    I2C_Write_Byte(MMC5603_REG_CTRL1, 0x80);
    delay_ms(50);  // 官方要求>20ms

    // 2. （可选）检查ID，现在应该是0x40
    // res = MMC5603_Read_Reg(MMC5603_REG_ID);
    // if (res != 0x40) return 0;  // 可打开验证

    // 3. 设置最高带宽（1.2ms，支持高ODR）
    I2C_Write_Byte(MMC5603_REG_CTRL1, 0x03);  // BW=11

    // 4. 开启自动SET/RESET（强烈推荐！消除漂移）
    I2C_Write_Byte(MMC5603_REG_CTRL0, 0x20);  // Auto_SR_en = 1 (bit5)

    // 5. 设置ODR ≈100Hz（1~255，推荐50~200）
    I2C_Write_Byte(MMC5603_REG_ODR, 100);

    // 6. 触发测量频率计算（必须，自清零）
    I2C_Write_Byte(MMC5603_REG_CTRL0, 0x80);  // Cmm_freq_en = 1 (bit7)
    delay_ms(2);

    // 7. 开启连续测量模式（关键！bit4=1）
    I2C_Write_Byte(MMC5603_REG_CTRL2, 0x10);  // Cmm_en = 1

    // 8. 可选：开启周期性SET（进一步减漂移）
    // I2C_Write_Byte(MMC5603_REG_CTRL2, 0x18);  // Cmm_en + En_prd_set (bit4+bit3)

    // 9. 执行一次手动SET（消除初始磁滞，很多实测必须加这步）
    I2C_Write_Byte(MMC5603_REG_CTRL0, 0x08);  // Do_Set = 1 (bit3，自清零)
    delay_ms(5);

    delay_ms(20);  // 等待首次数据就绪

    return 1;
}
/**
 * @brief MMC5603 连续读取 9 字节数据并转换为物理单位
 * @note 采用 Burst Read 模式确保 20位数据的一致性
 */
void MMC5603_ReadData(float *mx, float *my, float *mz)
{
    uint8_t buf[9];
    uint32_t raw_x, raw_y, raw_z;

    // 关键：等待数据就绪（Meas_m_done，通常bit6=0x40，或试bit0=0x01）
    uint8_t status;
    uint16_t timeout = 1000;
    do {
        status = MMC5603_Read_Reg(MMC5603_REG_STATUS);
        timeout--;
        if (timeout == 0) return;  // 超时保护
    } while (!(status & 0x40));  // 如果bit6无效，试 0x01 或 0x02

    // 连续读取9字节（你的代码完美）
    I2C_Start();
    I2C_Send_Byte((MMC5603_ADDR << 1) | 0);
    if (I2C_Wait_Ack()) { I2C_Stop(); return; }
    I2C_Send_Byte(MMC5603_REG_DATA);
    if (I2C_Wait_Ack()) { I2C_Stop(); return; }
    I2C_Start();
    I2C_Send_Byte((MMC5603_ADDR << 1) | 1);
    if (I2C_Wait_Ack()) { I2C_Stop(); return; }

    for (int i = 0; i < 8; i++) buf[i] = I2C_Read_Byte(1);
    buf[8] = I2C_Read_Byte(0);
    I2C_Stop();

    // 数据组合（正确）
    raw_x = (uint32_t)buf[0] << 12 | (uint32_t)buf[1] << 4 | (buf[6] >> 4);
    raw_y = (uint32_t)buf[2] << 12 | (uint32_t)buf[3] << 4 | (buf[7] >> 4);
    raw_z = (uint32_t)buf[4] << 12 | (uint32_t)buf[5] << 4 | (buf[8] >> 4);

    // 转换为Gauss
    *mx = ((float)raw_x - 524288.0f) / 16384.0f;
    *my = ((float)raw_y - 524288.0f) / 16384.0f;
    *mz = ((float)raw_z - 524288.0f) / 16384.0f;
}
//----------------------------------------------------------
// I2C写一个字节 
// reg:寄存器地址
// data:数据
// 返回值:0,正常
//    其他,错误代码
//----------------------------------------------------------
u8 I2C_Write_Byte(u8 reg,u8 data) 				 
{ 
    I2C_Start(); 
	I2C_Send_Byte((MMC5603_ADDR<<1)|0);//发送器件地址+写命令	
	if(I2C_Wait_Ack())	//等待应答
	{
		I2C_Stop();		 
		return 1;		
	}
    I2C_Send_Byte(reg);	//写寄存器地址
    I2C_Wait_Ack();		//等待应答 
	I2C_Send_Byte(data);//发送数据
	if(I2C_Wait_Ack())	//等待ACK
	{
		I2C_Stop();	 
		return 1;		 
	}		 
    I2C_Stop();	 
	return 0;
}
//------------------------------------------------
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
//------------------------------------------------
u8 MMC5603_Read_Reg(u8 reg)
{
	u8 res = 0;
    I2C_Start(); 
	I2C_Send_Byte((MMC5603_ADDR<<1)|0);//发送器件地址+写命令	
	I2C_Wait_Ack();		//等待应答 
    I2C_Send_Byte(reg);	//写寄存器地址
    I2C_Wait_Ack();		//等待应答
    I2C_Start();
	I2C_Send_Byte((MMC5603_ADDR<<1)|1);//发送器件地址+读命令	
    I2C_Wait_Ack();		//等待应答 
	res=I2C_Read_Byte(0);//读取数据,发送nACK 
    I2C_Stop();			//产生一个停止条件 
	return res;		
}

