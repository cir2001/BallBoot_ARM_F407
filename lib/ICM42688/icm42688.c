#include "icm42688.h"
#include "spi.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
//                          
//////////////////////////////////////////////////////////////////////////////////
void ICM_WriteReg(uint8_t reg, uint8_t data) 
{
    GPIOA->BSRR = (1 << (4 + 16)); // CS Low (PA4)
    SPI1_ReadWriteByte(reg);       // 写入寄存器地址 (Write模式 MSB=0)
    SPI1_ReadWriteByte(data);
    GPIOA->BSRR = (1 << 4);        // CS High
}

uint8_t ICM_ReadReg(uint8_t reg) 
{
    uint8_t data;
    GPIOA->BSRR = (1 << (4 + 16)); // CS Low
    SPI1_ReadWriteByte(reg | 0x80); // 读模式 (MSB=1)
    data = SPI1_ReadWriteByte(0xFF); // 写入伪数据读取
    GPIOA->BSRR = (1 << 4);        // CS High
    return data;
}

uint8_t ICM42688_Init(void) 
{
    SPI1_SetSpeed(4); // 设置为 32分频 (2.625MHz)，适合您的绞线环境
    // 1. 软件复位
    ICM_WriteReg(ICM_REG_DEVICE_CONFIG, 0x01);
    delay_ms(100); // 必须给足复位时间

    // 将 SPI 驱动斜率放慢到 6ns-18ns，减少绞线产生的反射干扰
    ICM_WriteReg(ICM_REG_DRIVE_CONFIG, 0x02);
    
    // 2. 检查 ID
    if (ICM_ReadReg(ICM_REG_WHO_AM_I) != 0x47) return 0;

    // 3. 确保当前在 Bank 0 (虽然默认是0，但在寄存器开发中显式切换更保险)
    ICM_WriteReg(ICM_REG_BANK_SEL, 0x00);

    // 4. 配置中断引脚 (INT1)
    // 推荐：推挽输出, 低电平有效, 脉冲模式
    ICM_WriteReg(ICM_REG_INT_CONFIG, (1 << 1) | (0 << 2)); 

    // 5. 映射中断源到 INT1
    // 开启数据就绪中断 (UI_DRDY)
    ICM_WriteReg(ICM_REG_INT_SOURCE0, (1 << 3));

    // 6. 配置陀螺仪与加速度计参数 (ODR 和 量程)
    // 陀螺仪: 2000dps (000), 1kHz ODR (0110) -> 0x06
    ICM_WriteReg(ICM_REG_GYRO_CONFIG0, (0 << 5) | 0x06); 
    
    // 加速度计: 16g (000), 1kHz ODR (0110) -> 0x06
    // 加速度计: 2g (011), 1kHz ODR (0110) -> 0x06
    ICM_WriteReg(ICM_REG_ACCEL_CONFIG0, (3 << 5) | 0x06);

    // 7. 最后一步：开启电源 (进入低噪声模式)
    // 设置为 0x0F: 开启加速度计(LN)和陀螺仪(LN)
    ICM_WriteReg(ICM_REG_PWR_MGMT_0, 0x0F);

    // 8. 等待传感器电路和数字滤波器稳定
    delay_ms(100); // 等待传感器稳定
    
    return 1; // 初始化成功
}

void ICM42688_ReadData(ICM_Data* data) 
{
    uint8_t buf[12];
    
    GPIOA->BSRR = (1 << (4 + 16)); // CS Low
    //SPI1_CS_LOW();
    SPI1_ReadWriteByte(ICM_REG_ACCEL_DATA_X1 | 0x80); // 起始地址 + 读位
    
    for(int i=0; i<12; i++) {
        buf[i] = SPI1_ReadWriteByte(0xFF);
    }

    while(SPI1->SR & SPI_SR_BSY); // 等待总线空闲
    GPIOA->BSRR = (1 << 4);        // CS High

    // 组合高低字节 (大端序转换)
    data->acc_x  = (int16_t)((buf[0] << 8) | buf[1]);
    data->acc_y  = (int16_t)((buf[2] << 8) | buf[3]);
    data->acc_z  = (int16_t)((buf[4] << 8) | buf[5]);
    data->gyro_x = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_y = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_z = (int16_t)((buf[10] << 8) | buf[11]);
}
