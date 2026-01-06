#include "mmc5603.h" 
#include "mpuiic.h"
#include "delay.h"

// 替换原有的 I2C_Write_Byte 定义，改用 mpuiic 里的硬件函数
// 这样整个文件的逻辑就统一了
#define MMC_Write(reg, data) I2C_Write_Reg(MMC5603_ADDR, reg, data)
#define MMC_Read(reg)        I2C_Read_Reg(MMC5603_ADDR, reg)

/**
 * @brief MMC5603 初始化
 * @return 1: 成功, 0: 失败
 */
u8 MMC5603_Init(void)
{
    // 硬件I2C初始化 (PB10, PB11)
    I2C_Hardware_Init();
    delay_ms(10);

    // 1. 软件复位
    MMC_Write(MMC5603_REG_CTRL1, 0x80);
    delay_ms(50); 

    // 2. 检查ID (推荐打开)
    u8 id = MMC_Read(MMC5603_REG_ID);
    if (id != 0x10) // 注意: 许多MMC5603 datasheet说ID是0x10，有些批次不同，请确认你的芯片手册
    {
        return 0; // 调试时可先注释掉
    }

    // 3. 设置最高带宽
    MMC_Write(MMC5603_REG_CTRL1, 0x03); 

    // 4. 开启自动SET/RESET
    MMC_Write(MMC5603_REG_CTRL0, 0x20);

    // 5. 设置ODR
    MMC_Write(MMC5603_REG_ODR, 100);

    // 6. 触发测量频率计算
    MMC_Write(MMC5603_REG_CTRL0, 0x80);
    delay_ms(2);

    // 7. 开启连续测量模式
    MMC_Write(MMC5603_REG_CTRL2, 0x10);

    // 9. 执行一次手动SET
    MMC_Write(MMC5603_REG_CTRL0, 0x08);
    delay_ms(5);

    delay_ms(20); 
    return 1;
}

/**
 * @brief MMC5603 连续读取 9 字节数据并转换为物理单位
 */
void MMC5603_ReadData(float *mx, float *my, float *mz)
{
    uint8_t buf[9];
    uint32_t raw_x, raw_y, raw_z;
    uint8_t status;

    // 1. 读取状态寄存器 (只读一次，绝不循环！)
    status = MMC_Read(MMC5603_REG_STATUS);

    // 2. 检查数据是否就绪 (Bit 6: Meas_m_done)
    if (!(status & 0x40)) 
    {
        // 关键修改：如果没准备好，直接返回！不要等！
        // 保持 mx, my, mz 为上一次的值 (或者全局变量里的旧值)
        return; 
    }

    // 3. 数据已就绪，硬件I2C连续读取
    if (I2C_Read_Regs(MMC5603_ADDR, MMC5603_REG_DATA, buf, 9) != 0)
    {
        return; // 读取失败
    }

    // 4. 数据拼接与转换
    raw_x = (uint32_t)buf[0] << 12 | (uint32_t)buf[1] << 4 | (buf[6] >> 4);
    raw_y = (uint32_t)buf[2] << 12 | (uint32_t)buf[3] << 4 | (buf[7] >> 4);
    raw_z = (uint32_t)buf[4] << 12 | (uint32_t)buf[5] << 4 | (buf[8] >> 4);

    // 转换公式 (根据手册: Output / 16384 = Gauss)
    // 减去 524288 是因为 20位无符号数的中点是 2^19
    *mx = ((float)raw_x - 524288.0f) / 16384.0f;
    *my = ((float)raw_y - 524288.0f) / 16384.0f;
    *mz = ((float)raw_z - 524288.0f) / 16384.0f;
}



