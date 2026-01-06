#include "mpuiic.h"
#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////
// I2C2 硬件驱动 (寄存器版)
// PB10 -> SCL (AF4)
// PB11 -> SDA (AF4)
//////////////////////////////////////////////////////////////////////////////////
// I2C2 初始化
void I2C_Hardware_Init(void)
{
    // 1. 开启时钟
    RCC->AHB1ENR |= 1 << 1;     // 使能 GPIOB 时钟
    RCC->APB1ENR |= 1 << 22;    // 使能 I2C2 时钟

    // 2. 配置 GPIO (PB10, PB11) 为复用开漏
    // PB10
    GPIOB->MODER &= ~(3 << (10 * 2));   
    GPIOB->MODER |= (2 << (10 * 2));    // 复用模式 (Mode = 10)
    GPIOB->OTYPER |= (1 << 10);         // 开漏输出 (OT = 1)
    GPIOB->OSPEEDR |= (3 << (10 * 2));  // 高速 (Speed = 11)
    GPIOB->PUPDR &= ~(3 << (10 * 2));
    GPIOB->PUPDR |= (1 << (10 * 2));    // 上拉 (PuPd = 01)
    
    // PB11
    GPIOB->MODER &= ~(3 << (11 * 2));
    GPIOB->MODER |= (2 << (11 * 2));    // 复用模式
    GPIOB->OTYPER |= (1 << 11);         // 开漏
    GPIOB->OSPEEDR |= (3 << (11 * 2));  // 高速
    GPIOB->PUPDR &= ~(3 << (11 * 2));
    GPIOB->PUPDR |= (1 << (11 * 2));    // 上拉

    // 3. 配置复用功能映射 (AF4 for I2C2)
    // AFR[1] 对应 GPIO 8-15
    GPIOB->AFR[1] &= ~(0x0F << ((10 - 8) * 4)); // 清除 PB10 AF
    GPIOB->AFR[1] |= (0x04 << ((10 - 8) * 4));  // PB10 -> AF4
    GPIOB->AFR[1] &= ~(0x0F << ((11 - 8) * 4)); // 清除 PB11 AF
    GPIOB->AFR[1] |= (0x04 << ((11 - 8) * 4));  // PB11 -> AF4

    // 4. I2C2 外设复位
    I2C2->CR1 |= 1 << 15; // SWRST
    delay_ms(2);
    I2C2->CR1 &= ~(1 << 15);

    // 5. 配置 I2C 参数 (标准模式 100kHz)
    // 假设 APB1 (PCLK1) = 42MHz
    I2C2->CR2 &= ~(0x3F);
    I2C2->CR2 |= 42;      // FREQ = 42MHz

    // 开启快速模式 (F/S bit = 1)
    I2C2->CCR |= (1 << 15);   // Fast Mode
    // 选择占空比 (DUTY bit)
    // 0: Duty cycle = 2 (T_low/T_high = 2)
    // 1: Duty cycle = 16/9 (T_low/T_high = 16/9)
    // 建议先用 DUTY=0 (2:1)，比较简单
    I2C2->CCR &= ~(1 << 14);  // DUTY = 0

    // 计算 CCR 值
    // 400kHz -> 周期 2.5us. T_high = 1/3 * 2.5us = 0.833us
    // CCR = 0.833us * 42MHz ≈ 35
    I2C2->CCR &= 0xF000;      // 清除低12位
    I2C2->CCR |= 35;          // 设置 CCR = 35

    // 计算 TRISE
    // Fast Mode 最大上升时间 300ns
    // TRISE = 300ns * 42MHz + 1 ≈ 13
    I2C2->TRISE = 13;

    // 6. 使能 I2C2
    I2C2->CR1 |= 1 << 0;  // PE Enable
    
    // 允许 ACK
    I2C2->CR1 |= 1 << 10; 
}

// 内部函数：等待标志位
// flag: 状态位掩码
// status: 1等待置位, 0等待清零
static u8 I2C_Wait_Flag(u16 flag, u8 status)
{
    u32 timeout = 0x5000;
    if(status)
    {
        while(!((I2C2->SR1) & flag))
        {
            if(--timeout == 0) return 1; // 超时
        }
    }
    else
    {
        while(((I2C2->SR1) & flag))
        {
            if(--timeout == 0) return 1;
        }
    }
    return 0;
}

// 写一个寄存器
// addr: 设备地址 (7位，无需左移，内部会处理)
// reg:  寄存器地址
// data: 数据
u8 I2C_Write_Reg(u8 addr, u8 reg, u8 data)
{
    u32 timeout = 0;
    
    // 1. 发送起始信号
    I2C2->CR1 |= 1 << 8; // START
    if(I2C_Wait_Flag(0x0001, 1)) return 1; // 等待 SB (Start Bit)

    // 2. 发送设备地址 (写)
    I2C2->DR = (addr << 1) | 0;
    if(I2C_Wait_Flag(0x0002, 1)) return 2; // 等待 ADDR (Address Sent)
    (void)I2C2->SR2; // 读取 SR2 清除 ADDR 标志

    // 3. 发送寄存器地址
    I2C2->DR = reg;
    if(I2C_Wait_Flag(0x0080, 1)) return 3; // 等待 TXE (Tx Empty)

    // 4. 发送数据
    I2C2->DR = data;
    if(I2C_Wait_Flag(0x0004, 1)) return 4; // 等待 BTF (Byte Transfer Finished)

    // 5. 停止信号
    I2C2->CR1 |= 1 << 9; // STOP

    return 0; // 成功
}

// 读取连续数据
// addr: 设备地址
// reg: 起始寄存器
// buf: 数据缓冲
// len: 长度
u8 I2C_Read_Regs(u8 addr, u8 reg, u8 *buf, u8 len)
{
    if(len == 0) return 0;
    
    // 1. 写寄存器地址阶段
    I2C2->CR1 |= 1 << 8; // START
    if(I2C_Wait_Flag(0x0001, 1)) return 1;

    I2C2->DR = (addr << 1) | 0; // Write Mode
    if(I2C_Wait_Flag(0x0002, 1)) return 2; 
    (void)I2C2->SR2;

    I2C2->DR = reg;
    if(I2C_Wait_Flag(0x0004, 1)) return 3; // Wait BTF to ensure reg sent

    // 2. 重复起始信号 (Restart) 读取数据
    I2C2->CR1 |= 1 << 8; // RE-START
    if(I2C_Wait_Flag(0x0001, 1)) return 4;

    I2C2->DR = (addr << 1) | 1; // Read Mode
    if(I2C_Wait_Flag(0x0002, 1)) return 5;
    
    // 此时 ADDR 已置位，准备开始接收
    if (len == 1)
    {
        I2C2->CR1 &= ~(1 << 10); // Clear ACK
        (void)I2C2->SR2;         // Clear ADDR
        I2C2->CR1 |= 1 << 9;     // STOP
        
        if(I2C_Wait_Flag(0x0040, 1)) return 6; // Wait RxNE
        buf[0] = I2C2->DR;
    }
    else
    {
        I2C2->CR1 |= 1 << 10;    // Enable ACK
        (void)I2C2->SR2;         // Clear ADDR
        
        for (int i = 0; i < len; i++)
        {
            // 如果是倒数第二个字节，准备结束
            if (i == len - 1)
            {
                I2C2->CR1 &= ~(1 << 10); // NACK
                I2C2->CR1 |= 1 << 9;     // STOP
            }
            
            if(I2C_Wait_Flag(0x0040, 1)) return 7; // Wait RxNE
            buf[i] = I2C2->DR;
        }
    }
    
    // 恢复 ACK 以备下次使用
    I2C2->CR1 |= 1 << 10; 
    return 0;
}

// 读单个寄存器辅助函数
u8 I2C_Read_Reg(u8 addr, u8 reg)
{
    u8 data;
    I2C_Read_Regs(addr, reg, &data, 1);
    return data;
}



