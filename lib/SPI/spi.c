#include "spi.h"
//////////////////////////////////////////////////////////////////////////////////	 
//							  
////////////////////////////////////////////////////////////////////////////////// 	 
//以下是SPI模块的初始化代码，配置成主机模式 						  
//SPI口初始化
//这里针是对SPI2的初始化
void SPI2_Init(void)
{	 
	u16 tempreg = 0;
    // 1. 时钟使能
    RCC->AHB1ENR |= 1 << 1;     // 使能 PORTB 时钟
    RCC->APB1ENR |= 1 << 14;    // 使能 SPI2 时钟 (注意：SPI2 在 APB1，最高 42MHz)

    // 2. GPIO 复用配置 (SCK, MISO, MOSI)
    GPIO_Set(GPIOB, PIN13|PIN15, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU);
    GPIO_AF_Set(GPIOB, 13, 5);  // PB13, AF5 (SPI2_SCK)
    GPIO_AF_Set(GPIOB, 15, 5);  // PB15, AF5 (SPI2_MOSI)

    // 3. 复位 SPI2
    RCC->APB1RSTR |= 1 << 14;   // 复位 SPI2
    RCC->APB1RSTR &= ~(1 << 14);// 停止复位 SPI2

    // 4. 配置 SPI2 (不包含使能位 SPE)
    tempreg |= 0 << 10;         // RXONLY = 0: 全双工模式
    tempreg |= 1 << 9;          // SSM = 1: 软件 NSS 管理
    tempreg |= 1 << 8;          // SSI = 1: 内部 NSS 信号置高 (Master 必须)
    tempreg |= 1 << 2;          // MSTR = 1: 主机模式
    tempreg |= 0 << 11;         // DFF = 0: 8位数据格式
    tempreg |= 0 << 1;          // CPOL = 1: 空闲模式下 SCK 为高电平
    tempreg |= 0 << 0;          // CPHA = 1: 数据采样从第2个时间边沿开始
    
    // 波特率控制
    // F407 APB1 时钟一般为 42MHz。
    // 7<<3 (div256) -> 42M/256 ≈ 164 KHz (非常慢，适合调试，实际使用可能需要提高速度)
    tempreg |= 3 << 3;          // BaudRate = fPCLK / 256
    
    tempreg |= 0 << 7;          // LSBFIRST = 0: MSB First

    // 5. 写入配置
    SPI2->CR1 = tempreg;        // 设置 CR1 (此时 SPE 为 0，SPI 尚未启动)
    SPI2->I2SCFGR &= ~(1 << 11);// 确认选择 SPI 模式 (非 I2S)

    // 6. 使能 SPI
    SPI2->CR1 |= 1 << 6;        // SPE = 1: 开启 SPI

    // 7. 启动传输 (发送 Dummy Byte 建立初始状态)
    SPI2_ReadWriteByte(0xff);	 
}   
//SPI1速度设置函数
//SpeedSet:0~7
//SPI速度=fAPB2/2^(SpeedSet+1)
//fAPB2时钟一般为84Mhz
void SPI2_SetSpeed(u8 SpeedSet)
{
	SpeedSet&=0X07;			//限制范围
	SPI2->CR1&=0XFFC7; 
	SPI2->CR1|=SpeedSet<<3;	//设置SPI1速度  
	SPI2->CR1|=1<<6; 		//SPI设备使能	  
} 
//SPI2 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{		 			 
	// 等待发送缓冲区空
    while((SPI2->SR & (1<<1)) == 0); 
    SPI2->DR = TxData;
    
    // 等待接收缓冲区非空 (虽然我们不看接收数据，但必须等待传输完成)
    while((SPI2->SR & (1<<0)) == 0); 
    return SPI2->DR;	    
}

