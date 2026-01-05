#include "spi.h"
//////////////////////////////////////////////////////////////////////////////////	 
//							  
////////////////////////////////////////////////////////////////////////////////// 	 
//以下是SPI模块的初始化代码，配置成主机模式 						  
//SPI口初始化
//////////////////////////////////////////////////////////////////////////////////	 
//	SPI1 初始化						  
////////////////////////////////////////////////////////////////////////////////// 	 
void SPI1_Init(void) 
{
    // 1. 使能 GPIOA 和 SPI1 时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // 2. 配置 PA5(SCK), PA6(MISO), PA7(MOSI) 为复用功能 (AF5)
    // 使用 PUPD_PU (上拉) 保证信号在空闲时稳定
    GPIO_Set(GPIOA, PIN5|PIN6|PIN7, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU);
    GPIO_AF_Set(GPIOA, 5, 5);  // PA5 -> SPI1_SCK
    GPIO_AF_Set(GPIOA, 6, 5);  // PA6 -> SPI1_MISO
    GPIO_AF_Set(GPIOA, 7, 5);  // PA7 -> SPI1_MOSI
    
    // 3. 配置 PA4(CS) 为普通输出，并初始化为高电平
    GPIO_Set(GPIOA, PIN4, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU);
    GPIOA->BSRR = (1 << 4);    // 显式拉高片选

    // 4. 配置 SPI1 寄存器
    // 必须确保 SPE=0 (SPI关闭状态) 才能修改 BR/CPOL/CPHA 等位
    SPI1->CR1 = 0; 

    // --- 频率设置 ---
    // BR[2:0] 含义 (fPCLK2 = 84MHz):
    // 011 (3): 16分频 = 5.25MHz
    // 100 (4): 32分频 = 2.625MHz (绞线连接建议使用此频率)
    SPI1->CR1 |= (4 << 3);      // 设置为 32 分频 (2.625MHz)
    
    // --- 模式设置 ---
    SPI1->CR1 |= SPI_CR1_MSTR;  // 主机模式
    SPI1->CR1 |= SPI_CR1_CPOL;  // Mode 3: CPOL=1
    SPI1->CR1 |= SPI_CR1_CPHA;  // Mode 3: CPHA=1
    SPI1->CR1 |= SPI_CR1_SSM;   // 软件从机管理
    SPI1->CR1 |= SPI_CR1_SSI;   // 内部 SS 信号置高
    
    // 5. 使能 SPI1
    SPI1->CR1 |= SPI_CR1_SPE; 
    
    // 启动传输前读一次 DR 寄存器清空可能存在的残留接收标志
    (void)SPI1->DR;
}

// SPI1 读写单字节 (带状态自锁增强)
u8 SPI1_ReadWriteByte(u8 tx_data) 
{
    // 等待发送缓冲区空 (TXE)
    while (!(SPI1->SR & SPI_SR_TXE)); 
    SPI1->DR = tx_data;
    
    // 等待接收缓冲区非空 (RXNE)
    while (!(SPI1->SR & SPI_SR_RXNE)); 
    return SPI1->DR;
}

// SPI1 速度设置函数
void SPI1_SetSpeed(u8 SpeedSet)
{
    SpeedSet &= 0x07;           // 限制范围 0~7
    SPI1->CR1 &= ~SPI_CR1_SPE;  // 修改波特率前必须先关闭 SPI
    SPI1->CR1 &= ~(7 << 3);     // 清除旧的 BR 位
    SPI1->CR1 |= (SpeedSet << 3); 
    SPI1->CR1 |= SPI_CR1_SPE;   // 重新开启
}

//////////////////////////////////////////////////////////////////////////////////	 
//	SPI2初始化						  
////////////////////////////////////////////////////////////////////////////////// 	 
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

