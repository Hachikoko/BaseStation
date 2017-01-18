#include "SPI.h"

void SPI1_init(){ 	//初始化SPI1
	
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能 GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能 SPI1 时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	
	 /* SPI1 configuration *///初始化SPI1结构体
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI1设置为两线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //设置SPI1为主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //串行时钟在不操作时，时钟为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第一个时钟沿开始采样数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS信号由软件（使用SSI位）管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //SPI波特率预分频值为4
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7; //CRC值计算的多项式
	SPI_Init(SPI1,&SPI_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, SPI1_SCK_P5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, SPI1_MISO_P6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, SPI1_MOSI_P7, GPIO_AF_SPI1);
	
	SPI_Cmd(SPI1, ENABLE);
	
	return;
}


u8 SPI1_ReadWriteByte(u8 TxData)                                        //SPI读写数据函数
{		
	u8 retry=0;				 	
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)      //发送缓存标志位为空
	{
		retry++;
		if(retry>200)return 0;
	}			  
	/* Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(SPI1, TxData);                                    //通过外设SPI1发送一个数据
	retry=0;
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)   //接收缓存标志位不为空
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1);                                 //通过SPI1返回接收数据				    
}

