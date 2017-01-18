#ifndef __SPI_H
#define __SPI_H

#include "stm32f4xx_spi.h"

#define SPI1_SCK_P5             GPIO_PinSource5
#define SPI1_MISO_P6            GPIO_PinSource6
#define SPI1_MOSI_P7            GPIO_PinSource7

void SPI1_init(void);
u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节

#endif
