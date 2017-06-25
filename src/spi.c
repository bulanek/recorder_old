/*
 * spi.c
 *
 *  Created on: 4. 4. 2017
 *      Author: boris
 */
//#include <stm32l152xc.h>
#include <stm32f401xe.h>
#include <cmsis_gcc.h>


uint32_t sendSD(const uint32_t toSend)
{
	SPI1->DR = toSend;
	while( (SPI1->SR & SPI_SR_RXNE) == 0)
	{
	}
	return SPI1->DR;
}

