/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include <stm32l1xx_it.hpp>

#include <definitions.h>
#include <sdcard.h>
#include <recorder.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/


//void SPI1_IRQHandler(void)
//{
//	if (f_TerminateSPI)
//	{
//		while ((SPI_SD_CARD_REG->SR & SPI_SR_BSY) != 0);
//		// Disable SPI
//		SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_SPE, 0);
//	}
//}

void SPI2_IRQHandler(void) {
	if (f_TerminateSPI) {
		// wait one spi2 clock
		TM_DelayUs(1000U);
		// Disable I2S
		SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SE, 0x00);
		return;
	}
	f_BufferI2S[f_BufferPosition] = SPI2->DR;
	f_signalPower += f_BufferI2S[f_BufferPosition];
	f_BufferPosition = (++f_BufferPosition) % (BLOCK_SIZE / 2); // circular buffer
	/* Check if data are available in SPI Data register */
}

void EXTI0_IRQHandler(void) {
	if ((SET_RESET_PORT->IDR & SET_RESET_PIN) != 0) {
		f_TerminateSPI = 0U;
		// Enable SPI
		SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_SPE, 1);
	} else {
		f_TerminateSPI = 1U;
	}
}
