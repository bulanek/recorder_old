/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f4xx_it.h"

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

/// I2S interrupt
void SPI2_IRQHandler(void)
{
    uint32_t data = SPI2->DR;
    f_BufferI2S[f_BufferPosition] = data;
    f_BufferPosition = (++f_BufferPosition) % (BLOCK_SIZE / 2); // circular buffer
    /* Check if data are available in SPI Data register */
}

void EXTI0_IRQHandler(void)
{
}
