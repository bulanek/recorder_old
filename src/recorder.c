/*
 * recorder.c
 *
 *  Created on: 19. 3. 2017
 *      Author: boris
 */

//#include <stm32l1xx_ll_gpio.h>
//#include <system_stm32l1xx.h>
#include <stm32f4xx_ll_gpio.h>
#include <system_stm32f4xx.h>
//#include <stm32l152xc.h>
//#include <stm32l1xx.h>
#include <stm32f401xe.h>
#include <stm32f4xx.h>

//#include <stm32l1xx_ll_exti.h>
#include <stm32f4xx_ll_exti.h>
#include <definitions.h>
#include <recorder.h>
#include <sdcard.h>




/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}


void TM_Delay_Init(void)
{
	SystemCoreClockUpdate();
}

volatile uint32_t f_TicksPerUs = 0;
#pragma GCC push_options
#pragma GCC optimize("O3")
void TM_DelayUs(const uint32_t timeUs)
{
	volatile uint32_t cycles = f_TicksPerUs/1000000L * timeUs;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while(DWT->CYCCNT - start < cycles);
}
#pragma GCC pop_options



////////////////////////////////////////////////////////////////////////////////
/// Initialize SPI.
////////////////////////////////////////////////////////////////////////////////
void InitializeGPIO(void)
{
	// GPIO clock enable.
	SET_REGISTER_VALUE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN, 1);
	SET_REGISTER_VALUE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN, 1);
	SET_REGISTER_VALUE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN, 1);

	// SPI (sdcard) configuration using LL lib.
	LL_GPIO_InitTypeDef gpioInit;

#ifdef DEBUG
	gpioInit.Pin = DEBUG_TX_PIN;
	gpioInit.Mode = LL_GPIO_MODE_OUTPUT;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpioInit.Pull = LL_GPIO_PULL_NO;
	gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	if (LL_GPIO_Init(DEBUG_PORT, &gpioInit) != SUCCESS) {
		return;
	}

	LL_GPIO_SetOutputPin(DEBUG_PORT, DEBUG_TX_PIN);
	LL_GPIO_ResetOutputPin(DEBUG_PORT, DEBUG_TX_PIN);

#endif

	gpioInit.Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
	gpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpioInit.Pull = LL_GPIO_PULL_NO;
	gpioInit.Alternate = SPI_AF;

	if (LL_GPIO_Init(SPI_PORT, &gpioInit) != SUCCESS) {
	}

	gpioInit.Pin = SPI_MISO_PIN;
	gpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
	gpioInit.Pull = LL_GPIO_PULL_DOWN;

	if (LL_GPIO_Init(SPI_PORT, &gpioInit) != SUCCESS)
	{
	}
	// TODO remove this while not using dev kit.
	gpioInit.Pin = SPI_NSS_PIN;
	gpioInit.Mode = LL_GPIO_MODE_OUTPUT;
	gpioInit.Pull = LL_GPIO_PULL_NO;
	if (LL_GPIO_Init(SPI_NSS_PORT, &gpioInit) != SUCCESS) {
	}


	SPI_NSS_PORT->BSRR |= SUFFIX_EXPAND_ADD(GPIO_BSRR_BS_, SPI_NSS_PIN);

	SPI_NSS_PORT->BSRR |= SUFFIX_EXPAND_ADD(GPIO_BSRR_BR_, SPI_NSS_PIN);
	SPI_NSS_PORT->ODR = 1;


	// I2S (MEMS) configuration
	gpioInit.Pin =  I2S_CK_PIN;
	gpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpioInit.Pull = LL_GPIO_PULL_NO;
	gpioInit.Alternate = SPI_I2S_AF;

	if (LL_GPIO_Init(I2S_PORT, &gpioInit) != SUCCESS)
	{
	}

	gpioInit.Pin = I2S_SD_PIN;
	gpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.Pull = LL_GPIO_PULL_NO;
	gpioInit.Alternate = SPI_I2S_AF;

	if (LL_GPIO_Init(I2S_PORT, &gpioInit) != SUCCESS)
	{
	}

	gpioInit.Pin = RECORD_ON_PIN | RECORD_THRESHOLD_PIN;
	gpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
	gpioInit.Pull = LL_GPIO_PULL_NO;
	gpioInit.Alternate = LL_GPIO_AF_0;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_LOW;

	if (LL_GPIO_Init(RECORD_PORT, &gpioInit) != SUCCESS)
	{
	}
	// Set record configuration

//	volatile uint8_t f_recordOn = RECORD_PORT->ODR & RECORD_ON_PIN;
//	volatile uint8_t f_recordThreshold = RECORD_PORT->ODR & RECORD_THRESHOLD_PIN;

//	SET_REGISTER_VALUE(RCC->AHBENR,RCC_AHBENR_GPIOCEN,0);


	// Set- reset pin

	/*
 	// TODO macro expansion e.g. MODER0 -> XX(MODER,0)
	// Set input mode of reset pin (default)
 	SET_REGISTER_VALUE(
 			SET_RESET_PORT->MODER,
 			GPIO_MODER_MODER0,
			0);

 	// Set pulldown mode
 	SET_REGISTER_VALUE(
 			SET_RESET_PORT->PUPDR,
			GPIO_PUPDR_PUPDR0,
			2);

 	// Unmask external interrupt
 	SET_REGISTER_VALUE(
 			EXTI->IMR,
 			EXTI_IMR_MR0,
			1);

 	// Enable rising edge interrupt
 	SET_REGISTER_VALUE(
 			EXTI->RTSR,
 			EXTI_RTSR_TR0,
			1);

 	// Enable falling edge interrupt
 	SET_REGISTER_VALUE(
 			EXTI->FTSR,
 			EXTI_FTSR_TR0,
			1);

 	// Connect pin 0 to alternate function 5
 	SET_REGISTER_VALUE(
 			SET_RESET_PORT->AFR[0],
 			GPIO_AFRL_AFRL0, 5);
	 */



}


////////////////////////////////////////////////////////////////////////////////
/// Initialize SPI.
////////////////////////////////////////////////////////////////////////////////
void Initialize(void)
{


 	// Debug
#ifdef DEBUG
// 		f_UartHandle.Init.BaudRate = 115000U;
// 		f_UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
// 		f_UartHandle.Init.Mode = UART_MODE_TX;
// 		f_UartHandle.Init.OverSampling = UART_OVERSAMPLING_8;
// 		f_UartHandle.Init.Parity = UART_PARITY_NONE;
// 		f_UartHandle.Init.StopBits = UART_STOPBITS_1;
// 		f_UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
// 		if(HAL_UART_Init(&f_UartHandle) != HAL_OK)
// 		{
// 		}
#endif

	// Initialize NVIC
	IRQn_Type type = SPI2_IRQn;
	NVIC_EnableIRQ(type);
	NVIC_SetPriority(type, 2);
	// TODO SPI3->SPI1
	type = SPI3_IRQn;
	NVIC_EnableIRQ(type);
	NVIC_SetPriority(type, 2);
	type = EXTI0_IRQn;
	NVIC_EnableIRQ(type);
	NVIC_SetPriority(type, 1);

	InitializeGPIO();

////////////////////////////////////////////////////////////////////////////////
	// Initialize SPI comm with sdcard
////////////////////////////////////////////////////////////////////////////////
	/* Enable the SPI clock */
	SET_REGISTER_VALUE(RCC->APB1ENR, RCC_APB1ENR_SPI2EN, 1);
	// TODO change while dev kit not used
//	SET_REGISTER_VALUE(RCC->APB2ENR, RCC_APB2ENR_SPI1EN, 1);
	SET_REGISTER_VALUE(RCC->APB1ENR, RCC_APB1ENR_SPI3EN, 1);

	// Disable SPI ( set format while disabled SPI)
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_SPE, 0);

	// Transmit, receive
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_BIDIMODE, 0);
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_RXONLY, 0);

	// Data frame format, 8 bit
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_DFF, 0);

	// MSBFIRST
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_LSBFIRST, 0);

	// Master mode
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_MSTR, 1);

	// Clock polarity
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_CPOL, 1);
	// Clock phase
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_CPHA, 1);

	// Error interrupt
//	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR2, SPI_CR2_ERRIE, 1);

	// SS output enable
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR2, SPI_CR2_SSOE, 1);

	// SPI Mode
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->I2SCFGR, SPI_I2SCFGR_I2SMOD, 0);


////////////////////////////////////////////////////////////////////////////////
	// Initialize I2S
////////////////////////////////////////////////////////////////////////////////

	// RX buffer not empty interrupt enable
	SET_REGISTER_VALUE(SPI2->CR2, SPI_CR2_RXNEIE, 1);
	// Disable I2S
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SE, 0);
	// Set I2S mod on SPI2
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SMOD, 0x01);
	// Set as master - receive
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SCFG, 0x3);
	// Standard: MSB
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SSTD, 0x01);
	// Set steady state (high level)
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_CKPOL, 0x01);
	// Data length to be transfered (16 bit)
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_DATLEN, 0x00);
	// Set number of bits per channel (16 bit)
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_CHLEN, 0x00);

	// Set 16kHz sampling (assume 2 MHz master clock, data length 16bit)
	SET_REGISTER_VALUE(SPI2->I2SPR, SPI_I2SPR_I2SDIV, 15);
	SET_REGISTER_VALUE(SPI2->I2SPR, SPI_I2SPR_ODD, 0x01);

	// SPI start (SPE) inside sd initialization
	// set frequency between 100-400kHz (262.144 kHz)
	// TODO
//	SET_REGISTER_VALUE(RCC->ICSCR, RCC_ICSCR_MSIRANGE,0b010);
	std_init();
	// set frequency to default (2.097 MHz)
//	SET_REGISTER_VALUE(RCC->ICSCR, RCC_ICSCR_MSIRANGE,0b101);

	// Enable I2S
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SE, 0x01);


}



