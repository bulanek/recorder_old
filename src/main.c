/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


//#include <stm32l1xx.h>
//#include <stm32l152c_discovery.h>
#include <stm32f4xx.h>

#include <definitions.h>
#include <recorder.h>
#include <sdcard.h>
//#include <stm32l152c_discovery_glass_lcd.h>
			
// Sum of 512B signal
uint32_t volatile f_signalPower = 0U;
/// Cached sum of 512B signal
uint32_t f_SignalCache = 0U;

const uint32_t SIGNAL_MIN = 100U;
uint32_t f_LowLevelCounter = 0U;
const uint32_t LOW_LEVEL_COUNTER_MAX = 10U;
uint32_t f_HighLevelCounter = 0U;
const uint32_t HIGH_LEVEL_COUNTER_MAX = 2U;

// Position of buffer for next processing (changed in I2S interrupt)
volatile uint32_t f_BufferPosition;
// cache of f_BufferPosition in order to detect change of f_BufferPosition
uint32_t f_BufferPositionCache = 0U;
volatile uint32_t f_TerminateSPI = 0U;

volatile uint16_t f_BufferI2S[BLOCK_SIZE/2];

volatile uint8_t f_recordOn;
volatile uint8_t f_recordThreshold;

volatile uint8_t f_Initialized = 0U;


int main(void)
{
	__disable_irq();

	SystemInit();

	f_BufferPosition = 0U;
	f_signalPower = 0U;
	f_BufferPositionCache = 0U;
	f_LowLevelCounter = f_HighLevelCounter = 0U;

	if (f_Initialized == 0U)
	{
		Initialize();
		f_Initialized = 1U;
	}
	else {
		SET_REGISTER_VALUE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN, 1);
		f_recordOn = RECORD_PORT->ODR & RECORD_ON_PIN;
		f_recordThreshold = RECORD_PORT->ODR
				& RECORD_THRESHOLD_PIN;
		SET_REGISTER_VALUE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN, 0);
	}
	__enable_irq();

	__WFI();
while (f_recordOn != 0U) {
		__disable_irq();
		// interrupt I2S occured.
		if (f_BufferPosition != f_BufferPositionCache)
		{
			// Copy buffer I2S to buffer of SD card
			if (f_BufferPosition == 0)
			{
				for (int i = 0; i < BLOCK_SIZE; ++i) {
					f_bufferSD[i] =
							(uint8_t) (f_BufferI2S[i / 2] >> 8 * (i % 2));
				}
			}
			f_BufferPositionCache = f_BufferPosition;
			f_SignalCache = f_signalPower;
			if (f_BufferPositionCache == 0U) {
				f_signalPower = 0U;
				__enable_irq();

				if (f_TerminateSPI == 0U) {
					std_write();
					if (f_SignalCache < SIGNAL_MIN) {
						++f_LowLevelCounter;
					}
				} else {
					if ((f_BufferPositionCache == 0U)
							&& (f_SignalCache > SIGNAL_MIN)) {
						++f_HighLevelCounter;
					}
				}


//				if (f_TerminateSPI == 0)
//				{
//					if (f_LowLevelCounter > LOW_LEVEL_COUNTER_MAX) {
//						std_terminate();
//						SET_REGISTER_VALUE(SPI1->CR1, SPI_CR1_SPE, 0);
//					}
//				}else{
//					if (f_HighLevelCounter > HIGH_LEVEL_COUNTER_MAX) {
//						// Enable SPI
//						SET_REGISTER_VALUE(SPI1->CR1, SPI_CR1_SPE, 1);
//						std_init();
//					}
//				}
			}
			else{
				__enable_irq();
			}
		}
		else{
		__enable_irq();
		}
		__WFI();
	}
}
