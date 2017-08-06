/*
 * definitions.h
 *
 *  Created on: 20. 3. 2017
 *      Author: boris
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

//#include <stm32l1xx_ll_gpio.h>
#include <stm32f4xx_ll_gpio.h>
//#include <sdcard.h>

////////////////////////////////////////////////////////////////////////////////
// I2S SPI GPIO
////////////////////////////////////////////////////////////////////////////////
#define CASSERT(predicate, file) _impl_CASSERT_LINE(predicate,__LINE__,file)

#define _impl_PASTE(a,b) a##b
#define _impl_CASSERT_LINE(predicate, line, file) \
    typedef char _impl_PASTE(assertion_failed_##file##_,line)[2*!!(predicate)-1];


#ifdef DEBUG
	#define DEBUG_TX_PIN (1<<6)
	#define DEBUG_AF (1 << 7)
#define DEBUG_PORT 	GPIOB
#endif

#define I2S_CLK                 	  RCC_AHBENR_GPIOBEN
#define I2S_PORT					  GPIOB

#define SPI_CLK                 	  RCC_AHBENR_GPIOCEN
#define SPI_PORT					  GPIOC
#define SPI_NSS_PORT					GPIOA

#define SPI_I2S_AF                        LL_GPIO_AF_5

#define SPI_AF							  LL_GPIO_AF_6

#define SPI_SD_CARD_REG			SPI3

#define SPI_SCK_PIN                       (1 << 10)
#define SPI_SCK_PIN_NUM                   10
#define SPI_MOSI_PIN                      (1 << 12)
#define SPI_MOSI_PIN_NUM                  12
#define SPI_MISO_PIN					  (1 << 11)
#define SPI_MISO_PIN_NUM				  11
#define SPI_NSS_PIN						  (1 << 15)
#define SPI_NSS_PIN_NUM					  15

#define I2S_SD_PIN                        (1 << 15)
#define I2S_CK_PIN						  (1 << 13)
#define I2S_WS_PIN						  (1 << 12)

#define LED_1_PIN							(1 << 3)
#define LED_2_PIN							(1 << 4)
#define LED_3_PIN							(1 << 5)
#define LED_PORT					GPIOB

#define RECORD_ON_PIN						(1 << 0)
#define RECORD_THRESHOLD_PIN				(1 << 1)
#define RECORD_PORT						GPIOC

////////////////////////////////////////////////////////////////////////////////
// Enable, disable gpio
////////////////////////////////////////////////////////////////////////////////

#define SET_RESET_PIN 					  (1 << 0)
#define SET_RESET_PORT					  GPIOA

#define SUFFIX_ADD(NAME, SUFFIX) NAME ## SUFFIX
#define SUFFIX_EXPAND_ADD(NAME, SUFFIX) SUFFIX_ADD(NAME,SUFFIX)

#define SET_REGISTER_VALUE(REGISTER, NAME, VALUE)							\
				REGISTER &= (~NAME ## _Msk);								\
                REGISTER |= ( (VALUE) << (NAME ## _Pos)) \

/// BLOCK SIZE for sd card
#define BLOCK_SIZE 512

volatile extern uint32_t f_TicksPerUs;
volatile extern uint32_t f_TerminateSPI;

/// Circular buffer of data for auto start and end of record.
volatile extern uint32_t f_BufferPosition;
volatile extern uint32_t f_signalPower;
volatile extern uint16_t f_BufferI2S[BLOCK_SIZE / 2];

volatile extern uint8_t f_recordOn;
volatile extern uint8_t f_recordThreshold;

//#ifdef DEBUG
//	UART_HandleTypeDef f_UartHandle;
//	static inline void debug_print(uint8_t* message, uint16_t size)
//	{
//		__disable_irq();
//		// Timeout is in milliseconds (missing doc)
//		HAL_UART_Transmit(&f_UartHandle, message, size, 1000);
//		__enable_irq();
//	}
//#endif

typedef enum{
	SPI_OK,
	SPI_ERROR
} SPI_STATUS;

#endif /* DEFINITIONS_H_ */
