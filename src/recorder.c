/*
 * recorder.c
 *
 *  Created on: 19. 3. 2017
 *      Author: boris
 */

#include <stm32f4xx_ll_gpio.h>
#include <system_stm32f4xx.h>
#include <stm32f401xe.h>
#include <stm32f4xx.h>

#include <assert.h>
#include <definitions.h>
#include <recorder.h>
#include <sdcard.h>
#include <diskio.h>


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

//	// SPI (sdcard) configuration using LL lib.
	LL_GPIO_InitTypeDef gpioInit;

	gpioInit.Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
	gpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpioInit.Pull = LL_GPIO_PULL_UP;
	gpioInit.Alternate = SPI_AF;

	ASSERT_ALL(LL_GPIO_Init(SPI_PORT, &gpioInit) == SUCCESS);

	gpioInit.Pin = SPI_NSS_PIN;
	ASSERT_ALL(LL_GPIO_Init(SPI_NSS_PORT, &gpioInit) == SUCCESS);

	gpioInit.Pin = SPI_MISO_PIN;
	gpioInit.Pull = LL_GPIO_PULL_UP;
	ASSERT_ALL(LL_GPIO_Init(SPI_PORT, &gpioInit) == SUCCESS);

	SPI_NSS_PORT->BSRR |= SUFFIX_EXPAND_ADD(GPIO_BSRR_BS, SPI_NSS_PIN_NUM);
	SPI_NSS_PORT->BSRR |= SUFFIX_EXPAND_ADD(GPIO_BSRR_BR, SPI_NSS_PIN_NUM);
	SPI_NSS_PORT->ODR = 1;


	// I2S (MEMS) configuration
	gpioInit.Pin =  I2S_CK_PIN | I2S_WS_PIN;
	gpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpioInit.Pull = LL_GPIO_PULL_DOWN;
	gpioInit.Alternate = SPI_I2S_AF;

	ASSERT_ALL(LL_GPIO_Init(I2S_PORT, &gpioInit) == SUCCESS);

	gpioInit.Mode = LL_GPIO_MODE_ALTERNATE;
	gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	gpioInit.Alternate = SPI_I2S_AF;
	gpioInit.Pin = I2S_SD_PIN;
	gpioInit.Pull = LL_GPIO_PULL_UP;

	ASSERT_ALL(LL_GPIO_Init(I2S_PORT, &gpioInit) == SUCCESS);

	// Config pin
	gpioInit.Pin =  CONFIG_PIN;
	gpioInit.Mode = LL_GPIO_MODE_INPUT;
	gpioInit.Pull = LL_GPIO_PULL_DOWN;

    ASSERT_ALL(LL_GPIO_Init(CONFIG_PORT, &gpioInit) == SUCCESS);

    // UART PINS

    gpioInit.Pin = CONFIG_UART_TX_PIN;
    gpioInit.Mode = LL_GPIO_MODE_OUTPUT;
    gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;

    ASSERT_ALL(LL_GPIO_Init(CONFIG_UART_PORT, &gpioInit) == SUCCESS);

    gpioInit.Pin = CONFIG_UART_RX_PIN;
    gpioInit.Mode = LL_GPIO_MODE_INPUT;
    gpioInit.Pull = LL_GPIO_PULL_UP;
    gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;

    ASSERT_ALL(LL_GPIO_Init(CONFIG_UART_PORT, &gpioInit) == SUCCESS);
}

uint8_t IsConfPinOn(void)
{
    return CONFIG_PORT->IDR & CONFIG_PIN;
}

void InitializeConfigUART(void)
{
    // TODO BB: in case of change USART1 -> different!!
    RCC->APB2ENR |=  RCC_APB2ENR_USART1EN;

    //Disable UART (in case the boot loader left it on)
    CONFIG_UART->CR1 = 0;
    CONFIG_UART->CR2 = 0;
    CONFIG_UART->CR3 = 0;

    /* Configure USART3 */
    /* 8 data bit, 1 start bit, 1 stop bit; no parity; receive enable;
     * over-sampling 16 */
    CONFIG_UART->CR1 = USART_CR1_RE | USART_CR1_TE;
    //BRR = 12 MHz / required UART clock
    CONFIG_UART->BRR = (uint16_t) (SystemCoreClock / BAUD_RATE);
    //enable uart
    CONFIG_UART->CR1 |= USART_CR1_UE;
}

////////////////////////////////////////////////////////////////////////////////
	// Initialize SPI comm with sdcard
////////////////////////////////////////////////////////////////////////////////
void InitializeSDCard(void)
{
	/* Enable the SPI clock */
	SET_REGISTER_VALUE(RCC->APB1ENR, RCC_APB1ENR_SPI2EN, 1);
	// TODO change while dev kit not used
	SET_REGISTER_VALUE(RCC->APB2ENR, RCC_APB2ENR_SPI1EN, 1);
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
	// set frequency between 100-400kHz
	// 16MHz F401 /64 = 250kHz
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_BR,0b101);
	std_init();
//	// Baud rate 16MHz/16 = 1MHz
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_BR,0b011);
	std_updateCSDReg();
}

////////////////////////////////////////////////////////////////////////////////
	// Initialize I2S for microphone
////////////////////////////////////////////////////////////////////////////////
void InitializeMicrophone(void)
{
    // Set clock
	SET_REGISTER_VALUE(RCC->CR, RCC_CR_PLLI2SON,0);
	SET_REGISTER_VALUE(RCC->CR, RCC_CR_PLLON,0);

    // STM32f041 reference note 105: VCO=PLL_input_f/PLLM; VCO in <1,2>MHz
	//                                 recommended 2MHz to suppress jitter.
    // for 16 MHz, PPLM = 8
    SET_REGISTER_VALUE(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, 8);
    // see reference note 590: table for PLLI2SN PLLI2SR
	SET_REGISTER_VALUE(RCC->PLLI2SCFGR, RCC_PLLI2SCFGR_PLLI2SN, 192);
	SET_REGISTER_VALUE(RCC->PLLI2SCFGR, RCC_PLLI2SCFGR_PLLI2SR, 3);
	// Set 16kHz sampling (assume  data length 16bit) (ref.note : 590)
	SET_REGISTER_VALUE(SPI2->I2SPR, SPI_I2SPR_I2SDIV, 62);
	SET_REGISTER_VALUE(SPI2->I2SPR, SPI_I2SPR_ODD, 0x01);
	SET_REGISTER_VALUE(RCC->CR, RCC_CR_PLLI2SON,1);
	SET_REGISTER_VALUE(RCC->CR, RCC_CR_PLLON,1);

	// Disable I2S
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SE, 0);
	// Interrupts
	SET_REGISTER_VALUE(SPI2->CR2, SPI_CR2_RXNEIE, 1);
	SET_REGISTER_VALUE(SPI2->CR2, SPI_CR2_ERRIE, 1);
	// Set I2S mod on SPI2
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SMOD, 0x01);
	// Set as master - receive
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SCFG, 0b11);
	// Standard: MSB
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SSTD, 0x01);
	// Set steady state (high level)
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_CKPOL, 0x01);
	// Data length to be transfered (16 bit)
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_DATLEN, 0x00);
	// Set number of bits per channel (16 bit)
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_CHLEN, 0x00);
	SET_REGISTER_VALUE(SPI2->I2SCFGR, SPI_I2SCFGR_I2SE, 1U);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize SPI.
////////////////////////////////////////////////////////////////////////////////
void InitializeRecorder(void)
{
	// Initialize NVIC
	IRQn_Type type = SPI2_IRQn;
	NVIC_EnableIRQ(type);
	NVIC_SetPriority(type, 4U);
	InitializeGPIO();
//	InitializeSDCard();
	InitializeMicrophone();
}
