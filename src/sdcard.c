/*
 * DualSD.cpp
 *
 *  Created on: 1 Nov 2016
 *      Author: ralim
 */

#include <stddef.h>
#include <sdcard.h>
#include <spi.h>
#include <definitions.h>


uint8_t single_transmit(const uint8_t data)
{
	while ((SPI_SD_CARD_REG->SR & SPI_SR_TXE) == 0);
	SPI_SD_CARD_REG->DR = data;
	while ((SPI_SD_CARD_REG->SR & SPI_SR_RXNE) == 0);
	return SPI_SD_CARD_REG->DR;
}


uint8_t std_cmd(uint8_t command, const uint32_t arg) {

	uint8_t result = 0xFF;
	uint8_t commandSequence[] = { (uint8_t) (command | 0x40), (uint8_t) (arg
			>> 24), (uint8_t) (arg >> 16), (uint8_t) (arg >> 8), (uint8_t) (arg
			& 0xFF), 0xFF };
	SDCommand SDCommand;
	SDCommand.m_commandIndex = command;
	SDCommand.m_argument = arg;
	SDCommand.m_startBit = 0U;
	SDCommand.m_transmissionBit = 1U;

	// set CRC
	if (command == CMD0_GO_IDLE_STATE) {
		// argument 0
//		commandSequence[5] = 0x95;
		SDCommand.m_CRCEndBit = 0x95;
	} else if (command == CMD8_SEND_IF_COND) {
		// argument 0x1AA
//		commandSequence[5] = 0x87;
		SDCommand.m_CRCEndBit = 0x87;
	}
	// 5 - commandSequence size
	for (int i = 0; i < 6; ++i) {
		while ((SPI_SD_CARD_REG->SR & SPI_SR_TXE) == 0);
//		SPI_SD_CARD_REG->DR = commandSequence[5 - i - 1];
		uint8_t value = *((uint8_t*) &SDCommand + 6 - i - 1);
		SPI_SD_CARD_REG->DR = value;// (uint32_t) *((uint8_t*) &SDCommand + 6 - i - 1);
	}

	do {
//		while ((SPI_SD_CARD_REG->SR & SPI_SR_RXNE) == 0);
		while ((SPI_SD_CARD_REG->SR & SPI_SR_TXE) == 0);
		result = SPI_SD_CARD_REG->DR;
		SPI_SD_CARD_REG->DR = 0xFF;
	} while ((result & 0x80) != 0U);
	return result;
}

SPI_STATUS std_init(void)
{

	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_SPE, 1);
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1,SPI_CR1_SSM, 1);
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1, SPI_CR1_SSI, 1);

	SET_REGISTER_VALUE(SPI_NSS_PORT->MODER, GPIO_MODER_MODE15, 0b01);
	SPI_NSS_PORT->BSRR |= SPI_NSS_PIN;

	// At least 74 clock toggle clk
	for (int i = 0; i < 80/4U; ++i) {
		single_transmit(0xFF);
	}
	SET_REGISTER_VALUE(SPI_NSS_PORT->MODER, GPIO_MODER_MODE15, 0b10);
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1,SPI_CR1_SSM, 0);
	SET_REGISTER_VALUE(SPI_SD_CARD_REG->CR1,SPI_CR1_SSI, 0);



	uint8_t r7ResponseMSB = std_cmd(CMD0_GO_IDLE_STATE, 0);
	if (r7ResponseMSB != R1_IDLE_STATE)
	{
		return SPI_ERROR;
	}


	// rest 32LSBbits of R7 response
	uint32_t r7ResponseLSB = 0U;
	r7ResponseMSB = std_cmd(CMD8_SEND_IF_COND,
			(CMD8_VOLTAGE << 8U) | CMD8_PATTERN);

	if (r7ResponseMSB == R1_ILLEGAL_COMMAND)
	{
		// version  1.X sd or non sd
	//		assert_failed(__FILE__,__LINE__);
		return SPI_ERROR;
	}
	else
	{
		for (unsigned int i = 0; i < 4U; ++i) {
			r7ResponseLSB |= (single_transmit(0XFF) << ((3 - i) * 8));
		}
		if (((r7ResponseLSB & 0xF00) >> 8) != CMD8_VOLTAGE) {
			// unsupported voltage
	//		assert_failed(__FILE__,__LINE__);
			return SPI_ERROR;
		}
		if ((r7ResponseLSB & 0xFF) != CMD8_PATTERN)
		{
			// wrong pattern back
			return SPI_ERROR;
		}
	}

	r7ResponseMSB = std_cmd(CMD_58_READ_OCR, 0);
	// TODO check OCR

	// all acmd preceded by CMD55.
	std_cmd(CMD55_APP_CMD, 0);
	// SDSC and also SDXC (30 bit)
	while ((r7ResponseMSB = std_cmd(ACMD41_SD_SEND_OP_COND, 0x4000))
			== R1_IDLE_STATE) {
	}

	r7ResponseMSB = std_cmd(CMD_58_READ_OCR, 0);
	for (unsigned int i = 0; i < 4U; ++i) {
		r7ResponseLSB |= single_transmit(0xFF) << (3 - i) * 8;
	}
	// Get CCS
	if ((r7ResponseLSB & 0x8000)) {
		f_SdType = SDHC_SDXC;
	} else {
		f_SdType = SDSC;
	}
	return SPI_OK;
}

void std_terminate(void)
{
	// TODO
}

void std_write(void)
{

	uint8_t response8Bytes;
	if (f_SdType == SDSC)
	{
		response8Bytes = std_cmd(CMD24_WRITE_SINGLE_BLOCK,f_address * BLOCK_SIZE);
	}else
	{
		response8Bytes = std_cmd(CMD24_WRITE_SINGLE_BLOCK, f_address);
	}
	if (response8Bytes != R1_SUCCESS)
	{
		// TODO
	}

	response8Bytes = single_transmit(TOKEN_SINGLE_WRITE_READ);
	if (response8Bytes != 0xFF) {
		// TODO error
	}

	for (int i = 0; i < BLOCK_SIZE; ++i) {
		response8Bytes = single_transmit(f_bufferSD[i]);
	}
	// status token
	response8Bytes = single_transmit(0xFF);
	// Data accepted
	if ((response8Bytes & 0x0F) != 0b0101) {
		// CRC error (not possible as turned off)
		if ((response8Bytes & 0x0F) == 0b1011) {
		}
		// write error
		else {
			std_cmd(CMD12_STOP_TRANSMISSION, 0);
		}
	}

	// wait on reponse
	do {
		response8Bytes = single_transmit(0xFF);
	} while (response8Bytes == 0);
	// check status (R2)
	response8Bytes = std_cmd(CMD13_SEND_STATUS, 0);
	if (response8Bytes != 0) {
		// TODO
	} else {
		response8Bytes = single_transmit(0xFF);
		if (response8Bytes != 0) {
			// TODO error
		}
	}

	if (f_SdType == SDSC) {
		f_address += BLOCK_SIZE;
	} else {
		++f_address;
	}

}


 /*****************************************************************************
  * EOF
  *****************************************************************************/

