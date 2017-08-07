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
	(void) SPI_SD_CARD_REG->DR;
	while ((SPI_SD_CARD_REG->SR & SPI_SR_TXE) == 0) ;
	SPI_SD_CARD_REG->DR = data;
	while ((SPI_SD_CARD_REG->SR & SPI_SR_RXNE) == 0) ;
	uint32_t returnData = SPI_SD_CARD_REG->DR;
	return returnData;
}


uint8_t std_cmd(uint8_t command, const uint32_t arg) {

	uint32_t result = 0xFFU;
	SDCommand SDCommand;
	SDCommand.m_commandIndex = command;
	SDCommand.m_argument = arg;
	SDCommand.m_startBit = 0U;
	SDCommand.m_transmissionBit = 1U;

	// set CRC
	if (command == CMD0_GO_IDLE_STATE) {
		// argument 0
		SDCommand.m_CRCEndBit = 0x95;
	} else if (command == CMD8_SEND_IF_COND) {
		// argument 0x1AA
		SDCommand.m_CRCEndBit = 0x87;
	}
	// 5 - commandSequence size
	(void) SPI_SD_CARD_REG->DR;
	for (int i = 0; i < 6; ++i) {
		while ((SPI_SD_CARD_REG->SR & SPI_SR_TXE) == 0);
		uint8_t value = *((uint8_t*) &SDCommand + 6 - i - 1);
		SPI_SD_CARD_REG->DR = value;// (uint32_t) *((uint8_t*) &SDCommand + 6 - i - 1);
	}
	while ((SPI_SD_CARD_REG->SR & SPI_SR_TXE) == 0U)
		;
	while ((SPI_SD_CARD_REG->SR & SPI_SR_RXNE) == 0U)
		;
	result = SPI_SD_CARD_REG->DR;
	for (int i = 0; i < 50; ++i) {
	}

	while (((result = single_transmit(0xFF)) & 0x80) != 0) ;

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
			uint8_t value = single_transmit(0XFF);
			r7ResponseLSB |= (value << ((3 - i) * 8));
		}
//		if (((r7ResponseLSB & 0xF00) >> 8) != CMD8_VOLTAGE) {
//			// unsupported voltage
//	//		assert_failed(__FILE__,__LINE__);
//			return SPI_ERROR;
//		}
		if (((Response7*) &r7ResponseLSB)->m_voltageAccepted != CMD8_VOLTAGE)
		{
			return SPI_ERROR;
		}


		if (((Response7*) &r7ResponseLSB)->m_checkPattern != CMD8_PATTERN)
		{
			// wrong pattern back
			return SPI_ERROR;
		}
	}

	r7ResponseMSB = std_cmd(CMD_58_READ_OCR, 0);
	if (r7ResponseMSB != R1_IDLE_STATE)
	{
		return SPI_ERROR;
	}

	// TODO check OCR
	r7ResponseLSB = 0U;
	for (unsigned int i = 0; i < 4U; ++i) {
		uint8_t value = single_transmit(0XFF);
		r7ResponseLSB |= (value << ((3 - i) * 8));
	}
	if ((((Response3*) &r7ResponseLSB)->m_voltages_27_36 & 0x80) == 0U) {
		// wrong OCR voltage supported
		return SPI_ERROR;
	}
	if (((Response3*) &r7ResponseLSB)->m_PowerUpStatus != 0U) {
		return SPI_ERROR;
	}

	// all acmd preceded by CMD55.
	std_cmd(CMD55_APP_CMD, 0);
	// SDSC and also SDXC (30 bit)
	r7ResponseMSB = std_cmd(ACMD41_SD_SEND_OP_COND, 0x4000);
	if (r7ResponseMSB != R1_IDLE_STATE)
	{
		return SPI_ERROR;
	}

	r7ResponseMSB = std_cmd(CMD_58_READ_OCR, 0);
	if (r7ResponseMSB != R1_IDLE_STATE)
	{
		return SPI_ERROR;
	}

	r7ResponseLSB = 0U;
	for (unsigned int i = 0; i < 4U; ++i) {
		r7ResponseLSB |= single_transmit(0xFF) << (3 - i) * 8;
	}

	// Get CCS
	if (((Response3*) &r7ResponseLSB)->m_CCS != 0U) {
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

void std_updateCSDReg(void)
{
	uint8_t response8Bytes = std_cmd(CMD9_SEND_CSD, 0U);

	for (uint8_t i = 0U; i < CSD_REG_SIZE_BYTES; ++i) {
		*((uint8_t*) &f_csdRegister + CSD_REG_SIZE_BYTES - 1 - i) = single_transmit(
				0xFF);
	}
}


 /*****************************************************************************
  * EOF
  *****************************************************************************/
