

 // definitions for SDCardUtilities.c ONLY: SPI Module configuration is in MyConfig.h
 #ifndef _SDMODULE
 #define _SDMODULE

#include <stdint.h>
#include <definitions.h>

 #define DBUG     1
 #define NODBUG    0


 #define CMD0_GO_IDLE_STATE         0             // CMD0 - resets card
 #define SEND_OP_COND        1            // CMD1 - send operating condition
 #define CMD8_SEND_IF_COND        8            // CMD8 - send interface condition
#define CMD8_PATTERN 0xAA // arb. pattern while sending CMD8
#define CMD8_VOLTAGE 0x01 // voltage in range 2.7V-3.6V..
 #define SEND_CSD            9            // CMD9 - send card status
 #define SET_BLOCKLEN        16            // CMD16 - set blocklength
 #define CMD24_WRITE_SINGLE_BLOCK    24            // CMD24 - write (single) block
 #define CMD55_APP_CMD                55            // CMD55 - next command is an application specific command, not standard
 #define CMD_58_READ_OCR            58            // CMD58 - read the ocr register
 #define CRC_ON_OFF            59            // CMD59 - turns crc on or off

#define CMD12_STOP_TRANSMISSION	12				// while acknowledge of transmission failed with write error status token
#define CMD13_SEND_STATUS            13            // ACMD13 - send sd card status
 #define ACMD41_SD_SEND_OP_COND        41            // ACMD41 - send operating condition

 #define READ_SINGLE_BLOCK    17            // CMD17 - read single block


 #define START_BLOCK            0xFE        // used to indicate the start of a data block
 #define MMC_FLOATING_BUS     0xFF        // Floating bus condition(?)



// R1 SPI response
#define R1_IDLE_STATE 0x01
#define R1_ERASE_RESET 0x02
#define R1_ILLEGAL_COMMAND 0x04
#define R1_SUCCESS 0

#define TOKEN_SINGLE_WRITE_READ 0b11111110

#define BLOCK_SIZE 512


typedef struct __attribute__((packed))
{
	uint8_t m_CRCEndBit;
	uint32_t m_argument;
	uint8_t m_commandIndex :6;
	uint8_t m_transmissionBit :1;
	uint8_t m_startBit :1;
} SDCommand;




 // Prototypes

/// transmit data and receive sd card response.
///
/// @param[in] data to transmit.
///
/// @return  sd card 8bit response
uint8_t single_transmit(const uint8_t data);

uint8_t std_cmd(uint8_t command, uint32_t arg);

SPI_STATUS std_init(void);

void std_terminate(void);

void std_write(void);

enum SDType{
	SDHC_SDXC,
	SDSC
} f_SdType;

/// Address, unit of 512 bytes.
uint32_t f_address;
/// buffer of data
uint8_t f_bufferSD[BLOCK_SIZE];


 #endif
 /*****************************************************************************
  * EOF
  *****************************************************************************/

