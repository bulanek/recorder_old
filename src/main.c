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

#include <assert.h>
#include <definitions.h>
#include <recorder.h>
#include <sdcard.h>
#include <pdm_filter.h>
#include <ff.h>

#include <stdio.h>      // printf
#include <sys/stat.h>   // stat struct
#include <unistd.h>     // STDOUT_FILENO,..
#include <errno.h>

// CONSTANTS

const char DEFAULT_FILE_NAME[] = "record";

/// Max size of file recorded
const uint32_t FILE_SIZE_MAX_BYTES = 0xFFFFFF;
/// Signal for power off
const uint32_t SIGNAL_MIN_LOW = 100U;
/// Signal for power on
const uint32_t SIGNAL_MIN_HIGH = 200U;
/// Counter for signal < SIGNAL_MIN_LOW
uint32_t f_LowLevelCounter = 0U;
const uint32_t LOW_LEVEL_COUNTER_MAX = 10U;
/// Counter for signal > SIGNAL_MIN_HIGH
uint32_t f_HighLevelCounter = 0U;
const uint32_t HIGH_LEVEL_COUNTER_MAX = 2U;

// Position of buffer for next processing (changed in I2S interrupt)
volatile uint32_t f_BufferPosition;
// cache of f_BufferPosition in order to detect change of f_BufferPosition
uint32_t f_BufferPositionCache;
volatile uint32_t f_TerminateSPI = 0U;
volatile uint16_t f_BufferI2S[BLOCK_SIZE / 2];
volatile uint8_t f_recordOn;


typedef enum
{
    Config, // UART Communicate with user
    Run,
    Convert // Convert PDM data
} Mode;

typedef struct
{
    Mode m_mode;
    char m_fileNameBase[20];
} Configuration;

Configuration f_Configuration = {.m_mode = Config};


// PRIVATE FUNCTIONS
/// @param[in]  u32_signal  signal of filled buffer
static inline void recordOnSM(const uint32_t u32_signal);

static void ChangeConfigurationUART(Configuration* const pConfiguration);

static void ConvertPDM(void);

extern void disk_timerproc (void);
void SysTick_Handler (void)
{
	disk_timerproc();	/* Disk timer process */
}

int main(void)
{
    f_BufferPosition = 0U;
    f_BufferPositionCache = 0U;
    f_LowLevelCounter = f_HighLevelCounter = 0U;
    sprintf(f_Configuration.m_fileNameBase,"%s", DEFAULT_FILE_NAME );

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    __enable_irq();

    // Check on configuration pin if configuration mode
    InitializeGPIO();
    if (IsConfPinOn() != 0U)
    {
        f_Configuration.m_mode = Config;
        InitializeConfigUART();
        ChangeConfigurationUART(&f_Configuration);
    }

    if (f_Configuration.m_mode == Convert)
    {
        ConvertPDM();
        return 0;
    }

    InitializeRecorder();
    FIL file;
    TCHAR fileNameBuffer[30];
    // TODO BB: counter of files (of given max size)
    FILINFO fileInfo;
    fileInfo.fsize = 0;
    int16_t fileNumber = -1;
    FRESULT fileResult;

    sprintf(fileNameBuffer, "%s_%i.bin", f_Configuration.m_fileNameBase, ++fileNumber);
    fileResult = f_open(&file, fileNameBuffer,
                (BYTE) ( FA_OPEN_APPEND | FA_WRITE ));
    ASSERT_ALL(fileResult == FR_OK);
    fileResult = f_stat(fileNameBuffer, &fileInfo);
    ASSERT_ALL(fileResult == FR_OK);
    while (1U)
    {
        __disable_irq();
        if (fileInfo.fsize >= FILE_SIZE_MAX_BYTES)
        {
            fileResult = f_close(&file);
            ASSERT_ALL(fileResult == FR_OK);

            sprintf(fileNameBuffer, "%s_%i.bin", f_Configuration.m_fileNameBase, ++fileNumber);
            fileResult = f_open(&file, fileNameBuffer,
            (BYTE) ( FA_OPEN_APPEND | FA_WRITE ));
            fileResult = f_stat(fileNameBuffer, &fileInfo);
            ASSERT_ALL(fileResult == FR_OK);
        }
        // interrupt I2S occured.
        if (f_BufferPosition != f_BufferPositionCache)
        {
            f_BufferPositionCache = f_BufferPosition;
            // Copy buffer I2S to buffer of SD card
            if (f_BufferPositionCache == 0U)
            {
                auto uint32_t u32_signal = 0U;
                for (int i = 0; i < BLOCK_SIZE; ++i)
                {
                    f_bufferSD[i] =
                            (uint8_t) (f_BufferI2S[i / 2] >> 8 * (i % 2));
                    u32_signal += f_bufferSD[i];
                }
                __enable_irq();

                // state machine for f_recordOn
                recordOnSM(u32_signal);
                if (f_recordOn != 0U)
                {
                    UINT bytesWritten = 0;
                    fileResult = f_write(&file, (void*) f_bufferSD,
                    BLOCK_SIZE, &bytesWritten);
                    ASSERT_ALL(fileResult == FR_OK);
                    fileResult = f_stat(fileNameBuffer, &fileInfo);
                    ASSERT_ALL(fileResult == FR_OK);
                }
            }
            else
            {
                __enable_irq();
            }
        }
        else
        {
            __enable_irq();
        }
        __WFI();
    }
}

static void recordOnSM(const uint32_t u32_signal)
{
    if (f_recordOn != 0U)
    {
        if (u32_signal < SIGNAL_MIN_LOW)
        {
            if (++f_LowLevelCounter > LOW_LEVEL_COUNTER_MAX)
            {
                f_recordOn = 0U;
            }
        }
        else
        {
            f_LowLevelCounter = 0U;
        }
    }
    else
    {
        if (u32_signal > SIGNAL_MIN_HIGH)
        {
            if (++f_HighLevelCounter > HIGH_LEVEL_COUNTER_MAX)
            {
                f_recordOn = 1U;
            }
        }
        else
        {
            f_HighLevelCounter = 0U;
        }
    }
}

void ChangeConfigurationUART(Configuration* const pConfiguration)
{
    printf("Configurations of device:\n");
    printf("\t c: convert PDM and save\n");
    printf("\t r: running microphone\n");
    char input = (char) getchar();
    if (input == 'c')
    {
        pConfiguration->m_mode = Convert;
    }
    else if (input == 'r')
    {
        pConfiguration->m_mode = Run;
    }
    else
    {
        ChangeConfigurationUART(pConfiguration);
    }
}

void ConvertPDM(void)
{
    // TODO BB: define
    PDMFilter_InitStruct pdmFilter =
    {
            .Fs = 16000,
            .LP_HZ = 0,
            .HP_HZ = 0,
            .In_MicChannels = 1,
            .InternalFilter = {0}
    };
    PDM_Filter_Init(&pdmFilter);
    FIL pFile;
    TCHAR fileNameBuffer[30];
    uint32_t counter = 0U;
    FRESULT openCloseRes;
    uint8_t bufferRead[BLOCK_SIZE];
    while (1)
    {
        sprintf(fileNameBuffer, "%s_%i.bin", f_Configuration.m_fileNameBase,
                counter++);
        openCloseRes = f_open(&pFile, fileNameBuffer,
                (BYTE) ( FA_READ | FA_OPEN_EXISTING));
        ASSERT_ALL(openCloseRes == FR_OK);

        FRESULT readRes = FR_OK;
        UINT bytesToRead = BLOCK_SIZE;
        UINT bytesRead = 0;
        uint16_t bufferConverted[BLOCK_SIZE];
        const uint16_t mic_gain = 2;
        while (1)
        {
            readRes = f_read(&pFile, (void*) bufferRead, bytesToRead,
                    &bytesRead);
            ASSERT_ALL(readRes == FR_OK);
                // TODO BB: what is returned?
            PDM_Filter_80_LSB(bufferRead, bufferConverted,
                        mic_gain, &pdmFilter);
        }
        openCloseRes = f_close(&pFile);
        ASSERT_ALL(openCloseRes == FR_OK);
    }
}



// FOR PRINTF IMPLEMENTATION
int _read(int file, char *pData, int len)
{
    if (file != STDIN_FILENO)
    {
        errno = EBADF;
        return -1;
    }
    int bytes_read;
    for (bytes_read = 0; bytes_read < len; ++bytes_read)
    {
        while ((CONFIG_UART->SR & USART_SR_RXNE) == 0U);
        *pData = (char) CONFIG_UART->DR;
        ++pData;
    }
    return bytes_read;
}

int _write(int file, char *pData, int len)
{
    if (file != STDOUT_FILENO)
    {
        errno = EBADF;
        return -1;
    }
    int bytes_written;
    for (bytes_written = 0; bytes_written < len; ++bytes_written)
    {
        while ((CONFIG_UART->SR & USART_SR_TXE) == 0U);
        CONFIG_UART->DR = (char) *pData;
        if (*pData == '\n')
        {
            while ((CONFIG_UART->SR & USART_SR_TXE) == 0U);
            CONFIG_UART->DR = '\r';
        }
        ++pData;
    }
    return bytes_written;
}

int _close(int file)
{
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    if ((file == STDOUT_FILENO) ||
        (file == STDIN_FILENO) ||
        (file == STDERR_FILENO))
    {
        return 1;
    }

    errno = EBADF;
    return 0;
}
