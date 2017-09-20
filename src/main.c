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

// CONSTANTS

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

// PRIVATE FUNCTIONS
static inline void initializeGlobVar(void);
/// @param[in]  u32_signal  signal of filled buffer
static inline void recordOnSM(const uint32_t u32_signal);

extern void disk_timerproc (void);
void SysTick_Handler (void)
{
	disk_timerproc();	/* Disk timer process */
}

int main(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);

    initializeGlobVar();
    InitializeRecorder();

    __enable_irq();
    __WFI();
    while (1U)
    {
        __disable_irq();
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
                    std_write();
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

static void initializeGlobVar(void)
{
    f_BufferPosition = 0U;
    f_BufferPositionCache = 0U;
    f_LowLevelCounter = f_HighLevelCounter = 0U;
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
