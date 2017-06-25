/*
 * recorder.h
 *
 *  Created on: 19. 3. 2017
 *      Author: boris
 */

#ifndef RECORDER_H_
#define RECORDER_H_


#ifdef __cplusplus
extern "C" {
#endif

void assert_failed(uint8_t* file, uint32_t line);

void TM_Delay_Init(void);

void TM_DelayUs(const uint32_t timeUs);

void InitializeGPIO(void);

/// Main initialization procedure with sd card initialization as well.
void Initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* RECORDER_H_ */
