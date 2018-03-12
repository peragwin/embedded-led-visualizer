// Module for audio related processing

#ifndef audio_h
#define audio_h

#include "stm32f7xx_hal.h"
#include <arm_math.h>

#define AUDIO_BUFFER_SIZE 512
#define NUM_BUCKETS 32

void Audio_Init(void);
void Audio_ensure_i2s_frame_sync(void);

int16_t* Audio_GetCurrentBuffer(void);
int16_t* Audio_GetBuffer(uint8_t which);
float32_t* Audio_GetProcessedOutput(void);

#endif
