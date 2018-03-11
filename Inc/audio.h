// Module for audio related processing

#ifndef audio_h
#define audio_h

#include "stm32f7xx_hal.h"

#define AUDIO_BUFFER_SIZE 512

void Audio_Init(void);
int16_t* Audio_GetCurrentBuffer(void);
int16_t* Audio_GetBuffer(uint8_t which);

#endif
