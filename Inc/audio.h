// Module for audio related processing

#ifndef audio_h
#define audio_h

#include "stm32f7xx_hal.h"
#include "frequency_sensor.h"
#include <arm_math.h>

#define AUDIO_BUFFER_SIZE 1024
#define NUM_BUCKETS 60
#define NUM_COLUMNS 80

extern FrequencySensor_TypeDef *frequency_sensor;

void Audio_Init(void);
void Audio_ensure_i2s_frame_sync(void);

int16_t* Audio_GetCurrentBuffer(void);
int16_t* Audio_GetBuffer(uint8_t which);
Drivers_TypeDef* Audio_GetProcessedOutput(void);

#endif
