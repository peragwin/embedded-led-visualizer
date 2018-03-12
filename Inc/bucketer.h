
#ifndef bucketer_h
#define bucketer_h

#include "stm32f7xx_hal.h"
#include <arm_math.h>

typedef struct {
    uint16_t size;
    uint16_t buckets;
    uint16_t *indices;
    float32_t *output;
} Bucketer_TypeDef;

Bucketer_TypeDef* NewBucketer(uint16_t size, uint16_t buckets, float32_t f_min, float32_t f_max);
void Bucket(Bucketer_TypeDef *b, float32_t *frame);

#endif
