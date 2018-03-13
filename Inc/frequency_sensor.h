
#ifndef frequency_sensor_h
#define frequency_sensor_h

#include "stm32f7xx_hal.h"
#include <arm_math.h>

typedef struct {
    uint16_t size;
    float32_t *gain;
    float32_t *diff;
} FilterValues_TypeDef;

typedef struct {
    float32_t offset;
    float32_t gain;
    float32_t differential_gain;
    float32_t sync;

    uint16_t mode;
} Parameters_TypeDef;

typedef struct {
    float32_t **amp;
    float32_t *diff;
    float32_t *energy;
    float32_t bass;
} Drivers_TypeDef;

typedef struct {
    uint16_t frame_size;
    float32_t *filterParams;
    float32_t *gain;
    float32_t *frame;
    float32_t *err;
    float32_t kp;
    float32_t kd;
} GainController_TypeDef;

typedef struct {
    uint16_t size;
    uint16_t columns;
    Parameters_TypeDef *params;
    FilterValues_TypeDef *filterParams;
    FilterValues_TypeDef *filterValues;
    GainController_TypeDef *agc;
    float32_t preemphasis;
    Drivers_TypeDef *drivers;
    uint8_t render_lock;
} FrequencySensor_TypeDef;

FilterValues_TypeDef* NewFilterValues(uint16_t size, float32_t *gain_values, float32_t *diff_values);
Drivers_TypeDef* NewDrivers(uint16_t size, uint16_t columns);
GainController_TypeDef* NewGainController(uint16_t size, float32_t *params, float32_t kp, float32_t kd);
FrequencySensor_TypeDef* NewFrequencySensor(uint16_t size, uint16_t columns);

void FS_Process(FrequencySensor_TypeDef *fs, float32_t *frame);

#endif