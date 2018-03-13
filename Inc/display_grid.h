#ifndef display_grid_h
#define display_grid_h

#include "stm32f7xx_hal.h"
#include "frequency_sensor.h"
#include "color.h"

typedef struct {
    uint16_t brightness;
    float32_t saturation;
    uint16_t period;
} DisplayParams_TypeDef;

typedef struct {
    DisplayParams_TypeDef *params;
    FrequencySensor_TypeDef *fs;

    // 2d grid of colors in ROW ORDER so that we can use memcpy on each column
    color_t *display;
} DisplayGrid_TypeDef;

DisplayGrid_TypeDef* NewDisplayGrid(FrequencySensor_TypeDef *fs);
void Render(DisplayGrid_TypeDef *g);

#endif
