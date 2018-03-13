#include "display_grid.h"
#include <stdlib.h>
#include <arm_math.h>

DisplayGrid_TypeDef* NewDisplayGrid(FrequencySensor_TypeDef *fs) {
    DisplayParams_TypeDef paramValues = {
        .brightness = 127,
        .saturation = 4,
        .period = 24,
    };
    DisplayParams_TypeDef *params = malloc(sizeof(DisplayParams_TypeDef));
    memcpy(params, &paramValues, sizeof(DisplayParams_TypeDef));

    color_t *display = calloc(fs->size * fs->columns, sizeof(color_t));

    DisplayGrid_TypeDef *g = malloc(sizeof(DisplayGrid_TypeDef));
    g->params = params;
    g->fs = fs;
    g->display = display;
    return g;
}

static float32_t sigmoid(float32_t x) {
    //return 1.0 / (1 + exp(-x));
    float32_t a = (a < 0) ? -a : a;
    return (1.0 + x / (1.0 + a)) / 2.0;
}

static color_t get_hsv(DisplayParams_TypeDef *params, float32_t amp, float32_t phase, float32_t phi) {
    float32_t br = params->brightness;
    float32_t sa = params->saturation;

    float32_t hue = fmod(180 * (phi + phase) / PI, 360);
    if (hue < 0) hue += 360;

    float32_t sat = sigmoid(sa + amp - 2);
    float32_t val = sigmoid(br/255 * (1+amp) - 2);

    return hsv(hue, sat, val);
}

static void render_column(DisplayGrid_TypeDef *g, uint16_t col, color_t *colors) {
    float32_t *amp = g->fs->drivers->amp[0];
    if (g->fs->params->mode == 1) {
        amp = g->fs->drivers->amp[col];
    }
    float32_t *phase = g->fs->drivers->energy;
    float32_t ws = 2*PI / (float32_t)g->params->period / 16;
    float32_t phi = ws * (float32_t)col;

    for (int i = 0; i < g->fs->size; i++) {
        colors[i] = get_hsv(g->params, amp[i], phase[i], phi);
    }
}

void Render(DisplayGrid_TypeDef *g) {
    int hl = g->fs->columns / 2;
    color_t c[g->fs->size];
    for (int i = 0; i < hl; i++) {
        render_column(g, i, c);
        int i0 = (hl + i)*g->fs->size;
        int i1 = (hl - 1 - i)*g->fs->size;
        // display is in row order so we can use memcpy
        memcpy(g->display+i0, c, g->fs->size * sizeof(color_t));
        memcpy(g->display+i1, c, g->fs->size * sizeof(color_t));
    }
}
