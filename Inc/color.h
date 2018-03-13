#ifndef color_h
#define color_h

#include <stdint.h>
#include <math.h>

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} color_t;

color_t hsv(float h, float s, float v);
color_t fixBits(color_t c);
color_t sim565(color_t c);

#endif
