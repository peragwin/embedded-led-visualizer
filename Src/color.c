
#include "color.h"

color_t hsv(float h, float s, float v) {
  h = fmod(h, 360);
  if (h < 0) h += 360;
  h /= 60.0;
  float c = s * v;
  float a = fmod(h, 2.0) - 1.0;
  if (a < 0) a = -a;
  float x = c * (1.0 - a);
  float m = v - c;
  float r = 0;
  float g = 0;
  float b = 0;
  if (h < 1.0) {
    r = c; g = x;
  } else if (h < 2.0) {
    r = x; g = c;
  } else if (h < 3.0) {
    g = c; b = x;
  } else if (h < 4.0) {
    g = x; b = c;
  } else if (h < 5.0) {
    r = x; b = c;
  } else {
    r = c; b = x;
  }
  color_t color = {255*(m+r)+0.5, 255*(m+g)+0.5, 255*(m+b)+0.5};
  return color;
}

color_t fixBits(color_t c) {
  c.r &= 0xfc;
  c.g &= 0xfc;
  c.b &= 0xfc;
  return c;
}

color_t sim565(color_t c) {
  c.r &= 0xf8;
  c.g &= 0xfc;
  c.b &= 0xf8;
  return c;
}