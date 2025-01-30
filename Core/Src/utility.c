#include "utility.h"

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float clip(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}