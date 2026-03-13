#ifndef __MATH_USER_H__
#define __MATH_USER_H__

#define R2D (180.0f / PI)  //弧度转角度
#define D2R (PI / 180.0f)  //角度转弧度
#include "Includes.h"

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);

#endif
