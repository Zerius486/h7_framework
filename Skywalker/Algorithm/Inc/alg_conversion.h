#ifndef ALG_CONVERSION_H
#define ALG_CONVERSION_H
#include "stdint.h"
float RpmToRadS(float rpm);
float RadSToRpm(float rad_s);
float EncoderToRadian(uint16_t encoder_value);
uint16_t RadianToEncoder(float radian);
float RadianToDegree(float radian);
float DegreeToRadian(float degree);
float EncoderToDegree(uint16_t encoder_value);
uint16_t DegreeToEncoder(float degree);
#endif  // ALG_CONVERSION_H