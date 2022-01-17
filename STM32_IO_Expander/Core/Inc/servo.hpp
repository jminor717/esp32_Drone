#include "main.h"
#include "common_Config.h"





#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

void ServosInit();
void SetServoAngle(int16_t angle, uint16_t servoNum);

#ifdef __cplusplus
}
#endif



