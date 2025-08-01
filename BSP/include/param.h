#ifndef PARAM_H
#define PARAM_H

#include "stdint.h"

typedef struct
{
    float Ls;
    float Rs;
    float lambda;
    float pair;
    float Ts;
    float udc;
    float period;
    uint8_t runFlag;
    uint8_t stopFlag;
}tMotorParam;

extern tMotorParam motorParam;

void paramInit(tMotorParam *motor_param);

#endif //PARAM_H
