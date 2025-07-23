#ifndef SPEED_H
#define SPEED_H

#include "main.h"
#include "svpwm.h"
#include "param.h"

typedef struct
{
    float Kp;
    float Ki;
    float outMax;
    float outMin;
    float speedRef;
    float speedFdk;

    float integral_state;
    float out;
}tspeedParam;

extern tspeedParam speedParam;

void speedPidParamInit(tspeedParam *tspeedParam);
void setSpeedRef(tspeedParam *idParam, float value);
void setSpeedFdk(tspeedParam *idParam, float value);
void speedPidCalculate(void);
#endif //SPEED_H
