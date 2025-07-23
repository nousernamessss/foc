#ifndef CURRENTPID_H
#define CURRENTPID_H

#include "arm_math.h"
#include "main.h"
#include "param.h"

typedef struct
{
    float Kp;
    float Ki;
    float outMax;
    float outMin;
    float currentRef;
    float currentFdk;

    float integral_state;
    float out;
}tcurrentParam;

extern tcurrentParam idParam;
extern tcurrentParam iqParam;

void currentPidParamInit(tcurrentParam *idParam, tcurrentParam *iqParam);
void setIdRef(tcurrentParam *idParam, float value);
void setIdFdk(tcurrentParam *idParam, float value);
void setIqRef(tcurrentParam *iqParam, float value);
void setIqFdk(tcurrentParam *iqParam, float value);
void currentPidCalculate(void);
#endif //CURRENTPID_H
