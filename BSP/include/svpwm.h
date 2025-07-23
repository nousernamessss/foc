#ifndef SVPWM_H
#define SVPWM_H

#include "arm_math.h"
#include "string.h"
#include "math.h"

typedef struct
{
    float ua;
    float ub;
    float uc;
    float ud;
    float uq;
    float ualpha;
    float ubeta;
    float ialpha;
    float ibeta;
    float id;
    float iq;
    float ia;
    float ib;
    float ic;
    float theta;

    float tCmp1;
    float tCmp2;
    float tCmp3;

    float idTarget;
    float iqTarget;
}tsvpwmParam;

extern tsvpwmParam svpwmParam;

void svpwmParamInit(void);
void antiClarkConvert(void);
void antiParkConvert(void);
void clarkConvert(void);
void parkConvert(void);
void svpwmCalculate(void);
#endif //SVPWM_H
