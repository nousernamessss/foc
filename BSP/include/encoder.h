#ifndef ENCODER_H
#define ENCODER_H

#include "led.h"
#include "param.h"
#include "main.h"
#include "hfi.h"
#include "if.h"
#include "svpwm.h"
#include "rlsEstimator.h"

extern uint32_t key2Count;
extern uint32_t key3Count;
extern uint8_t qprControlEnabled;

typedef struct
{
    int32_t encoderPulseCount;
    uint8_t calculateCounte;
    float motorSpeedRpm;
    float filterParam;
    float initialAngle;
    eMotorState motorState;
}tencoderParam;

extern tencoderParam encoderParam;

void encoderInit(tencoderParam *encoderParam);
void encoderStep(void);
#endif //ENCODER_H
