#ifndef IF_H
#define IF_H

#include "main.h"
#include "stdint.h"
#include "param.h"
#include "svpwm.h"
#include "hfi.h"

// #define NOT_USE_HFI
#define USE_HFI

typedef enum
{
    MOTOR_STATE_STOP,
    MOTOR_STATE_PRE_POSITION,
    MOTOR_STATE_RAMP_UP,
    MOTOR_INCRESE_IQ,
    MOTOR_STATE_RUN
}eMotorState;

void ifStep(void);
#endif //IF_H
