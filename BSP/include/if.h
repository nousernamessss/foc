#ifndef IF_H
#define IF_H

#include "main.h"
#include "stdint.h"
#include "param.h"
#include "svpwm.h"
#include "hfi.h"
#include "smc.h"

// #define NOT_USE_HFI
#define NOT_USE_SMC
// #define USE_ENCORDER
// #define USE_MODBUS
// #define PARAM_IDENTIFIER

typedef enum
{
    MOTOR_STATE_STOP,
    MOTOR_STATE_PRE_POSITION,
    MOTOR_STATE_RAMP_UP,
    MOTOR_INCRESE_IQ,
    MOTOR_STATE_RUN
}eMotorState;

extern eMotorState motor_state;
void ifStep(void);
#endif //IF_H
