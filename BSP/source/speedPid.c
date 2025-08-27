#include "speedPid.h"

tspeedParam speedParam;

void speedPidParamInit(tspeedParam *tspeedParam)
{
    tspeedParam->Kp = 0.0147f;
    tspeedParam->Ki = 0.24f;
    tspeedParam->integral_state = 0.f;
    tspeedParam->outMax = 12.f;
    tspeedParam->outMin = -12.f;
    tspeedParam->speedFdk = 0.f;
    tspeedParam->speedRef = 0.f;
    tspeedParam->out = 0;
}

inline void setSpeedRef(tspeedParam *speedParam, float value)
{
#ifdef USE_MODBUS
    uint32_t temp_u32;
    float temp_float;

    temp_u32 = ((uint32_t)usRegHoldingBuf[0] << 16) | usRegHoldingBuf[1];

    memcpy(&temp_float, &temp_u32, sizeof(float));

    speedParam->speedRef = temp_float;
#else

#ifdef USE_ENCORDER
    speedParam->speedRef = 100.f * (float)key2Count - 100.f * (float)key3Count;
#else
    speedParam->speedRef = 500.f + 100.f * (float)key2Count - 100.f * (float)key3Count;
    if (speedParam->speedRef > 1500.f)
        speedParam->speedRef = 1500.f;

#endif
#endif
}

inline void setSpeedFdk(tspeedParam *speedParam, float value)
{
    speedParam->speedFdk = value;
}

static void speedPidStep(tspeedParam *param)
{
    float error;

    error = param->speedRef - param->speedFdk;
    param->integral_state += error;

    // if (param->integral_state > param->outMax)
    //     param->integral_state = param->outMax;
    // else if (param->integral_state < param->outMin)
    //     param->integral_state = param->outMin;
    // else
    //     param->integral_state = param->integral_state;

    param->out = param->Kp * error + motorParam.period * param->Ki * param->integral_state;

    if (param->out > param->outMax)
        param->out = param->outMax;
    else if (param->out < param->outMin)
        param->out = param->outMin;
    else
        param->out = param->out;
}

void resetSpeedPidParam(tspeedParam *speedParam)
{
    speedParam->integral_state = 0.f;
    speedParam->speedFdk = 0.f;
    speedParam->speedRef = 0.f;
}

void speedPidCalculate(void)
{
    speedPidStep(&speedParam);
}