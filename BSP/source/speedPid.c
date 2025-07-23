#include "speedPid.h"

tspeedParam speedParam;

void speedPidParamInit(tspeedParam *tspeedParam)
{
    tspeedParam->Kp = 0.0147f;
    tspeedParam->Ki = 0.24f;
    tspeedParam->integral_state = 0.f;
    tspeedParam->outMax = 5.f;
    tspeedParam->outMin = -5.f;
    tspeedParam->speedFdk = 0.f;
    tspeedParam->speedRef = 0.f;
    tspeedParam->out = 0;
}

inline void setSpeedRef(tspeedParam *speedParam, float value)
{
    speedParam->speedRef = value;
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
    //
    param->out = param->Kp * error + motorParam.period * param->Ki * param->integral_state;
    // if (param->out > param->outMax)
    //     param->out = param->outMax;
    // else if (param->out < param->outMin)
    //     param->out = param->outMin;
    // else
    //     param->out = param->out;
}

void speedPidCalculate(void)
{
    speedPidStep(&speedParam);
}