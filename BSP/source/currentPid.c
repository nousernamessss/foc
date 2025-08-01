#include "currentPid.h"

#include "svpwm.h"

tcurrentParam idParam;
tcurrentParam iqParam;

void currentPidParamInit(tcurrentParam *idParam, tcurrentParam *iqParam)
{
    idParam->Kp = 0.9735f;
    idParam->Ki = 575.6636f;
    idParam->currentRef = 0.f;
    idParam->currentFdk = 0.f;
    idParam->outMax = motorParam.udc / sqrtf(3) * 0.9f;
    idParam->outMin = -motorParam.udc / sqrtf(3) * 0.9f;
    idParam->out = 0.f;
    idParam->integral_state = 0.f;

    iqParam->Kp = 0.9735f;
    iqParam->Ki = 575.6636f;
    iqParam->currentRef = 0.f;
    iqParam->currentFdk = 0.f;
    iqParam->outMax = motorParam.udc / sqrtf(3) * 0.9f;
    iqParam->outMin = -motorParam.udc / sqrtf(3) * 0.9f;
    iqParam->out = 0.f;
    iqParam->integral_state = 0.f;
}

inline void setIdRef(tcurrentParam *idParam, float value)
{
    idParam->currentRef = value;
}

inline void setIdFdk(tcurrentParam *idParam, float value)
{
    idParam->currentFdk = value;
}

inline void setIqRef(tcurrentParam *iqParam, float value)
{
    iqParam->currentRef = value;
}

inline void setIqFdk(tcurrentParam *iqParam, float value)
{
    iqParam->currentFdk = value;
}

static void currentPidStep(tcurrentParam *param)
{
    float error;

    error = param->currentRef - param->currentFdk;
    param->integral_state += error;

    // if (param->integral_state > param->outMax)
    //     param->integral_state = param->outMax;
    // else if (param->integral_state < param->outMin)
    //     param->integral_state = param->outMin;
    // else
    //     param->integral_state = param->integral_state;
    //
    param->out = param->Kp * error + motorParam.period * param->Ki * param->integral_state;
    if (param->out > param->outMax)
        param->out = param->outMax;
    else if (param->out < param->outMin)
        param->out = param->outMin;
    else
        param->out = param->out;
}

void resetCurrentPidParam(tcurrentParam *param)
{
    param->integral_state = 0;
    param->currentFdk = 0.f;
    param->currentRef = 0.f;
}

void currentPidCalculate(void)
{
    currentPidStep(&idParam);
    currentPidStep(&iqParam);

    svpwmParam.ud = idParam.out;
    svpwmParam.uq = iqParam.out;
}