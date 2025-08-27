#include "smc.h"

#include "speedPid.h"

static float saturation(float x) {
    if (x > 1.0f) {
        return 1.0f;
    } else if (x < -1.0f) {
        return -1.0f;
    } else {
        return x;
    }
}

tsmcParam smcParam;

void smcInit(tsmcParam *smcParam)
{
    smcParam->c = 50.f;
    smcParam->kExp = 30.f;
    smcParam->kReach = 30.f;
    smcParam->dt = motorParam.period;
    smcParam->phi = 2.f;
    smcParam->integralError = 0.f;
    smcParam->output = 0.f;
    smcParam->s = 0.f;
}

static float speedLimit(float value)
{
    if (value > 1750.f)
        return 1750.f;
    else if (value < -1750.f)
        return -1750.f;
    else
        return value;
}

float setSmcSpeedRef(void)
{
    uint32_t temp_u32;
    float temp_float;

    temp_u32 = ((uint32_t)usRegHoldingBuf[0] << 16) | usRegHoldingBuf[1];

    memcpy(&temp_float, &temp_u32, sizeof(float));

    speedLimit(temp_float);
    return temp_float;
}

void smcCalculate(tsmcParam *smcParam, float speedRef, float speedFdk)
{
    float error = speedRef - speedFdk;
    smcParam->integralError += error * smcParam->dt;

    smcParam->s = error + smcParam->c * smcParam->integralError;

    float part1 = smcParam->kExp * smcParam->s;
    float part2 = smcParam->kReach * saturation(smcParam->s);
    float part3 = smcParam->c * error;

    smcParam->output = motorParam.J * (part1 + part2 + part3) / (3.f / 2.f * motorParam.pair * motorParam.lambda);
}
