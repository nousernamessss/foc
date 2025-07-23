#include "svpwm.h"

#include "param.h"

tsvpwmParam svpwmParam;

void svpwmParamInit(void)
{
    memset(&svpwmParam, 0, sizeof(svpwmParam));
}
void antiClarkConvert(void)
{
    svpwmParam.ua = svpwmParam.ualpha;
    svpwmParam.ub = (-1.f / 2.f) * svpwmParam.ualpha + (sqrtf(3) / 2.f) * svpwmParam.ubeta;
    svpwmParam.uc = (-1.f / 2.f) * svpwmParam.ualpha - (sqrtf(3) / 2.f) * svpwmParam.ubeta;
}

void antiParkConvert(void)
{
    svpwmParam.ualpha = svpwmParam.ud * arm_cos_f32(svpwmParam.theta) -
                    svpwmParam.uq * arm_sin_f32(svpwmParam.theta);
    svpwmParam.ubeta = svpwmParam.ud * arm_sin_f32(svpwmParam.theta) +
                    svpwmParam.uq * arm_cos_f32(svpwmParam.theta);
}

void clarkConvert(void)
{
    svpwmParam.ialpha = svpwmParam.ia;
    svpwmParam.ibeta = (svpwmParam.ib - svpwmParam.ic) / sqrtf(3.0f);
}

void parkConvert(void)
{
    svpwmParam.id = svpwmParam.ialpha * arm_cos_f32(svpwmParam.theta) +
                    svpwmParam.ibeta * arm_sin_f32(svpwmParam.theta);

    svpwmParam.iq = svpwmParam.ibeta * arm_cos_f32(svpwmParam.theta) -
                    svpwmParam.ialpha * arm_sin_f32(svpwmParam.theta);
}

void svpwmCalculate(void)
{
    float max = fmaxf(svpwmParam.ua, fmaxf(svpwmParam.ub, svpwmParam.uc));
    float min = fminf(svpwmParam.ua, fminf(svpwmParam.ub, svpwmParam.uc));

    float v0 = (-1.f / 2.f) * (max + min);
    float tempA = svpwmParam.ua + v0;
    float tempB = svpwmParam.ub + v0;
    float tempC = svpwmParam.uc + v0;

    svpwmParam.tCmp1 = (-tempA / motorParam.udc + 0.5f) * motorParam.Ts;
    svpwmParam.tCmp2 = (-tempB / motorParam.udc + 0.5f) * motorParam.Ts;
    svpwmParam.tCmp3 = (-tempC / motorParam.udc + 0.5f) * motorParam.Ts;
}
