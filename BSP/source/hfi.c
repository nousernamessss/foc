#include "hfi.h"

#include "tim.h"

thfiParam hfiParam;

void hfiInit(thfiParam *param)
{
    param->state = HFI_STATE_INITIAL_POS_ID;
    param->ialphaHighFrequency = 0.f;
    param->ibetaHighFrequency = 0.f;
    param->ialphaLowFrequency = 0.f;
    param->ibetaLowFrequency = 0.f;
    param->ialphaLast = 0.f;
    param->ibetaLast = 0.f;
    param->ialphaLastLast = 0.f;
    param->ibetaLastLast = 0.f;

    param->volatage = 2.4f;
    param->hfiSign = 1.f;

    param->estimatedSpeedRad = 0.f;
    param->estimatedAngleRad = 0.f;
    param->polarityCorrectionAngleRad = 0.f;
    param->thetaError = 0.f;
    param->pll_integrator = 0.f;
}

void hfiSeparateCurrents(thfiParam *param, float total_ialpha, float total_ibeta)
{
    // 实现公式(6) - 提取高频分量
    param->ialphaHighFrequency = (total_ialpha - 2.0f * param->ialphaLast + param->ialphaLastLast) / 4.0f;
    param->ibetaHighFrequency = (total_ibeta  - 2.0f * param->ibetaLast  + param->ibetaLastLast) / 4.0f;

    // 实现公式(7) - 提取基波分量
    param->ialphaLowFrequency = (total_ialpha + 2.0f * param->ialphaLast + param->ialphaLastLast) / 4.0f;
    param->ibetaLowFrequency = (total_ibeta  + 2.0f * param->ibetaLast  + param->ibetaLastLast) / 4.0f;

    // 更新电流历史值
    param->ialphaLastLast = param->ialphaLast;
    param->ibetaLastLast = param->ibetaLast;
    param->ialphaLast = total_ialpha;
    param->ibetaLast = total_ibeta;
}

void hfiStep(thfiParam *hfiParam, tsvpwmParam *svpwmParam)
{
    float newIalpha = svpwmParam->ialpha;
    float newIbeta = svpwmParam->ibeta;

    hfiParam->hfiSign = -1.f * hfiParam->hfiSign;
    float udHfi = hfiParam->hfiSign * hfiParam->volatage;

    float angleInjection = hfiParam->estimatedAngleRad + hfiParam->polarityCorrectionAngleRad;
    float cosAngle = arm_cos_f32(angleInjection);
    float sinAngle = arm_sin_f32(angleInjection);

    float ualphaHfi = udHfi * cosAngle;
    float ubetaHfi = udHfi * sinAngle;

    svpwmParam->ualpha += ualphaHfi;
    svpwmParam->ubeta += ubetaHfi;

    switch (hfiParam->state)
    {
        case HFI_STATE_INITIAL_POS_ID:
        {
            hfiParam->thetaError = -hfiParam->ialphaHighFrequency * arm_sin_f32(hfiParam->estimatedAngleRad) +
                                    hfiParam->ibetaHighFrequency * arm_cos_f32(hfiParam->estimatedAngleRad);

            float pOut = hfiParam->thetaError * HFI_PLL_KP;
            hfiParam->pll_integrator += hfiParam->thetaError * HFI_PLL_KI * motorParam.period;
            hfiParam->estimatedSpeedRad = pOut + hfiParam->pll_integrator;
            hfiParam->estimatedAngleRad += hfiParam->estimatedSpeedRad * motorParam.period;

            hfiParam->estimatedAngleRad = fmodf(hfiParam->estimatedAngleRad, 2.0f * PI);
            if (hfiParam->estimatedAngleRad < 0.0f) {
                hfiParam->estimatedAngleRad += 2.0f * PI;
            }
            break;
        }

        case HFI_STATE_POLARITY_ID:
        {
            float cosAngle = arm_cos_f32(hfiParam->estimatedAngleRad);
            float sinAngle = arm_sin_f32(hfiParam->estimatedAngleRad);

            float idFrequency = hfiParam->ialphaHighFrequency * cosAngle +
                                hfiParam->ibetaHighFrequency * sinAngle;

            if (svpwmParam->idTarget > 0.f)
                hfiParam->idFrequencySumPos += fabsf(idFrequency);
            else if(svpwmParam->idTarget < 0.f)
            {
                hfiParam->idFrequencySumNeg += fabsf(idFrequency);
            }

            break;
        }
        default:
            break;;
    }
}
