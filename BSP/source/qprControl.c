#include "qprControl.h"

tqprController qprControllerId;
tqprController qprControllerIq;

void qprInit(tqprController *qpr, float Kr, float wc, float initialW0, float Ts)
{
    qpr->kr = Kr;
    qpr->wc = wc;
    qpr->w0 = initialW0;
    qpr->Ts = Ts;

    qpr->eKm1 = 0.f;
    qpr->eKm2 = 0.f;
    qpr->yKm1 = 0.f;
    qpr->yKm2 = 0.f;

    float tDelayEquivalent = 1.5f * qpr->Ts;
    float phi = qpr->w0 * tDelayEquivalent;

    float sinW0T = arm_sin_f32(qpr->w0 * qpr->Ts);
    float cosW0T = arm_cos_f32(qpr->w0 * qpr->Ts);
    float sinPhi = arm_sin_f32(phi);
    float cosPhi = arm_cos_f32(phi);

    float b0Raw = qpr->kr * (qpr->wc / (2.f * qpr->w0)) * (sinW0T * cosPhi - sinPhi * (1 - cosW0T));
    float b1Raw = qpr->kr * (-qpr->wc / qpr->w0) * sinPhi * (1 - cosW0T);
    float b2Raw = qpr->kr * (qpr->wc / (2.f * qpr->w0)) * (-sinW0T * cosPhi - sinPhi * (1.f - cosW0T));

    float a0Raw = 1.f + (qpr->wc / qpr->w0) * sinW0T;
    float a1Raw = -2.f * cosW0T;
    float a2Raw = 1.f - (qpr->wc / qpr->w0) * sinW0T;

    qpr->b0 = b0Raw / a0Raw;
    qpr->b1 = b1Raw / a0Raw;
    qpr->b2 = b2Raw / a0Raw;
    qpr->a1 = a1Raw / a0Raw;
    qpr->a2 = a2Raw / a0Raw;
}

float qprUpdate(tqprController *qpr, float errorK, float newOmega)
{
    if (fabsf(newOmega - qpr->w0) > 1e-6f)
    {
        qpr->w0 = newOmega;
    }

    float krLocal = qpr->kr;
    float wcLocal = qpr->wc;
    float wc0Local = qpr->w0;
    float tsLocal = qpr->Ts;

    float tDelayEquivalent = 1.5f * qpr->Ts;
    float phi = qpr->w0 * tDelayEquivalent;

    float sinW0T = arm_sin_f32(qpr->w0 * qpr->Ts);
    float cosW0T = arm_cos_f32(qpr->w0 * qpr->Ts);
    float sinPhi = arm_sin_f32(phi);
    float cosPhi = arm_cos_f32(phi);

    float b0Raw = qpr->kr * (qpr->wc / (2.f * qpr->w0)) * (sinW0T * cosPhi - sinPhi * (1 - cosW0T));
    float b1Raw = qpr->kr * (-qpr->wc / qpr->w0) * sinPhi * (1 - cosW0T);
    float b2Raw = qpr->kr * (qpr->wc / (2.f * qpr->w0)) * (-sinW0T * cosPhi - sinPhi * (1.f - cosW0T));

    float a0Raw = 1.f + (qpr->wc / qpr->w0) * sinW0T;
    float a1Raw = -2.f * cosW0T;
    float a2Raw = 1.f - (qpr->wc / qpr->w0) * sinW0T;

    qpr->b0 = b0Raw / a0Raw;
    qpr->b1 = b1Raw / a0Raw;
    qpr->b2 = b2Raw / a0Raw;
    qpr->a1 = a1Raw / a0Raw;
    qpr->a2 = a2Raw / a0Raw;

    float yK= qpr->b0 * errorK + qpr->b1 * qpr->eKm1 + qpr->b2 * qpr->eKm2
                - qpr->a1 * qpr->yKm1 - qpr->a2 * qpr->yKm2;

    if (yK > 6.f)
        yK = 6.f;
    else if (yK < -6.f)
        yK = -6.f;

    qpr->eKm2 = qpr->eKm1;
    qpr->eKm1 = errorK;
    qpr->yKm2 = qpr->yKm1;
    qpr->yKm1 = yK;

    return yK;
}
