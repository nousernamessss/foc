#ifndef QPRCONTROL_H
#define QPRCONTROL_H

#include "param.h"
#include "arm_math.h"

typedef struct
{
    float kr;           //调谐增益
    float wc;           //截止频率(谐振控制器的带宽)
    float w0;           //谐振频率
    float Ts;           //采样频率

    float eKm1;         //差分方程中间相
    float eKm2;
    float yKm1;
    float yKm2;

    float b0, b1, b2;   //差分方程系数
    float a1, a2;
}tqprController;

extern tqprController qprControllerId;
extern tqprController qprControllerIq;

void qprInit(tqprController *qpr, float Kr, float wc, float initialW0, float Ts);
float qprUpdate(tqprController *qpr, float errorK, float newOmega);

#endif //QPRCONTROL_H
