#include "param.h"

tMotorParam motorParam;

void paramInit(tMotorParam *motor_param)
{
    motor_param->udc = 24.f;
    motor_param->Ts = 8000.f;
    motor_param->Rs = 0.89f;
    motor_param->Ls = 0.00062F;
    motor_param->pair = 4.f;
    motor_param->lambda = 0.0059268f;
    motor_param->period = 0.0001f;
}
