#ifndef SMC_H
#define SMC_H

#include "param.h"
#include "arm_math.h"

typedef struct
{
    float c;        //滑模面参数
    float kExp;     //指数项增益
    float kReach;   //趋近率增益
    float phi;      //边界层厚度
    float dt;       //控制器执行周期

    float integralError;    //速度误差的积分项
    float s;                //滑膜面s的值
    float output;           //控制器输出IqRef
}tsmcParam;

extern tsmcParam smcParam;

void smcInit(tsmcParam *smcParam);
float setSmcSpeedRef(void);
void smcCalculate(tsmcParam *smcParam, float speedRef, float speedFdk);
#endif //SMC_H
