#ifndef HFI_H
#define HFI_H

#include "arm_math.h"
#include "svpwm.h"
#include "param.h"

#define HFI_PLL_KP              200.0f      // 锁相环(PLL)的比例增益
#define HFI_PLL_KI              2500.0f    // 锁相环(PLL)的积分增益

typedef enum
{
    HFI_STATE_INITIAL_POS_ID,  // 阶段I: 初始位置辨识
    HFI_STATE_POLARITY_ID,     // 阶段II: 转子极性辨识
} HFI_State;

typedef struct
{
    HFI_State state;
    float ialphaLast;
    float ibetaLast;
    float ialphaLastLast;
    float ibetaLastLast;

    float ialphaHighFrequency;
    float ibetaHighFrequency;
    float ialphaLowFrequency;
    float ibetaLowFrequency;
    float volatage;
    float hfiSign;

    float estimatedSpeedRad;
    float estimatedAngleRad;

    float polarityCorrectionAngleRad;       //极性补偿角度

    float pll_integrator;
    float thetaError;

    float idFrequencySumPos;
    float idFrequencySumNeg;
}thfiParam;

extern thfiParam hfiParam;

void hfiInit(thfiParam *param);
void hfiSeparateCurrents(thfiParam *param, float total_ialpha, float total_ibeta);
void hfiStep(thfiParam *hfiParam, tsvpwmParam *svpwmParam);
#endif //HFI_H
