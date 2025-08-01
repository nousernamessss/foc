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
    uint32_t temp_u32;
    float temp_float;

    // 2. 将两个16位的寄存器拼接成一个32位整数 (位模式)
    //    这假设设备是大端模式 (高位在前)，与Modbus标准一致。
    temp_u32 = ((uint32_t)usRegHoldingBuf[0] << 16) | usRegHoldingBuf[1];

    // 3. 使用 memcpy 将32位整数的位模式原封不动地复制到浮点数变量中
    //    这是最安全、最标准的“重新解释”方法，可以避免编译器的意外优化和严格别名规则问题。
    memcpy(&temp_float, &temp_u32, sizeof(float));

    // 4. 将正确解码后的浮点数值赋给目标变量
    speedParam->speedRef = temp_float;
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

void resetSpeedPidParam(tspeedParam *speedParam)
{
    speedParam->integral_state = 0.f;
    speedParam->speedFdk = 0.f;
    speedParam->speedRef = 0.f;
}

void speedPidCalculate(void)
{
    speedPidStep(&speedParam);
}