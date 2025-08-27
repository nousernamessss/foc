#include "encoder.h"

#include <string.h>

#include "adc.h"
#include "speedPid.h"
#include "tim.h"

tencoderParam encoderParam;
static uint32_t state_counter = 0;
uint32_t key1Count = 0;
uint32_t key2Count = 0;
uint32_t key3Count = 0;

void encoderInit(tencoderParam *encoderParam)
{
    encoderParam->encoderPulseCount = 0;
    encoderParam->motorSpeedRpm = 0.f;
    encoderParam->filterParam = 0.1f;
    encoderParam->initialAngle = 0.f;
    encoderParam->motorState = MOTOR_STATE_STOP;
    encoderParam->calculateCounte = 5;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == Button1_Pin)
    {
        motorParam.runFlag = 1;
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

        HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
        HAL_TIM_Base_Start_IT(&htim6);

        // if (motorParam.runFlag == 1)
        // {
        //     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        //     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        //     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        //     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
        //
        //     HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        //     HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
        //     HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
        //
        //     HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
        //     HAL_TIM_Base_Stop_IT(&htim6);
        // }
    }

    else if(GPIO_Pin == Button2_Pin)
    {
        key2Count++;
    }

    else if(GPIO_Pin == Button3_Pin)
    {
        key3Count++;
    }
}

void encoderStep(void)
{
    switch (encoderParam.motorState)
    {
        case MOTOR_STATE_STOP:
        {
            if (motorParam.runFlag == 1)
            {
                encoderParam.motorState = MOTOR_STATE_PRE_POSITION;
            }
            break;
        }

        case MOTOR_STATE_PRE_POSITION:
        {
            state_counter++;
            if (hfiParam.state == HFI_STATE_INITIAL_POS_ID)
            {
                setIdRef(&idParam, 0.f);
                setIqRef(&iqParam, 0.f);

                if (state_counter >= 300)
                {
                    state_counter = 0;
                    hfiParam.state = HFI_STATE_POLARITY_ID;
                }
            }

            else if (hfiParam.state == HFI_STATE_POLARITY_ID)
            {
                if (state_counter < 100)
                {
                    setIdRef(&idParam, 3.f);
                }

                else if (state_counter < 200)
                {
                    setIdRef(&idParam, -3.f);
                }

                else
                {
                    if (hfiParam.idFrequencySumPos > hfiParam.idFrequencySumNeg)
                    {
                        hfiParam.polarityCorrectionAngleRad = 0.0f; // 方向正确，无需补偿
                    }
                    else
                    {
                        hfiParam.polarityCorrectionAngleRad = PI;
                    }

                    float initial_angle_rad = hfiParam.estimatedAngleRad + hfiParam.polarityCorrectionAngleRad;

                    encoderParam.motorState = MOTOR_STATE_RUN;
                    state_counter = 0;

                    encoderParam.initialAngle = initial_angle_rad;
                }
            }

            clarkConvert();
            hfiSeparateCurrents(&hfiParam, svpwmParam.ialpha, svpwmParam.ibeta);
            svpwmParam.theta = hfiParam.estimatedAngleRad + hfiParam.polarityCorrectionAngleRad;
            svpwmParam.id = hfiParam.ialphaLowFrequency * arm_cos_f32(svpwmParam.theta) +
                            hfiParam.ibetaLowFrequency * arm_sin_f32(svpwmParam.theta);
            svpwmParam.iq = -hfiParam.ialphaLowFrequency * arm_sin_f32(svpwmParam.theta) +
                            hfiParam.ibetaLowFrequency * arm_cos_f32(svpwmParam.theta);

            currentPidCalculate();
            antiParkConvert();

            hfiStep(&hfiParam, &svpwmParam);
            svpwmCalculate();
            break;
        }

        case MOTOR_STATE_RUN:
        {
            int32_t remainder = encoderParam.encoderPulseCount % 4000;
            svpwmParam.theta = ((float)remainder / 4000.f) * 2.f * PI * motorParam.pair + encoderParam.initialAngle;
            if (svpwmParam.theta > 2.f * PI)
                svpwmParam.theta -= 2.f * PI;
            if (svpwmParam.theta < 0.f)
                svpwmParam.theta += 2.f * PI;
            clarkConvert();
            parkConvert();

#ifdef NOT_USE_SMC
            encoderParam.calculateCounte++;
            if (encoderParam.calculateCounte % 5 == 0)
            {
                encoderParam.calculateCounte = 0;
                setSpeedRef(&speedParam, 0.f);
                setSpeedFdk(&speedParam, encoderParam.motorSpeedRpm);

                float speed = encoderParam.motorSpeedRpm;

                uint32_t *pSpeed = (uint32_t *)&speed;

                usRegHoldingBuf[2] = (*pSpeed & 0xFFFF0000) >> 16;
                usRegHoldingBuf[3] = (*pSpeed & 0x0000FFFF);

                speedPidCalculate();
            }

#ifdef PARAM_IDENTIFIER
            setIdRef(&idParam, 0.f + triWaveGenerate(&triWaveGenerator));
#else
            setIdRef(&idParam, 0.f);
#endif
            setIqRef(&iqParam, speedParam.out);
            setIdFdk(&idParam, svpwmParam.id);
            setIqFdk(&iqParam, svpwmParam.iq);
#else
            float speedRef = setSmcSpeedRef();
            smcCalculate(&smcParam, speedRef, encoderParam.motorSpeedRpm);

            float speed = encoderParam.motorSpeedRpm;

            uint32_t *pSpeed = (uint32_t *)&speed;

            usRegHoldingBuf[2] = (*pSpeed & 0xFFFF0000) >> 16;
            usRegHoldingBuf[3] = (*pSpeed & 0x0000FFFF);

            setIdRef(&idParam, 0.f);
            setIqRef(&iqParam, smcParam.output);
            setIdFdk(&idParam, svpwmParam.id);
            setIqFdk(&iqParam, svpwmParam.iq);
#endif
            currentPidCalculate();
            antiParkConvert();
            antiClarkConvert();
            svpwmCalculate();
            break;
        }
        default:
            break;
    }
}
