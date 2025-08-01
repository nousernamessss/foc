#include "if.h"

#include <stdlib.h>

#include "communication.h"
#include "currentPid.h"
#include "key.h"
#include "led.h"
#include "SMO.h"
#include "tim.h"
#include "speedPid.h"

const float prePositionTime = 1.f;
const float rampUpTime = 1.f;

const float targetSpeed = 500.f;
const float targetSpeedRad = targetSpeed * 4.f * PI / 30.f;
const float acceleration = targetSpeedRad / rampUpTime;

static eMotorState motor_state = MOTOR_STATE_STOP;
static float state_counter = 0;
static float currentSpeed = 0.f;

static float absf(float value)
{
    if (value <= 0.f)
        return -value;
    else
        return value;
}

void ifStep(void)
{
    if (motorParam.runFlag == 1 && motor_state == MOTOR_STATE_STOP)
    {
        // 从停止状态切换到预定位状态
        motor_state = MOTOR_STATE_PRE_POSITION;
        state_counter = 0; // 重置状态计时器
    }

    else if (motorParam.runFlag == 0 && motor_state != MOTOR_STATE_STOP)
    {
        // 从任何运行状态切换到停止状态
        motor_state = MOTOR_STATE_STOP;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    }

    if (motor_state == MOTOR_STATE_STOP)
        return; // 停止状态下不执行任何FOC代码

    switch (motor_state)
    {
        case MOTOR_STATE_PRE_POSITION:
        {
#ifdef NOT_USE_HFI
            svpwmParam.theta = 0.f;
            svpwmParam.ud = -1.f;
            svpwmParam.uq = 0.f;
            antiParkConvert();
            antiClarkConvert();
            svpwmCalculate();
            state_counter++;

            if (state_counter >= prePositionTime * 10000)
            {
                motor_state = MOTOR_STATE_RAMP_UP;
                state_counter = 0;
            }
            break;
#else
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
                        hfiParam.polarityCorrectionAngleRad = PI; // 方向相反，补偿180度
                    }

                    float initial_angle_rad = hfiParam.estimatedAngleRad + hfiParam.polarityCorrectionAngleRad;

                    // --- 辨识完成 ---
                    motor_state = MOTOR_STATE_RAMP_UP; // 切换到下一个主状态
                    state_counter = 0;

                    // 【关键】用我们辨识出的角度来初始化爬坡状态的角度
                    svpwmParam.theta = initial_angle_rad;
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
#endif
        }

        case MOTOR_STATE_RAMP_UP:
        {
            currentSpeed += acceleration * 0.0001f;
            svpwmParam.theta += currentSpeed * 0.0001f;
            if (svpwmParam.theta > 6.28318548F)
            {
                svpwmParam.theta -= 6.28318548F;
            }
            else if (svpwmParam.theta < 0.0F)
            {
                svpwmParam.theta += 6.28318548F;
            }

            clarkConvert();
            parkConvert();

            setIdRef(&idParam, 0.f);
            setIqRef(&iqParam, 4.f);
            setIdFdk(&idParam, svpwmParam.id);
            setIqFdk(&iqParam, svpwmParam.iq);

            currentPidCalculate();
            antiParkConvert();
            antiClarkConvert();
            svpwmCalculate();

            state_counter++;
            Estimator_Step(&SMO_Controller, svpwmParam.ualpha, svpwmParam.ubeta, svpwmParam.ialpha, svpwmParam.ibeta);
            if (state_counter >= rampUpTime * 10000)
            {
                motor_state = MOTOR_INCRESE_IQ;
            }
            break;
        }

        case MOTOR_INCRESE_IQ:
        {
            svpwmParam.theta += targetSpeedRad * 0.0001f;
            if (svpwmParam.theta > 6.28318548F)
            {
                svpwmParam.theta -= 6.28318548F;
            }
            else if (svpwmParam.theta < 0.0F)
            {
                svpwmParam.theta += 6.28318548F;
            }
            clarkConvert();
            parkConvert();

            setIdRef(&idParam, 0.f);
            setIqRef(&iqParam, 4.f - state_counter / 10000.f);
            setIdFdk(&idParam, svpwmParam.id);
            setIqFdk(&iqParam, svpwmParam.iq);

            currentPidCalculate();
            antiParkConvert();
            antiClarkConvert();
            svpwmCalculate();
            state_counter++;
            Estimator_Step(&SMO_Controller, svpwmParam.ualpha, svpwmParam.ubeta, svpwmParam.ialpha, svpwmParam.ibeta);
            if (absf(svpwmParam.theta - SMO_Controller.theta_estimated) < 5.f * PI / 180.f)
            {
                motor_state = MOTOR_STATE_RUN;
                state_counter = 0;
            }
            break;
        }

        case MOTOR_STATE_RUN:
        {
            led1Ctrl(LED_OFF);
            Estimator_Step(&SMO_Controller, svpwmParam.ualpha, svpwmParam.ubeta, svpwmParam.ialpha, svpwmParam.ibeta);
            svpwmParam.theta = SMO_Controller.theta_estimated;

            clarkConvert();
            parkConvert();

            setSpeedRef(&speedParam, targetSpeed + i * 100.f);
            setSpeedFdk(&speedParam, SMO_Controller.omega_filtered / 4.f * 30.f / PI);

            speedPidCalculate();
            setIdRef(&idParam, 0.f);
            setIqRef(&iqParam, speedParam.out);
            setIdFdk(&idParam, svpwmParam.id);
            setIqFdk(&iqParam, svpwmParam.iq);

            currentPidCalculate();
            antiParkConvert();
            antiClarkConvert();
            svpwmCalculate();
        }
    }
}
