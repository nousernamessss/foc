#include "key.h"

#include "main.h"
#include "speedPid.h"
#include "tim.h"

int i = 0;
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
    }

    else if(GPIO_Pin == Button2_Pin)
    {
        motorParam.runFlag = 1;
        i++;
    }

    else if(GPIO_Pin == Button3_Pin)
    {
        i--;
    }
}
