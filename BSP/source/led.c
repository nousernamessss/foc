#include "led.h"

void led1Ctrl(const uint8_t state)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state == LED_ON ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void led2Ctrl(const uint8_t state)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state == LED_ON ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void led1Toggle(void)
{
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void led2Toggle(void)
{
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}