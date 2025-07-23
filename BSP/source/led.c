#include "led.h"

void led1Ctrl(const uint8_t state)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state == LED_ON ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void led2Ctrl(const uint8_t state)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state == LED_ON ? GPIO_PIN_RESET : GPIO_PIN_SET);
}