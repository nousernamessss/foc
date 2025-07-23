#ifndef LED_H
#define LED_H

#include "stm32g4xx.h"
#include "main.h"

#define LED_ON  1
#define LED_OFF 0

typedef enum
{
    LED1 = 0,
    LED2 = 1,
    LEDMAX = 2
}eledNum;

void led1Ctrl(const uint8_t state);
void led2Ctrl(const uint8_t state);
void led1Toggle(void);
void led2Toggle(void);

#endif //LED_H
