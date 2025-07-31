#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"
#include "stdint.h"
#include "usart.h"
#include "string.h"


typedef struct
{
    uint8_t sendDataNums;
    uint8_t frameTail[4];
    uint16_t frameSize;
    uint8_t txBuffer[16];
}tcommunicationParam;

extern tcommunicationParam communicationParam;



void communicationInit(void);
void sendData(UART_HandleTypeDef *huart, const float *txBuf);

#endif //COMMUNICATION_H
