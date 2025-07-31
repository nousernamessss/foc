#include "communication.h"

#include "led.h"

tcommunicationParam communicationParam;

void communicationInit(void)
{
    communicationParam.sendDataNums = 3;
    communicationParam.frameSize = communicationParam.sendDataNums * 4 + 4;

    communicationParam.frameTail[0] = 0x00;
    communicationParam.frameTail[1] = 0x00;
    communicationParam.frameTail[2] = 0x80;
    communicationParam.frameTail[3] = 0x7f;

    memset(communicationParam.txBuffer, 0, sizeof(communicationParam.txBuffer));

}

void sendData(UART_HandleTypeDef *huart, const float *txBuf)
{
    for (uint8_t i = 0; i < communicationParam.sendDataNums; i++)
    {
        memcpy(&(communicationParam.txBuffer[i * 4]), &txBuf[i], sizeof(float));
    }

    memcpy(&communicationParam.txBuffer[communicationParam.sendDataNums * 4], communicationParam.frameTail, 4);
    HAL_UART_Transmit_DMA(huart, communicationParam.txBuffer, communicationParam.frameSize);
}







