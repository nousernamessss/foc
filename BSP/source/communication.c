#include "communication.h"

#include "led.h"

tcommunicationParam communicationParam;
tCanCommunication canComm;
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

void CAN_Communication_Init(void)
{
    // 配置发送报文头的固定参数
    canComm.txHeader.IdType = FDCAN_STANDARD_ID;
    canComm.txHeader.TxFrameType = FDCAN_DATA_FRAME;
    canComm.txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    canComm.txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    canComm.txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    canComm.txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    canComm.txHeader.MessageMarker = 0;

    // 因为我们设计的函数是发送一个浮点数（4字节），所以数据长度固定为4
    canComm.txHeader.DataLength = FDCAN_DLC_BYTES_4;
}

HAL_StatusTypeDef CAN_SendFloat(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, float data)
{
    // 1. 设置本次发送的CAN ID
    canComm.txHeader.Identifier = can_id;

    // 2. 将浮点数数据拷贝到发送缓冲区
    memcpy(canComm.txData, &data, sizeof(float));

    // 3. 等待FDCAN的发送FIFO有可用空间
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
    {
        // 如果FIFO已满，可以等待或进行超时处理
    }

    led1Ctrl(LED_OFF);
    led2Ctrl(LED_OFF);
    // 4. 调用HAL库函数，将报文添加到发送FIFO中
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &canComm.txHeader, canComm.txData);
}
