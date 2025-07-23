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

typedef struct
{
    FDCAN_TxHeaderTypeDef txHeader; // FDCAN发送头，用于配置ID、数据长度等
    uint8_t txData[8];              // 发送数据缓冲区，大小为8字节（CAN帧最大长度）
} tCanCommunication;
extern tCanCommunication canComm;


void communicationInit(void);
void CAN_Communication_Init(void);
void sendData(UART_HandleTypeDef *huart, const float *txBuf);
HAL_StatusTypeDef CAN_SendFloat(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, float data);
#endif //COMMUNICATION_H
