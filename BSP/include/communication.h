#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"
#include "stdint.h"
#include "usart.h"
#include "string.h"

/**
 * @brief 初始化通信模块
 */
void communicationInit(void);

/**
 * @brief 将一组float数据添加到准备发送的Vofa帧中。
 * @param data 要添加的float数组指针
 * @param numFloats 数组中float的数量
 * @return 0: 成功, -1: 帧缓冲区已满
 */
int8_t vofaAddFloatToFrame(const float *data, uint8_t numFloats);

/**
 * @brief 将当前帧缓冲区中的所有数据打包并通过DMA队列发送。
 * @param huart 使用的UART句柄
 * @note 此函数会清空内部的帧缓冲区。
 */
void sendData(UART_HandleTypeDef *huart);


#endif //COMMUNICATION_H