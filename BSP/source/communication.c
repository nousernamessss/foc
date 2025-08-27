#include "communication.h"

// --- Vofa+ 帧聚合缓冲区 ---
#define VOFA_FRAME_MAX_FLOATS   32      // 一个Vofa数据帧中最多可容纳的float数量
#define VOFA_FRAME_MAX_BYTES    (VOFA_FRAME_MAX_FLOATS * 4)
static float vofaFrameBuffer[VOFA_FRAME_MAX_FLOATS];    // 用于聚合float数据的缓冲区
static uint8_t vofaFloatCount = 0;                      // 当前缓冲区中的float数量

// --- Vofa+ 帧尾 ---
static const uint8_t VOFA_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7f};

// --- 后台DMA发送队列 (用于解决高频调用冲突) ---
#define TX_QUEUE_SIZE           8       // 发送队列的槽位数量
#define TX_BUFFER_SIZE          (VOFA_FRAME_MAX_BYTES + 4) // 发送缓冲区的最大尺寸
static uint8_t  txQueue[TX_QUEUE_SIZE][TX_BUFFER_SIZE]; // 队列，存放完整的待发送数据包
static uint16_t txLengths[TX_QUEUE_SIZE];               // 队列，存放每个包的长度
static volatile uint8_t txHead = 0;
static volatile uint8_t txTail = 0;

// 内部函数，处理后台发送队列
static void processTxQueue(UART_HandleTypeDef *huart)
{
    // 进入临界区，防止中断和主程序同时操作队列
    __disable_irq();

    if ((huart->gState == HAL_UART_STATE_READY) && (txHead != txTail))
    {
        uint8_t currentTail = txTail;
        txTail = (txTail + 1) % TX_QUEUE_SIZE;

        // 离开临界区前启动DMA
        HAL_UART_Transmit_DMA(huart, txQueue[currentTail], txLengths[currentTail]);
    }

    __enable_irq();
}

// 初始化函数
void communicationInit(void)
{
    vofaFloatCount = 0;
    txHead = 0;
    txTail = 0;
}

// 将数据添加到帧缓冲区
int8_t vofaAddFloatToFrame(const float *data, uint8_t numFloats)
{
    if (vofaFloatCount + numFloats > VOFA_FRAME_MAX_FLOATS)
    {
        return -1; // 缓冲区空间不足
    }

    memcpy(&vofaFrameBuffer[vofaFloatCount], data, numFloats * sizeof(float));
    vofaFloatCount += numFloats;
    return 0;
}

// 发送整个帧
void sendData(UART_HandleTypeDef *huart)
{
    if (vofaFloatCount == 0) // 如果帧中没有数据，则不发送
    {
        return;
    }

    uint8_t nextHead = (txHead + 1) % TX_QUEUE_SIZE;
    if (nextHead == txTail) // 后台发送队列已满，丢弃本次数据
    {
        vofaFloatCount = 0; // 清空缓冲区，准备下一次
        return;
    }

    // 1. 准备数据包：拷贝所有floats
    uint16_t floatBytes = vofaFloatCount * 4;
    memcpy(txQueue[txHead], vofaFrameBuffer, floatBytes);

    // 2. 准备数据包：拷贝帧尾
    memcpy(&txQueue[txHead][floatBytes], VOFA_FRAME_TAIL, 4);

    // 3. 记录这个包的总长度
    txLengths[txHead] = floatBytes + 4;

    // 4. 更新队列头指针
    txHead = nextHead;

    // 5. 清空Vofa帧缓冲区计数，为下一次聚合做准备
    vofaFloatCount = 0;

    // 6. 触发后台发送任务
    processTxQueue(huart);
}

/**
 * @brief UART发送完成回调函数
 * @note 请确保此函数在您的项目中被定义(例如在main.c或stm32fxxx_it.c中)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 检查是否是您使用的UART, 例如USART3
    if (huart->Instance == USART3)
    {
        processTxQueue(huart);
    }
}