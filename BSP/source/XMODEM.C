/**
 * @file    xmodem.c
 * @brief   XMODEM-CRC 接收方协议实现
 * @note    此文件依赖于STM32 HAL库，用于串口通信和Flash操作。
 */

#include "main.h" // 假设包含了所有HAL库头文件和外设句柄(如huart3)
#include <string.h>

#include "usart.h"

// --- XMODEM 控制字符定义 ---
#define SOH         (0x01)  // Start of Header (128-byte packet)
#define EOT         (0x04)  // End of Transmission
#define ACK         (0x06)  // Acknowledge
#define NAK         (0x15)  // Negative Acknowledge
#define CAN         (0x18)  // Cancel
#define CRC_C       ('C')   // 'C' character to start CRC-16 mode

// --- XMODEM 协议参数定义 ---
#define XMODEM_PACKET_SIZE          (128)   // 数据包中核心数据的大小
#define XMODEM_PACKET_HEADER_SIZE   (3)     // 包头大小 (SOH + PKG_NUM + N_PKG_NUM)
#define XMODEM_PACKET_CRC_SIZE      (2)     // CRC校验的大小
#define XMODEM_PACKET_TOTAL_SIZE    (XMODEM_PACKET_HEADER_SIZE + XMODEM_PACKET_SIZE + XMODEM_PACKET_CRC_SIZE)

#define XMODEM_MAX_RETRIES          (10)    // 最大重试次数
#define XMODEM_TIMEOUT_MS           (1000)  // 接收一个字节的超时时间 (ms)
#define XMODEM_START_TIMEOUT_S      (60)    // 等待开始传输的总超时时间 (s)


// --- Flash 存储定义 (需要根据您的MCU型号和分区进行修改) ---
#define FLASH_OTA_TEMP_AREA_START   (0x08040000) // 示例: OTA固件临时存储区的起始地址
#define FLASH_OTA_TEMP_AREA_SIZE    (256 * 1024)   // 示例: 256KB大小
// 您需要一个函数来擦除Flash扇区
// extern void Flash_EraseSector(uint32_t address, uint32_t size);
// 您需要一个函数来向Flash写入数据
// extern HAL_StatusTypeDef Flash_Write(uint32_t address, uint8_t* data, uint32_t len);


/**
 * @brief  通过串口发送单个字节
 * @param  byte 要发送的字节
 */
static void _xmodem_send_byte(uint8_t byte)
{
    HAL_UART_Transmit(&huart3, &byte, 1, XMODEM_TIMEOUT_MS);
}

/**
 * @brief  通过串口接收单个字节（带超时）
 * @param  byte 指向接收数据缓冲区的指针
 * @return HAL_StatusTypeDef HAL_OK表示成功, HAL_TIMEOUT表示超时
 */
static HAL_StatusTypeDef _xmodem_receive_byte(uint8_t *byte)
{
    return HAL_UART_Receive(&huart3, byte, 1, XMODEM_TIMEOUT_MS);
}

/**
 * @brief  计算16位CRC校验码
 * @param  ptr 数据指针
 * @param  count 数据长度
 * @return uint16_t 计算出的CRC值
 */
static uint16_t _crc16_update(uint16_t crc, uint8_t data)
{
    crc = crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x8000)
        {
            crc = (crc << 1) ^ 0x1021;
        }
        else
        {
            crc = crc << 1;
        }
    }
    return crc;
}

static uint16_t _calculate_crc16(const uint8_t *ptr, int count)
{
    uint16_t crc = 0;
    while (count-- > 0)
    {
        crc = _crc16_update(crc, *ptr++);
    }
    return crc;
}


/**
 * @brief  接收一个完整的XMODEM数据包
 * @param  buffer 用于存放128字节核心数据的缓冲区
 * @param  packet_number 期望接收的包序号
 * @return int 0=成功, -1=失败/超时, 1=收到EOT, 2=收到CAN
 */
static int _receive_packet(uint8_t *buffer, uint8_t packet_number)
{
    uint8_t header[XMODEM_PACKET_HEADER_SIZE];
    uint8_t crc_bytes[XMODEM_PACKET_CRC_SIZE];
    uint16_t received_crc, calculated_crc;
    HAL_StatusTypeDef status;

    // 1. 接收包头 (SOH)
    status = _xmodem_receive_byte(&header[0]);
    if (status != HAL_OK) return -1; // 超时

    if (header[0] == EOT) return 1; // 传输结束
    if (header[0] == CAN) return 2; // 传输取消
    if (header[0] != SOH) return -1; // 不是一个有效的数据包

    // 2. 接收包序号和反码
    if (_xmodem_receive_byte(&header[1]) != HAL_OK) return -1;
    if (_xmodem_receive_byte(&header[2]) != HAL_OK) return -1;

    // 3. 接收128字节数据
    if (HAL_UART_Receive(&huart3, buffer, XMODEM_PACKET_SIZE, XMODEM_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    // 4. 接收2字节CRC
    if (_xmodem_receive_byte(&crc_bytes[0]) != HAL_OK) return -1;
    if (_xmodem_receive_byte(&crc_bytes[1]) != HAL_OK) return -1;

    // 5. 校验数据包
    // 校验包序号
    if (header[1] != packet_number || header[2] != (uint8_t)(~packet_number))
    {
        // 包序号错误，可能是重复的包，也可能是乱序，直接忽略
        // 发送NAK让对方重传
        return -1;
    }

    // 校验CRC
    received_crc = ((uint16_t)crc_bytes[0] << 8) | crc_bytes[1];
    calculated_crc = _calculate_crc16(buffer, XMODEM_PACKET_SIZE);
    if (received_crc != calculated_crc)
    {
        return -1; // CRC校验失败
    }

    // 所有校验通过
    return 0;
}

/**
 * @brief  执行XMODEM固件接收流程
 * @note   此函数是一个阻塞函数，成功或失败后会通过软件复位来重启系统。
 */
void ReceiveFirmware_XMODEM(void)
{
    uint8_t packet_buffer[XMODEM_PACKET_SIZE];
    uint8_t current_packet_number = 1;
    uint8_t retries = 0;
    int packet_status;
    uint32_t flash_write_address = FLASH_OTA_TEMP_AREA_START;

    // --- 1. 准备阶段: 擦除Flash并请求开始传输 ---

    // 擦除用于OTA的Flash区域 (此函数需要您自己实现)
    // Flash_EraseSector(FLASH_OTA_TEMP_AREA_START, FLASH_OTA_TEMP_AREA_SIZE);

    // 循环发送 'C' 请求开始传输，直到收到第一个包或超时
    for (int i = 0; i < XMODEM_START_TIMEOUT_S; ++i)
    {
        _xmodem_send_byte(CRC_C);
        // 尝试接收第一个数据包
        packet_status = _receive_packet(packet_buffer, current_packet_number);
        if (packet_status == 0) // 成功收到第一个包
        {
            goto packet_received; // 跳转到处理流程
        }
    }
    // 如果60秒内没有收到任何有效数据，则超时失败，重启
    NVIC_SystemReset();


    // --- 2. 数据传输主循环 ---
    while(1)
    {
        packet_status = _receive_packet(packet_buffer, current_packet_number);

    packet_received: // 使用goto来处理第一次接收的特殊情况
        if (packet_status == 0) // 成功接收到一个有效数据包
        {
            // 将数据写入Flash (此函数需要您自己实现)
            // if (Flash_Write(flash_write_address, packet_buffer, XMODEM_PACKET_SIZE) == HAL_OK)
            // {
                flash_write_address += XMODEM_PACKET_SIZE;
                current_packet_number++;
                retries = 0;
                _xmodem_send_byte(ACK); // 发送确认
            // }
            // else // Flash写入失败，这是严重错误
            // {
            //     _xmodem_send_byte(CAN); // 发送取消信号
            //     _xmodem_send_byte(CAN);
            //     NVIC_SystemReset(); // 重启
            // }
        }
        else if (packet_status == 1) // 收到 EOT
        {
            // --- 3. 结束阶段 ---
            _xmodem_send_byte(ACK); // 对EOT进行最终确认

            // 在这里可以添加对整个固件的最终校验逻辑 (例如，校验文件大小或尾部的magic word)

            // 校验通过后，写入升级成功标志 (需要您自己实现Flash写操作)
            // uint32_t success_flag = 0xABCD1234;
            // Flash_Write(UPDATE_FLAG_ADDRESS, (uint8_t*)&success_flag, 4);

            // 延时一小段时间确保ACK发送完成
            HAL_Delay(100);
            // 软件复位，Bootloader将接管后续工作
            NVIC_SystemReset();
        }
        else if (packet_status == 2) // 对方取消了传输
        {
            NVIC_SystemReset();
        }
        else // 接收失败 (超时或校验错误)
        {
            retries++;
            if (retries > XMODEM_MAX_RETRIES)
            {
                // 连续失败次数过多，中止传输
                _xmodem_send_byte(CAN);
                _xmodem_send_byte(CAN);
                NVIC_SystemReset();
            }
            else
            {
                // 请求重传
                _xmodem_send_byte(NAK);
            }
        }
    }
}
