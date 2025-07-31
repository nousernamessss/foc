/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"
#include "main.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "usart.h"
#include "port.h"
/* ----------------------- static functions ---------------------------------*/
static UCHAR ucRTUByte;
extern DMA_HandleTypeDef hdma_usart3_rx;
/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    (void)xRxEnable; // 避免编译器警告

    if (xTxEnable)
    {
        // 使能发送缓冲区空中断
        __HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);
    }
    else
    {
        // 禁止发送缓冲区空中断
        __HAL_UART_DISABLE_IT(&huart3, UART_IT_TXE);
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL bRet = TRUE;

    huart3.Instance = USART3;
    huart3.Init.BaudRate = ulBaudRate;

    if (ucDataBits == 8)
    {
        huart3.Init.WordLength = UART_WORDLENGTH_8B;
    }
    else
    {
        bRet = FALSE;
    }

    huart3.Init.StopBits = UART_STOPBITS_1;

    if (eParity == MB_PAR_NONE)
    {
        huart3.Init.Parity = UART_PARITY_NONE;
    }
    else if (eParity == MB_PAR_EVEN)
    {
        huart3.Init.Parity = UART_PARITY_EVEN;
    }
    else if (eParity == MB_PAR_ODD)
    {
        huart3.Init.Parity = UART_PARITY_ODD;
    }
    else
    {
        bRet = FALSE;
    }

    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        bRet = FALSE;
    }

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    if(HAL_UART_Receive_DMA(&huart3, s_ucMBARMRxBuf, MB_DMA_RX_BUF_SIZE) != HAL_OK)
    {
        return FALSE;
    }
    return bRet;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    huart3.Instance->TDR = (ucByte & (uint8_t)0xFF);
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    if (usMBARMRxReadPos < MB_DMA_RX_BUF_SIZE)
    {
        *pucByte = s_ucMBARMRxBuf[usMBARMRxReadPos];
        usMBARMRxReadPos++;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}

void USART3_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
    {
        uint16_t usRxDataLen;

        // 1. 清除空闲中断标志位（必须先清除，否则会连续不断进入中断）
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);

        // 2. 停止本次DMA传输
        HAL_UART_DMAStop(&huart3);

        // 3. 计算接收到的数据长度
        //    DMA的剩余传输长度寄存器(CNDTR)是递减的
        usRxDataLen = MB_DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

        // 4. 将接收到的所有字节送入FreeMODBUS协议栈
        //    重置读指针
        usMBARMRxReadPos = 0;
        //    循环调用协议栈的"接收到一字节"回调
        for(int i=0; i < usRxDataLen; i++)
        {
            // 这个函数内部会调用 xMBPortSerialGetByte 来取走数据
            pxMBFrameCBByteReceived();
        }

        // 5. 重新启动DMA，准备接收下一帧数据
        HAL_UART_Receive_DMA(&huart3, s_ucMBARMRxBuf, MB_DMA_RX_BUF_SIZE);
    }

    if(__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_TXE)!= RESET)
    {
        pxMBFrameCBTransmitterEmpty();
    }

    //HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */

    /* USER CODE END USART3_IRQn 1 */
}




