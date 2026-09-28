/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   三个 USART 的接收环形缓冲(中断收字节 -> 任务取字节)
  * @note    为什么不用消息队列传字节:
  *          xQueueCmd 每项 8 字节, 若把每个串口字节都塞进去, 一帧 10 字节的
  *          命令就要占 10 项, 很容易把队列打满并把真正的控制命令挤掉。
  *          因此收发用轻量环形缓冲, 只有"解析完成的控制命令"才进队列。
  *          中断里只做 memcpy + 指针自增, 不做解析, 保证 ISR 足够短。
  ******************************************************************************
  */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 逻辑串口编号 */
typedef enum {
    UART_ID_UI = 0,     /* USART2 - 串口屏 TJC4827X243_011C, 115200 */
    UART_ID_BT,         /* USART3 - HC-05 蓝牙, 9600 */
    UART_ID_VOICE,      /* USART1 - SU-03T 语音, 9600 */
    UART_ID_MAX
} UartId_t;

/** 每个串口的接收缓冲大小(可被工程配置覆盖) */
#ifndef UART_RX_BUF_SIZE
#define UART_RX_BUF_SIZE    256U
#endif

/**
  * @brief  初始化三个串口的接收中断
  * @note   必须在 osKernelStart() 之前调用, 且要在 MX_USARTx_UART_Init() 之后
  */
void BspUart_Init(void);

/**
  * @brief  中断上下文: 存入一个收到的字节
  * @note   只由 HAL_UART_RxCpltCallback 调用
  */
void BspUart_RxIsrByte(UartId_t id, uint8_t byte);

/** @brief 任务上下文: 取一个字节, 无数据返回 0 */
uint8_t BspUart_GetByte(UartId_t id, uint8_t *byte);

/** @brief 当前缓冲区里还有多少字节未读 */
uint16_t BspUart_Available(UartId_t id);

/** @brief 丢弃缓冲区里所有未读数据(解析出错重置帧时用) */
void BspUart_Flush(UartId_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
