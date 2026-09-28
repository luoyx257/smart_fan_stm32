/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   串口接收环形缓冲实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
#include "usart.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    UART_HandleTypeDef *huart;              /* 对应 HAL 句柄 */
    uint8_t  buf[UART_RX_BUF_SIZE];         /* 环形缓冲 */
    volatile uint16_t head;                 /* 写指针(中断里改) */
    volatile uint16_t tail;                 /* 读指针(任务里改) */
    uint8_t  rx_byte;                       /* HAL 逐字节接收的落点 */
} UartRxCtx_t;

/* Private variables ---------------------------------------------------------*/
/* 注意: 这里的 rx_byte 会被 HAL_UART_Receive_IT 直接写入, 必须是静态存储 */
static UartRxCtx_t s_uart[UART_ID_MAX];

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  启动(或重启)逐字节接收
  * @note   F1 的 HAL 没有空闲中断 API, 因此用"每次收 1 字节 + 回调里重新挂载"
  *         的经典写法。每字节一个中断, 9600 波特率下约 1ms 一次, 开销可接受。
  */
static void Uart_StartReceive(UartId_t id)
{
    (void)HAL_UART_Receive_IT(s_uart[id].huart, &s_uart[id].rx_byte, 1U);
}

/* Exported functions --------------------------------------------------------*/

/** @brief 绑定句柄并启动接收 */
void BspUart_Init(void)
{
    s_uart[UART_ID_UI].huart    = &huart2;
    s_uart[UART_ID_BT].huart    = &huart3;
    s_uart[UART_ID_VOICE].huart = &huart1;

    {
        uint8_t i;
        for (i = 0U; i < (uint8_t)UART_ID_MAX; i++)
        {
            s_uart[i].head    = 0U;
            s_uart[i].tail    = 0U;
            s_uart[i].rx_byte = 0U;
            Uart_StartReceive((UartId_t)i);
        }
    }
}

/** @brief 中断收字节 */
void BspUart_RxIsrByte(UartId_t id, uint8_t byte)
{
    uint16_t next;

    if (id >= UART_ID_MAX)
    {
        return;
    }

    next = (uint16_t)((s_uart[id].head + 1U) % UART_RX_BUF_SIZE);

    if (next == s_uart[id].tail)
    {
        /* 缓冲满: 丢弃这个字节, 保持 tail 不动。
           宁可丢一个字节也不能覆盖未读数据, 否则解析器会拿到错乱帧。 */
        return;
    }

    s_uart[id].buf[s_uart[id].head] = byte;
    s_uart[id].head = next;
}

/** @brief 任务取字节 */
uint8_t BspUart_GetByte(UartId_t id, uint8_t *byte)
{
    if ((id >= UART_ID_MAX) || (byte == NULL))
    {
        return 0U;
    }

    if (s_uart[id].tail == s_uart[id].head)
    {
        return 0U;      /* 空 */
    }

    *byte = s_uart[id].buf[s_uart[id].tail];
    s_uart[id].tail = (uint16_t)((s_uart[id].tail + 1U) % UART_RX_BUF_SIZE);
    return 1U;
}

/** @brief 可读字节数 */
uint16_t BspUart_Available(UartId_t id)
{
    uint16_t head;
    uint16_t tail;

    if (id >= UART_ID_MAX)
    {
        return 0U;
    }

    head = s_uart[id].head;
    tail = s_uart[id].tail;

    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(UART_RX_BUF_SIZE - tail + head);
}

/** @brief 清空未读数据 */
void BspUart_Flush(UartId_t id)
{
    if (id >= UART_ID_MAX)
    {
        return;
    }
    s_uart[id].tail = s_uart[id].head;
}

/*------------------------------------------------------------------------------
 * HAL 弱函数回调 —— 三个串口的接收完成 / 错误处理统一放这里
 *----------------------------------------------------------------------------*/

/** @brief 接收完成: 存入环形缓冲并重新挂载接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance == USART2)
    {
        BspUart_RxIsrByte(UART_ID_UI, s_uart[UART_ID_UI].rx_byte);
        (void)HAL_UART_Receive_IT(&huart2, &s_uart[UART_ID_UI].rx_byte, 1U);
    }
    else if (huart->Instance == USART3)
    {
        BspUart_RxIsrByte(UART_ID_BT, s_uart[UART_ID_BT].rx_byte);
        (void)HAL_UART_Receive_IT(&huart3, &s_uart[UART_ID_BT].rx_byte, 1U);
    }
    else if (huart->Instance == USART1)
    {
        BspUart_RxIsrByte(UART_ID_VOICE, s_uart[UART_ID_VOICE].rx_byte);
        (void)HAL_UART_Receive_IT(&huart1, &s_uart[UART_ID_VOICE].rx_byte, 1U);
    }
    else
    {
        /* 未使用的串口 */
    }
}

/**
  * @brief  错误回调: 清标志后继续接收
  * @note   不重新挂载的话, 一次噪声导致的 ORE 错误会让该串口永久收不到数据,
  *         这是很常见又很难查的坑, 必须在这里恢复。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    /* 清掉溢出/帧错误标志, 然后重新挂载接收 */
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (huart->Instance == USART2)
    {
        (void)HAL_UART_Receive_IT(&huart2, &s_uart[UART_ID_UI].rx_byte, 1U);
    }
    else if (huart->Instance == USART3)
    {
        (void)HAL_UART_Receive_IT(&huart3, &s_uart[UART_ID_BT].rx_byte, 1U);
    }
    else if (huart->Instance == USART1)
    {
        (void)HAL_UART_Receive_IT(&huart1, &s_uart[UART_ID_VOICE].rx_byte, 1U);
    }
    else
    {
        /* 未使用的串口 */
    }
}
