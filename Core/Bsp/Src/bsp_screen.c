/**
  ******************************************************************************
  * @file    bsp_screen.c
  * @brief   串口屏底层实现
  * @note    每条指令单独组包发送, 不跨函数累积状态, 这样调用顺序不会影响结果。
  *          指令格式: "对象.属性=值" + 0xFF 0xFF 0xFF
  *
  *          【并发约定】本文件所有发送函数都不是线程安全的, 也刻意不用临界区或
  *          互斥量保护。原因: HAL_UART_Transmit 是阻塞发送, 一帧几十字节在
  *          115200 下要 1~5ms, 若用 taskENTER_CRITICAL() 包住会关中断数毫秒,
  *          期间 SysTick / TIM4 都不走, HAL 自己的超时计数也失效(极端情况直接
  *          死等), 风险比"字节交错"大得多。
  *          因此约定: 只有 Task_Uart_UI 真正调用发送函数; 其它任务需要刷新屏幕
  *          时调用 AppUi_RequestReport() 打标记, 由 Task_Uart_UI 下一拍统一发送。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_screen.h"
#include "usart.h"

/* Private define ------------------------------------------------------------*/
#define SCREEN_UART             (&huart2)
#define SCREEN_TX_TIMEOUT_MS    100U
#define SCREEN_CMD_MAX          48U     /* 单条指令最大长度(含 0xFF*3 与结束符) */

/* Private function prototypes -----------------------------------------------*/
static void Screen_SendCommand(const char *cmd);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  发送一条指令(自动补 3 个 0xFF 结束符), 并做串行化
  * @param  cmd 以 '\0' 结尾的指令主体, 长度不得超过 SCREEN_CMD_MAX - 4
  */
static void Screen_SendCommand(const char *cmd)
{
    uint8_t  frame[SCREEN_CMD_MAX];
    uint16_t len = 0U;

    if (cmd == NULL)
    {
        return;
    }

    while ((cmd[len] != '\0') && (len < (SCREEN_CMD_MAX - 3U)))
    {
        frame[len] = (uint8_t)cmd[len];
        len++;
    }

    frame[len++] = 0xFFU;
    frame[len++] = 0xFFU;
    frame[len++] = 0xFFU;

    (void)HAL_UART_Transmit(SCREEN_UART, frame, len, SCREEN_TX_TIMEOUT_MS);
}

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化 */
void Screen_Init(void)
{
    /* 目前无需特殊初始化, 保留接口便于以后加"复位屏幕"指令 */
}

/** @brief 直接发送字符串(自动补结束符) */
void Screen_Send(const char *str)
{
    Screen_SendCommand(str);
}

/** @brief 发送任意原始数据(不加结束符) */
void Screen_SendBuf(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }
    (void)HAL_UART_Transmit(SCREEN_UART, (uint8_t *)data, len, SCREEN_TX_TIMEOUT_MS);
}

/** @brief 发送 3 个 0xFF 结束符 */
void Screen_SendEnd(void)
{
    uint8_t end3[3];
    end3[0] = 0xFFU;
    end3[1] = 0xFFU;
    end3[2] = 0xFFU;
    Screen_SendBuf(end3, 3U);
}

/**
  * @brief  写数字: "obj.val=123"
  * @note   数值转字符串用手写循环, 不引入 printf(省 Flash, 也避免浮点库)
  */
void Screen_SetNumber(const char *obj, int32_t value)
{
    char     cmd[SCREEN_CMD_MAX];
    uint8_t  pos = 0U;
    uint8_t  nlen = 0U;
    uint8_t  i;
    char     numbuf[12];
    uint32_t mag;

    if (obj == NULL)
    {
        return;
    }

    while ((obj[pos] != '\0') && (pos < 20U))
    {
        cmd[pos] = obj[pos];
        pos++;
    }

    cmd[pos++] = '.';
    cmd[pos++] = 'v';
    cmd[pos++] = 'a';
    cmd[pos++] = 'l';
    cmd[pos++] = '=';

    if (value < 0)
    {
        cmd[pos++] = '-';
        mag = (uint32_t)(-value);
    }
    else
    {
        mag = (uint32_t)value;
    }

    if (mag == 0U)
    {
        numbuf[nlen++] = '0';
    }
    else
    {
        while ((mag > 0U) && (nlen < sizeof(numbuf)))
        {
            numbuf[nlen++] = (char)('0' + (mag % 10U));
            mag /= 10U;
        }
    }
    for (i = 0U; i < nlen; i++)
    {
        cmd[pos++] = numbuf[nlen - 1U - i];
    }
    cmd[pos] = '\0';

    Screen_SendCommand(cmd);
}

/** @brief 写文本: "obj.txt=\"abc\"" */
void Screen_SetText(const char *obj, const char *text)
{
    char    cmd[SCREEN_CMD_MAX];
    uint8_t pos = 0U;

    if ((obj == NULL) || (text == NULL))
    {
        return;
    }

    while ((obj[pos] != '\0') && (pos < 20U))
    {
        cmd[pos] = obj[pos];
        pos++;
    }

    cmd[pos++] = '.';
    cmd[pos++] = 't';
    cmd[pos++] = 'x';
    cmd[pos++] = 't';
    cmd[pos++] = '=';
    cmd[pos++] = '"';

    while ((*text != '\0') && (pos < (SCREEN_CMD_MAX - 2U)))
    {
        cmd[pos++] = *text++;
    }
    cmd[pos++] = '"';
    cmd[pos]   = '\0';

    Screen_SendCommand(cmd);
}

/**
  * @brief  切换页面: 发送 "page N" + 结束符
  * @note   这里只支持单数字页号(0~9), 需要更多页请改成拼两位数字
  */
void Screen_GotoPage(uint8_t page_id)
{
    char cmd[8];

    cmd[0] = 'p';
    cmd[1] = 'a';
    cmd[2] = 'g';
    cmd[3] = 'e';
    cmd[4] = ' ';
    cmd[5] = (char)('0' + (char)(page_id % 10U));
    cmd[6] = '\0';

    Screen_SendCommand(cmd);
}

/**
  * @brief  兼容接口: 本实现每条指令即时发送, 无需提交
  */
void Screen_Commit(void)
{
    /* 空实现: 见 bsp_screen.h 说明 */
}
