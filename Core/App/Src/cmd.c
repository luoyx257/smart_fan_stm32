/**
  ******************************************************************************
  * @file    cmd.c
  * @brief   统一命令队列封装
  * @note    队列句柄 xQueueCmdHandle 由 CubeMX 生成的 freertos.c 创建,
  *          尺寸 16 项 x 8 字节, 与 CmdMsg_t 完全匹配。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cmd.h"
#include "cmsis_os.h"

/* Private constants ---------------------------------------------------------*/
#define CMD_WAIT_FOREVER    (0xFFFFFFFFUL)

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  投递命令
  * @note   只在任务上下文使用。三个 UI 任务都会调用, 用普通 osMessageQueuePut
  *         即可(osMessageQueuePut 内部已做临界区保护)。
  */
uint8_t Cmd_Post(uint8_t cmd, uint16_t param, uint8_t source)
{
    CmdMsg_t msg;

    if ((xQueueCmdHandle == NULL) || (cmd == (uint8_t)CMD_NONE))
    {
        return 0U;
    }

    msg.cmd    = cmd;
    msg.source = source;
    msg.param  = param;
    msg.tick   = (uint32_t)osKernelGetTickCount();

    /* 超时 0 = 不等待, 队列满直接丢弃(避免 UI 任务被阻塞) */
    if (osMessageQueuePut(xQueueCmdHandle, &msg, 0U, 0U) != osOK)
    {
        g_state.stat.cmd_drop++;
        return 0U;
    }

    g_state.stat.cmd_rx++;
    return 1U;
}

/**
  * @brief  阻塞取命令
  * @param  msg        输出参数
  * @param  timeout_ms 超时毫秒; CMD_WAIT_FOREVER 表示永久阻塞等待
  */
uint8_t Cmd_Fetch(CmdMsg_t *msg, uint32_t timeout_ms)
{
    if ((msg == NULL) || (xQueueCmdHandle == NULL))
    {
        return 0U;
    }

    /* CMSIS-RTOS v2: osWaitForever == 0xFFFFFFFF, 与 CMD_WAIT_FOREVER 一致 */
    if (osMessageQueueGet(xQueueCmdHandle, msg, NULL, timeout_ms) != osOK)
    {
        return 0U;
    }
    return 1U;
}
