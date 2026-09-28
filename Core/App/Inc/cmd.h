/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   统一控制命令定义 —— 串口屏 / 蓝牙 / 语音 三个入口共用
  * @note    所有 UI 来源解析出的命令都投递到同一个 xQueueCmd,
  *          由 Task_MotorFan 单点消费, 避免多处并发改状态。
  ******************************************************************************
  */

#ifndef __CMD_H
#define __CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "app_state.h"

/*------------------------------------------------------------------------------
 * CubeMX 生成的 RTOS 对象句柄(定义在 freertos.c)
 * 这里用原生 FreeRTOS 类型声明, 免得每个 App 模块都要 include cmsis_os.h。
 * 本工程只用到两个消息队列, 不在中断里操作任何内核对象。
 *----------------------------------------------------------------------------*/
extern QueueHandle_t xQueueCmdHandle;           /* xQueueCmd, 元素 8 字节 x 16 */
extern QueueHandle_t xQueueSensorDataHandle;    /* xQueueSensorData, 8 字节 x 8 */

/*------------------------------------------------------------------------------
 * 命令类型
 *----------------------------------------------------------------------------*/
typedef enum {
    CMD_NONE = 0x00,

    CMD_FAN_ON,             /* 手动开风扇(param 忽略, 用当前档位, 0 档则提到 1 档) */
    CMD_FAN_OFF,            /* 手动关风扇 */
    CMD_FAN_TOGGLE,         /* 风扇开关取反 */
    CMD_LEVEL_UP,           /* 档位 +1, 到 5 档后保持(不循环) */
    CMD_LEVEL_DOWN,         /* 档位 -1 */

    CMD_MODE_SET,           /* param: FanMode_t (0=自动 1=手动) */
    CMD_MODE_TOGGLE,        /* 自动/手动切换 */
    CMD_TEMP_SET,           /* param: 温度阈值 ℃, 限幅 TEMP_THRESHOLD_MIN~MAX */

    CMD_COUNTDOWN_SET,      /* param: 倒计时秒数, 0 表示取消倒计时 */
    CMD_COUNTDOWN_CANCEL,   /* 取消倒计时, 不影响风扇 */

    CMD_SWING_SET,          /* param: SwingState_t (0=不摇头 1=摇头) */
    CMD_SWING_TOGGLE,       /* 摇头开关取反 */

    CMD_LEVEL_SET,          /* param: 直接设定档位 0~5 (串口屏滑块用) */

    CMD_PID_LEVEL,          /* param: PID 运算得到的档位(仅自动模式有效, 内部命令) */

    CMD_QUERY_STATUS,       /* 请求上报一次状态(给串口屏初始化用) */
    CMD_MAX
} CmdId_t;

/*------------------------------------------------------------------------------
 * 命令来源(仅用于调试/日志)
 *----------------------------------------------------------------------------*/
typedef enum {
    SRC_NONE = 0,
    SRC_UART_UI,        /* USART2 串口屏 */
    SRC_BLUETOOTH,      /* USART3 HC-05 */
    SRC_VOICE,          /* USART1 SU-03T */
    SRC_INTERNAL        /* 任务内部自己产生(例如自动模式自动开关) */
} CmdSource_t;

/*------------------------------------------------------------------------------
 * 接口
 *----------------------------------------------------------------------------*/

/**
  * @brief  投递一条命令到 xQueueCmd
  * @param  cmd    命令类型
  * @param  param  命令参数
  * @param  source 来源
  * @retval 1 = 入队成功, 0 = 队列满被丢弃
  * @note   本函数可在任务上下文调用; 不要在中断里调用(中断里用带 FromISR 的版本)
  */
uint8_t Cmd_Post(uint8_t cmd, uint16_t param, uint8_t source);

/**
  * @brief  从 xQueueCmd 取一条命令(阻塞)
  * @param  msg        输出
  * @param  timeout_ms 超时, 0xFFFFFFFF 表示永久等待
  * @retval 1 = 取到, 0 = 超时
  */
uint8_t Cmd_Fetch(CmdMsg_t *msg, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_H */
