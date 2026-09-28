/**
  ******************************************************************************
  * @file    app_ui.h
  * @brief   串口屏(USART2, TJC4827X243_011C) 显示任务
  * @note    职责:
  *            - 解析串口屏下发的控件消息 -> Cmd_Post() 投递到 xQueueCmd
  *            - 按需把 g_state 上报到串口屏
  *          并发约定: 只有本任务调用 bsp_screen 的发送函数。其它任务要刷新
  *          屏幕时调用 AppUi_RequestReport(), 由本任务统一发送。
  ******************************************************************************
  */

#ifndef __APP_UI_H
#define __APP_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化串口屏驱动 */
void AppUi_Init(void);

/** 任务主循环体(Task_Uart_UI) */
void AppUi_Task(void *argument);

/**
  * @brief  请求立即上报一次状态(线程安全, 只自增计数器)
  * @note   由 Task_MotorFan 在执行器状态变化后调用
  */
void AppUi_RequestReport(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UI_H */
