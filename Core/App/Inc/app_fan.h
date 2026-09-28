/**
  ******************************************************************************
  * @file    app_fan.h
  * @brief   风扇主控任务 —— 唯一的执行器决策点
  * @note    Task_MotorFan(优先级 High) 独占:
  *            - 消费 xQueueCmd 命令
  *            - 手动/自动模式逻辑
  *            - 倒计时逻辑
  *            - 舵机摇头状态机
  *            - 蜂鸣器报警
  *          其它任务只投递命令、只读状态, 不允许直接操作执行器。
  ******************************************************************************
  */

#ifndef __APP_FAN_H
#define __APP_FAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化执行器(风扇/舵机/蜂鸣器)与全局状态 */
void AppFan_Init(void);

/** 任务主循环体(Task_MotorFan) */
void AppFan_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FAN_H */
