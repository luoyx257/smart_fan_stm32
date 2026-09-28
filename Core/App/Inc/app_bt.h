/**
  ******************************************************************************
  * @file    app_bt.h
  * @brief   HC-05 蓝牙解析任务(USART3, 9600)
  * @note    采用"文本行协议", 手机 APP(如"蓝牙调试助手")直接发 ASCII 命令,
  *          以 '\n' 或 '\r' 结尾, 大写敏感(代码内部会统一转大写再比较)。
  *          完整命令表见 USER_PROTOCOL.md。
  ******************************************************************************
  */

#ifndef __APP_BT_H
#define __APP_BT_H

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 */
void AppBt_Init(void);

/** 任务主循环体(Task_Bluetooth) */
void AppBt_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BT_H */
