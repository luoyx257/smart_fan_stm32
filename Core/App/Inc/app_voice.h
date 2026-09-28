/**
  ******************************************************************************
  * @file    app_voice.h
  * @brief   SU-03T 语音模块解析任务(USART1, 9600)
  * @note    帧格式(自定义, 详见 USER_PROTOCOL.md):
  *            0xAA 0x55 CMD VALUE XOR
  *          CMD 为命令码, VALUE 为参数(温度阈值/档位/分钟数), XOR 为前 4 字节异或。
  ******************************************************************************
  */

#ifndef __APP_VOICE_H
#define __APP_VOICE_H

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 */
void AppVoice_Init(void);

/** 任务主循环体(Task_Voice) */
void AppVoice_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __APP_VOICE_H */
