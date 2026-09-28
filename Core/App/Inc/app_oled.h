/**
  ******************************************************************************
  * @file    app_oled.h
  * @brief   OLED 显示任务
  ******************************************************************************
  */

#ifndef __APP_OLED_H
#define __APP_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 OLED(自动探地址 0x3C/0x3D) */
void AppOled_Init(void);

/** 任务主循环体(Task_OLED) */
void AppOled_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __APP_OLED_H */
