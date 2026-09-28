/**
  ******************************************************************************
  * @file    bsp_buzzer.h
  * @brief   有源蜂鸣器驱动(MH-FMD 模块, PB8)
  * @note    模块内部自带振荡电路和三极管, 低电平导通发声。
  *          PB8 在 CubeMX 中初始电平已设为 SET(高), 上电不响。
  ******************************************************************************
  */

#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化(关闭蜂鸣器) */
void Buzzer_Init(void);

/** 开/关 */
void Buzzer_On(void);
void Buzzer_Off(void);

/** @param on 1 = 响, 0 = 停 */
void Buzzer_Set(uint8_t on);

/** 当前是否在响 */
uint8_t Buzzer_IsOn(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BUZZER_H */
