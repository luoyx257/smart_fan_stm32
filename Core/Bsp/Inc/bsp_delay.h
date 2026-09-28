/**
  ******************************************************************************
  * @file    bsp_delay.h
  * @brief   微秒级延时 —— 基于 Cortex-M3 DWT 周期计数器
  * @note    DHT11 单总线时序需要精确到 1us, 而 HAL_Delay 只有 1ms 精度,
  *          且 TIM4 已被用作 HAL 时基、SysTick 已交给 FreeRTOS,
  *          因此改用 DWT->CYCCNT 周期计数器(不占定时器、不产生中断)。
  *          SystemCoreClock 不高于 100MHz 时 CYCCNT 不会在两次读取间回绕,
  *          本工程为 72MHz, 单次 32 位回绕约 59.6 秒, 完全够用。
  ******************************************************************************
  */

#ifndef __BSP_DELAY_H
#define __BSP_DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化 DWT 周期计数器, 必须在 SystemClock_Config() 之后调用 */
void BSP_DelayInit(void);

/** 忙等 us 微秒(不自旋打断 RTOS, 仅用于极短的硬件时序) */
void BSP_DelayUs(uint32_t us);

/** 读取当前微秒时间戳(相对 DWT 启动时刻) */
uint32_t BSP_Micros(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DELAY_H */
