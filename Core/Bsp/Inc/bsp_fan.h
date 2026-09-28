/**
  ******************************************************************************
  * @file    bsp_fan.h
  * @brief   TB6612 电机驱动 —— 风扇调速
  * @note    接线:
  *            PWMA  = PB5  (TIM3_CH2, 部分重映射, 20kHz)
  *            AIN1  = PB0
  *            AIN2  = PB1
  *            STBY  硬件直接接高电平, 软件不管
  *          TB6612 真值表(AIN1/AIN2):
  *            0 0 -> 停止(滑行)
  *            1 0 -> 正转
  *            0 1 -> 反转
  *            1 1 -> 刹车(短接制动)
  ******************************************************************************
  */

#ifndef __BSP_FAN_H
#define __BSP_FAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化: 拉低使能、停止电机、启动 PWM 输出 */
void Fan_Init(void);

/**
  * @brief  设置风扇转速
  * @param  duty_permille 占空比千分比 0~1000 (0 = 停)
  */
void Fan_SetDuty(uint16_t duty_permille);

/** 立即停止(滑行) */
void Fan_Stop(void);

/** 刹车(电机两端短接, 停得更快, 但会有制动电流) */
void Fan_Brake(void);

/** 反转(风道装反时可用; 默认不使用) */
void Fan_SetReverse(uint8_t reverse);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FAN_H */
