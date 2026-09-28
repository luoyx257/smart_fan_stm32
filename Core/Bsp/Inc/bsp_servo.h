/**
  ******************************************************************************
  * @file    bsp_servo.h
  * @brief   舵机 PWM 驱动(PA8 / TIM1_CH1)
  * @note    舵机型号: 维特 P20 系列 PWM 数字舵机(工作 5~8.4V, 堵转 1~2.3A)
  *          PWM 参数: 周期 20ms(50Hz), 脉宽 500~2500us, 中位 1500us
  *                    500~2500us 对应 0~270°
  *          ！！供电注意！！ 舵机必须用独立 5~6V 电源, 且与单片机共地;
  *          蓝板 5V 引脚(USB 供电约 500mA)带不动这个舵机, 会导致单片机复位。
  ******************************************************************************
  */

#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化: 启动 TIM1_CH1 PWM 并回中位 */
void Servo_Init(void);

/**
  * @brief  设置舵机角度
  * @param  angle_deg 角度 0~270, 会自动限幅到 SERVO_ANGLE_MIN~SERVO_ANGLE_MAX
  */
void Servo_SetAngle(uint16_t angle_deg);

/** 回到机械中位(135°) */
void Servo_Center(void);

/** 直接把角度换算成脉宽 us, 便于调试观察 */
uint16_t Servo_AngleToPulseUs(uint16_t angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SERVO_H */
