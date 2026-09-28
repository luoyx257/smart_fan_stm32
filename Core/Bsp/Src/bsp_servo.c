/**
  ******************************************************************************
  * @file    bsp_servo.c
  * @brief   舵机 PWM 实现
  * @note    TIM1 高级定时器, 输出需要 MOE 置位, HAL_TIM_PWM_Start() 内部会做。
  *          TIM1: PSC=71 -> 1MHz 计数(1 tick = 1us), ARR=19999 -> 20ms 周期。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_servo.h"
#include "app_state.h"
#include "tim.h"

/* Private define ------------------------------------------------------------*/
#define SERVO_MAX_ANGLE     270U    /* 该系列舵机总行程 270° */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  角度 -> 脉宽微秒
  * @note   线性映射: 0° -> 500us, 270° -> 2500us
  */
uint16_t Servo_AngleToPulseUs(uint16_t angle_deg)
{
    uint32_t span_us;
    uint32_t pulse;

    if (angle_deg > SERVO_MAX_ANGLE)
    {
        angle_deg = SERVO_MAX_ANGLE;
    }

    /* 脉宽跨度 500 -> 2500 共 2000us */
    span_us = (uint32_t)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);

    pulse = (uint32_t)SERVO_PULSE_MIN_US
          + (span_us * (uint32_t)angle_deg) / SERVO_MAX_ANGLE;

    return (uint16_t)pulse;
}

/** @brief 初始化舵机通道 */
void Servo_Init(void)
{
    /* 先把比较值设为中位, 避免启动瞬间舵机猛冲 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SERVO_PULSE_MID_US);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    Servo_Center();
}

/**
  * @brief  设置角度
  * @note   做双重限幅:
  *           1) 物理限幅 0~270°
  *           2) 软件限幅 SERVO_ANGLE_MIN~SERVO_ANGLE_MAX (默认 45~225°),
  *              避免摇头时顶到机械限位导致舵机堵转发热
  */
void Servo_SetAngle(uint16_t angle_deg)
{
    uint16_t pulse;

    if (angle_deg < SERVO_ANGLE_MIN)
    {
        angle_deg = SERVO_ANGLE_MIN;
    }
    if (angle_deg > SERVO_ANGLE_MAX)
    {
        angle_deg = SERVO_ANGLE_MAX;
    }
    if (angle_deg > SERVO_MAX_ANGLE)
    {
        angle_deg = SERVO_MAX_ANGLE;
    }

    pulse = Servo_AngleToPulseUs(angle_deg);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pulse);
}

/** @brief 回中位 */
void Servo_Center(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SERVO_PULSE_MID_US);
}
