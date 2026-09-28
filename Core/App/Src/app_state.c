/**
  ******************************************************************************
  * @file    app_state.c
  * @brief   全局状态实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_state.h"

/* Private variables ---------------------------------------------------------*/
/* 全局唯一状态实例。放在 .bss, 不占任务栈, 不占 FreeRTOS 堆。 */
AppState_t g_state;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  档位转占空比(千分比)
  * @param  level 档位, 超范围自动限幅
  * @retval 占空比 0~1000 (千分比)
  */
uint16_t AppState_LevelToDuty(uint8_t level)
{
    /* 静态常量表: 0~5 档对应的占空比千分比 */
    static const uint16_t duty_table[FAN_LEVEL_MAX + 1U] = {
        FAN_DUTY_LEVEL0,
        FAN_DUTY_LEVEL1,
        FAN_DUTY_LEVEL2,
        FAN_DUTY_LEVEL3,
        FAN_DUTY_LEVEL4,
        FAN_DUTY_LEVEL5
    };

    if (level > FAN_LEVEL_MAX)
    {
        level = FAN_LEVEL_MAX;      /* 上限保护 */
    }
    return duty_table[level];
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  上电初始化全局状态为安全值
  * @note   必须在创建任务之前调用(或作为第一个任务的起始动作)
  */
void AppState_Init(void)
{
    /* --- 传感器 --- */
    g_state.sensor.temp      = 0U;
    g_state.sensor.humi      = 0U;
    g_state.sensor.dht_ok    = 0U;
    g_state.sensor.human     = 0U;
    g_state.sensor.seated    = 0U;
    g_state.sensor.pressure  = 0U;

    /* --- 用户配置: 上电默认手动模式、风扇停、不摇头, 避免一上电就吹 --- */
    g_state.cfg.mode            = MODE_MANUAL;
    g_state.cfg.temp_threshold  = TEMP_THRESHOLD_DEFAULT;
    g_state.cfg.swing           = SWING_OFF;

    /* --- 执行器 --- */
    g_state.act.fan_on          = 0U;
    g_state.act.level           = 0U;
    g_state.act.duty            = FAN_DUTY_LEVEL0;
    g_state.act.buzzer_on       = 0U;
    g_state.act.servo_phase     = SERVO_IDLE;
    g_state.act.servo_angle     = 135U;     /* 上电回中位 */

    /* --- 倒计时 --- */
    g_state.timer.remain_ms     = 0U;
    g_state.timer.set_ms        = 0U;
    g_state.timer.active        = 0U;

    /* --- 自动模式内部 --- */
    g_state.autom.nobody_ms     = 0U;
    g_state.autom.alarm_ms      = 0U;
    g_state.autom.alarm_on      = 0U;
    g_state.autom.pid_run       = 0U;

    /* --- 统计 --- */
    g_state.stat.cmd_rx         = 0U;
    g_state.stat.cmd_drop       = 0U;
    g_state.stat.dht_err        = 0U;
}
