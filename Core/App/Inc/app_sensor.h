/**
  ******************************************************************************
  * @file    app_sensor.h
  * @brief   传感器采集任务(500ms 周期)
  ******************************************************************************
  */

#ifndef __APP_SENSOR_H
#define __APP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化传感器 */
void AppSensor_Init(void);

/** 任务主循环体(Task_Sensor) */
void AppSensor_Task(void *argument);

/**
  * @brief  复位 PID 内部状态
  * @note   进入自动模式 / 风扇关闭时由 Task_MotorFan 调用,
  *         避免把上一次运行的历史误差带到下一次控制中。
  */
void AppSensor_PidReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SENSOR_H */
