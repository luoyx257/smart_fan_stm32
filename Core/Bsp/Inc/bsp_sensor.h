/**
  ******************************************************************************
  * @file    bsp_sensor.h
  * @brief   人体红外(HC-SR501, PA0) 与 压力传感器(DO=PA5 / AO=PA4) 驱动
  * @note    PA0/PA5 在 CubeMX 中生成的是 NOPULL 输入, 这里改成上拉输入,
  *          避免模块未插时引脚悬空乱跳。DHT11 见 bsp_dht11.h。
  ******************************************************************************
  */

#ifndef __BSP_SENSOR_H
#define __BSP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化红外与压力通道的 GPIO(含把 PA0/PA5 改成上拉输入) */
void Sensor_Init(void);

/** 人体红外: 1 = 检测到人, 0 = 无人。HC-SR501 高电平有效 */
uint8_t Sensor_InfraredDetected(void);

/** 压力 DO: 1 = 判定坐下(模块越过电位器阈值), 0 = 未坐 */
uint8_t Sensor_PressureDoActive(void);

/** 读取压力 AO 原始 ADC 值(0~4095), 失败返回 0 */
uint16_t Sensor_PressureRawAdc(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SENSOR_H */
