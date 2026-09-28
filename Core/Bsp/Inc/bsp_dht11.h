/**
  ******************************************************************************
  * @file    bsp_dht11.h
  * @brief   DHT11 温湿度传感器驱动(单总线, PA1)
  * @note    ★ 采用"阻塞式完整时序"实现 ★
  *          原因: DHT11 的起始信号(18ms)与传感器应答(80us)必须在同一次连续
  *          调用中完成, 一旦被 FreeRTOS 的 tick 切开就必然读不到数据。
  *          因此 DHT11_Read() 内部一次做完 18ms 起始 + 5ms 读 40bit,
  *          全程约 23ms 阻塞; 带 2 秒间隔保护, 未到间隔立即返回。
  ******************************************************************************
  */

#ifndef __BSP_DHT11_H
#define __BSP_DHT11_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化 GPIO 为空闲状态(输入, 上拉) */
void DHT11_Init(void);

/**
  * @brief  执行一次完整读取(阻塞约 23ms, 带 2s 间隔保护)
  * @retval 1 = 本次确实读取且成功
  *         0 = 未到 2s 间隔(没读), 或读取/校验失败
  * @note   成功后数据通过 DHT11_Fetch() 取出
  */
uint8_t DHT11_Read(void);

/**
  * @brief  周期调用入口(内部等价于 DHT11_Read, 保留以兼容上层写法)
  */
void DHT11_Process(void);

/** 兼容接口: 等价于 DHT11_Read() */
uint8_t DHT11_Start(void);

/** 恒返回 1(阻塞式实现, 保留接口兼容性) */
uint8_t DHT11_IsIdle(void);

/** 最近一次读取是否成功 */
uint8_t DHT11_IsOk(void);

/** 读取完成次数(无论成败都 +1), 上层据此判断"刚结束一次读取" */
uint32_t DHT11_GetDoneCount(void);

/**
  * @brief  取回数据并清标志
  * @param  temp 温度 ℃
  * @param  humi 湿度 %
  * @retval 1 = 有新数据, 0 = 没有
  */
uint8_t DHT11_Fetch(uint8_t *temp, uint8_t *humi);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DHT11_H */
