/**
  ******************************************************************************
  * @file    bsp_delay.c
  * @brief   DWT 微秒延时实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_delay.h"
#include "stm32f1xx.h"

/* ---------------------------------------------------------------------------
 * DWT->CYCCNT 使能流程(Cortex-M3 参考手册):
 *   1. DEMCR.TRCENA = 1          打开跟踪与调试模块总开关
 *   2. DWT->CYCCNT = 0           计数器清零
 *   3. DWT->CTRL |= CYCCNTENA    使能周期计数器
 * ------------------------------------------------------------------------- */

/** @brief 初始化 DWT 周期计数器 */
void BSP_DelayInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 使能跟踪单元 */
    DWT->CYCCNT = 0U;                                 /* 周期计数清零 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              /* 启动周期计数 */
}

/** @brief 读取微秒时间戳, 72MHz 下 1us = 72 个周期 */
uint32_t BSP_Micros(void)
{
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}

/** @brief 忙等 us 微秒 */
void BSP_DelayUs(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if (us == 0U)
    {
        return;
    }

    /* 本次需要等待的 CPU 周期数 */
    ticks = us * (SystemCoreClock / 1000000U);

    start = DWT->CYCCNT;
    /* 无符号减法自然处理回绕 */
    while ((uint32_t)(DWT->CYCCNT - start) < ticks)
    {
        /* busy wait */
    }
}
