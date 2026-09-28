/**
  ******************************************************************************
  * @file    bsp_pid.h
  * @brief   增量式 PID 控制器(整数运算, 定点放大, 不用浮点)
  * @note    为什么用"增量式"而不是"位置式":
  *            1) 温控是大惯性对象, 增量式输出的是"增量", 天然不会因为积分项
  *               瞬间累积而把执行器打到极限, 更适合"档位"这种离散执行器;
  *            2) 不需要保存历史积分项, 只留最近几次误差, RAM 开销小;
  *            3) 不用做积分抗饱和的复杂处理, 因为它结构上就不存在积分饱和。
  *
  *          为什么不用 float:
  *            Cortex-M3 没有硬件 FPU, 软件浮点会引入浮点库(几 KB Flash)
  *            并且每次运算几十个周期。温控这种慢过程用定点足够, 于是:
  *              误差单位 = 0.1℃   (err10)
  *              输出单位 = 1/1000 档 (out_milli)
  *              增益单位 = 1/100
  ******************************************************************************
  */

#ifndef __BSP_PID_H
#define __BSP_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*------------------------------------------------------------------------------
 * 定点单位约定
 *----------------------------------------------------------------------------*/
#define PID_ERR_SCALE       10      /* 误差放大 10 倍: err10 = (temp - target) * 10, 单位 0.1℃ */
#define PID_OUT_MILLI       1000    /* 输出放大 1000 倍: 1 档 = 1000 */
#define PID_GAIN_SCALE      100     /* 增益放大 100 倍 */

/*------------------------------------------------------------------------------
 * 控制器状态
 *----------------------------------------------------------------------------*/
typedef struct {
    /* --- 整定参数(已按 PID_GAIN_SCALE 放大) --- */
    int32_t kp;             /* 比例增益 */
    int32_t ki;             /* 积分增益 */
    int32_t kd;             /* 微分增益 */

    /* --- 输入 / 输出 --- */
    int32_t target_c;       /* 目标温度 ℃ (整数) */
    int32_t cur_c;          /* 当前温度 ℃ (整数) */
    int32_t out_milli;      /* 输出累计值, 单位 1/1000 档 */
    int32_t out_limit_milli;/* 输出上限(1/1000 档), 对应最高档 */
    int32_t out_floor_milli;/* 输出下限(1/1000 档), 对应最低档 */

    /* --- 历史误差(增量式 PID 只需要最近 3 次) --- */
    int32_t e1;             /* 上次误差 */
    int32_t e2;             /* 上上次误差 */

    /* --- 死区: 目标附近不调节, 避免稳态来回抖动 --- */
    int32_t deadband10;     /* 死区半宽, 单位 0.1℃ */

    /* --- 运行状态 --- */
    uint8_t enabled;        /* 1 = 使能 */
    uint8_t inited;         /* 1 = 已初始化历史误差 */
} Pid_t;

/*------------------------------------------------------------------------------
 * 接口
 *----------------------------------------------------------------------------*/

/**
  * @brief  初始化控制器
  * @param  pid        控制器实例
  * @param  kp / ki / kd  增益(未放大, 内部会乘 PID_GAIN_SCALE)
  * @param  out_min / out_max  输出限幅, 单位为"档"(整数, 例如 1 和 5)
  */
void PID_Init(Pid_t *pid, int32_t kp, int32_t ki, int32_t kd,
              int32_t out_min, int32_t out_max);

/**
  * @brief  设置死区半宽
  * @param  deadband10 死区半宽, 单位 0.1℃。例如传 5 表示 ±0.5℃ 内不调节。
  * @note   必须设置, 否则 DHT11 的 1℃ 分辨率会让误差在整数温度间反复跳变,
  *         PID 会把档位来回拉。
  */
void PID_SetDeadband(Pid_t *pid, int32_t deadband10);

/**
  * @brief  跟随外部直接设定的输出(手动调档时调用, 避免 PID 接管时输出突跳)
  * @param  level 当前实际档位, 单位"档"
  */
void PID_SetOutputLevel(Pid_t *pid, int32_t level);

/**
  * @brief  清零历史误差(重新进入 PID 控制时调用)
  */
void PID_Reset(Pid_t *pid);

/**
  * @brief  执行一个控制周期
  * @param  pid       控制器实例
  * @param  temp_c    当前温度 ℃
  * @param  target_c  目标温度 ℃
  * @retval 本次运算后应输出的档位(已按 out_limit 限幅, 并经死区处理)
  * @note   典型调用周期应远小于对象的响应时间常数。本工程温控对象时间常数在
  *         分钟级, 因此调用周期取 10s(见 app_state.h 的 PID_PERIOD_MS)。
  */
int32_t PID_Compute(Pid_t *pid, int32_t temp_c, int32_t target_c);

/**
  * @brief  取当前输出档位(不运算)
  */
int32_t PID_GetLevel(const Pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PID_H */
