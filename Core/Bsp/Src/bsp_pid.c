/**
  ******************************************************************************
  * @file    bsp_pid.c
  * @brief   增量式 PID 实现
  * @note    公式(增量式 / 速度式 PID):
  *
  *            Δu = Kp*(e[k] - e[k-1]) + Ki*e[k] + Kd*(e[k] - 2e[k-1] + e[k-2])
  *            u[k] = u[k-1] + Δu
  *
  *          其中 e[k] = 当前温度 - 目标温度 (误差为正表示"还太热", 需要加档)
  *
  *          定点实现:
  *            e 以 0.1℃ 为单位      -> err10 = (temp*10 - target*10)
  *            u 以 1/1000 档为单位   -> out_milli
  *            Δu 计算里先做 err10 与增益的乘积, 再统一除以 (PID_GAIN_SCALE*PID_ERR_SCALE)
  *
  *          限幅策略:
  *            输出被夹到 [out_floor, out_limit]；当误差方向仍然要求继续增大输出
  *            但输出已经顶到上限时, 通过"不更新历史误差"?  —— 不需要:
  *            增量式 PID 的积分作用体现在 u[k-1] 上, 而 u 已经被限幅, 因此
  *            结构上就不存在积分饱和(抗饱和天然成立), 这也是选增量式的原因之一。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_pid.h"
#include <stddef.h>     /* NULL */

/* Private define ------------------------------------------------------------*/
/* Δu 的统一除数: 增益放大倍数 x 误差放大倍数 */
#define PID_DIVISOR         (PID_GAIN_SCALE * PID_ERR_SCALE)

/* Private function prototypes -----------------------------------------------*/
static int32_t PID_Clamp(int32_t v, int32_t lo, int32_t hi);

/* Private functions ---------------------------------------------------------*/

/** @brief 限幅 */
static int32_t PID_Clamp(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化 */
void PID_Init(Pid_t *pid, int32_t kp, int32_t ki, int32_t kd,
              int32_t out_min, int32_t out_max)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp * PID_GAIN_SCALE;
    pid->ki = ki * PID_GAIN_SCALE;
    pid->kd = kd * PID_GAIN_SCALE;

    pid->out_limit_milli = out_max * PID_OUT_MILLI;
    pid->out_floor_milli = out_min * PID_OUT_MILLI;
    if (pid->out_floor_milli > pid->out_limit_milli)
    {
        pid->out_floor_milli = pid->out_limit_milli;
    }

    pid->target_c  = 0;
    pid->cur_c     = 0;
    pid->out_milli = pid->out_floor_milli;
    pid->e1        = 0;
    pid->e2        = 0;
    pid->deadband10 = 0;
    pid->enabled   = 0U;
    pid->inited    = 0U;
}

/** @brief 设置死区半宽(单位 0.1℃) */
void PID_SetDeadband(Pid_t *pid, int32_t deadband10)
{
    if (pid != NULL)
    {
        pid->deadband10 = (deadband10 < 0) ? 0 : deadband10;
    }
}

/** @brief 让输出跟随当前档位(避免 PID 接管瞬间输出突跳) */
void PID_SetOutputLevel(Pid_t *pid, int32_t level)
{
    if (pid == NULL)
    {
        return;
    }
    pid->out_milli = PID_Clamp(level * PID_OUT_MILLI,
                               pid->out_floor_milli,
                               pid->out_limit_milli);
}

/** @brief 清零历史 */
void PID_Reset(Pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->e1     = 0;
    pid->e2     = 0;
    pid->inited = 0U;
}

/** @brief 取当前输出档位 */
int32_t PID_GetLevel(const Pid_t *pid)
{
    if (pid == NULL)
    {
        return 0;
    }
    /* 四舍五入到整数档 */
    return (pid->out_milli + (PID_OUT_MILLI / 2)) / PID_OUT_MILLI;
}

/**
  * @brief  执行一个控制周期
  * @note   死区处理: |误差| <= deadband 时认为"已经在目标附近", 本次不调节。
  *         这对温度控制很关键 —— DHT11 分辨率只有 1℃, 若目标正好落在两个
  *         整数温度之间, 误差会在 ±1 之间来回跳, 没有死区就会反复加减档。
  */
int32_t PID_Compute(Pid_t *pid, int32_t temp_c, int32_t target_c)
{
    int32_t e0;         /* 本次误差(0.1℃) */
    int32_t du;         /* 输出增量(1/1000 档) */
    int32_t p_term;
    int32_t i_term;
    int32_t d_term;

    if ((pid == NULL) || (pid->enabled == 0U))
    {
        return PID_GetLevel(pid);
    }

    pid->cur_c    = temp_c;
    pid->target_c = target_c;

    /* ---- 误差: 正误差 = 太热, 需要加档 ---- */
    e0 = (temp_c - target_c) * PID_ERR_SCALE;

    /* ---- 死区: 在目标附近不调节, 防止稳态抖动 ---- */
    if ((e0 <= pid->deadband10) && (e0 >= -pid->deadband10))
    {
        return PID_GetLevel(pid);
    }

    if (pid->inited == 0U)
    {
        /* 首次运算: 把历史误差置成当前误差, 避免微分项产生一个巨大的冲击 */
        pid->e1     = e0;
        pid->e2     = e0;
        pid->inited = 1U;
    }

    /* ---- 增量式 PID 三项 ---- */
    /* P: Kp*(e0 - e1) */
    p_term = pid->kp * (e0 - pid->e1);

    /* I: Ki*e0 */
    i_term = pid->ki * e0;

    /* D: Kd*(e0 - 2*e1 + e2) */
    d_term = pid->kd * ((e0 - (2 * pid->e1)) + pid->e2);

    du = (p_term + i_term + d_term) / PID_DIVISOR;

    /* ---- 输出累加并限幅 ---- */
    pid->out_milli += du;
    pid->out_milli = PID_Clamp(pid->out_milli,
                               pid->out_floor_milli,
                               pid->out_limit_milli);

    /* ---- 保存误差历史 ---- */
    pid->e2 = pid->e1;
    pid->e1 = e0;

    return PID_GetLevel(pid);
}
