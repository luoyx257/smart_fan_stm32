/**
  ******************************************************************************
  * @file    app_fan.c
  * @brief   风扇主控任务实现
  * @note    设计要点:
  *          1) 主循环用 osMessageQueueGet 阻塞等待(最长 20ms), 因此倒计时/摇头
  *             状态机天然获得 20ms 的调度节拍, 不需要额外定时器。
  *          2) 自动模式判据: (红外有人 OR 压力坐下) AND 温度 > 阈值
  *             关闭判据   : 人离开(红外无 AND 压力无) 持续 30s
  *                          或 温度 <= 阈值 - 回差(2℃)
  *          3) 档位 1~5 对应占空比 40/55/70/85/100%。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_fan.h"
#include "app_state.h"
#include "app_sensor.h"     /* 为 AppSensor_PidReset() */
#include "cmd.h"
#include "cmsis_os.h"
#include "bsp_fan.h"
#include "bsp_servo.h"
#include "bsp_buzzer.h"
#include "app_ui.h"

/* Private define ------------------------------------------------------------*/
#define FAN_TICK_MS             20U     /* 主循环节拍 */
#define FAN_QUEUE_TIMEOUT_MS    20U     /* 等待命令的超时, 决定控制节拍 */

/* Private variables ---------------------------------------------------------*/
static uint32_t s_last_cmd_tick = 0U;   /* 上一次接受命令的时刻, 用于全局去抖 */

/* Private function prototypes -----------------------------------------------*/
static void     Fan_HandleCommand(const CmdMsg_t *msg);
static void     Fan_ApplyOutputs(void);
static void     Fan_AutoLogic(uint32_t tick_ms);
static void     Fan_CountdownLogic(uint32_t tick_ms);
static void     Fan_SwingLogic(uint32_t tick_ms);
static void     Fan_AlarmLogic(uint32_t tick_ms, uint8_t need_alarm);
static void     Fan_SetLevel(uint8_t level);
static void     Fan_Start(void);
static void     Fan_Shutdown(uint8_t stop_swing);
static uint8_t  Fan_DebounceOk(uint32_t tick_ms);

/* Private functions ---------------------------------------------------------*/

/** @brief 命令去抖: 距离上一条命令不足 CMD_DEBOUNCE_MS 的重复命令丢弃 */
static uint8_t Fan_DebounceOk(uint32_t tick_ms)
{
    if ((uint32_t)(tick_ms - s_last_cmd_tick) < CMD_DEBOUNCE_MS)
    {
        return 0U;
    }
    s_last_cmd_tick = tick_ms;
    return 1U;
}

/**
  * @brief  设置档位(0~5), 同时更新占空比与风扇开关
  * @note   0 档 = 停; 1~5 档 = 40/55/70/85/100%
  */
static void Fan_SetLevel(uint8_t level)
{
    if (level > FAN_LEVEL_MAX)
    {
        level = FAN_LEVEL_MAX;
    }

    g_state.act.level = level;
    g_state.act.duty  = AppState_LevelToDuty(level);
    g_state.act.fan_on = (level > 0U) ? 1U : 0U;
}

/** @brief 手动开风扇: 若当前 0 档则提升到 1 档 */
static void Fan_Start(void)
{
    if (g_state.act.level == 0U)
    {
        Fan_SetLevel(1U);
    }
    else
    {
        g_state.act.fan_on = 1U;
    }
}

/**
  * @brief  关闭风扇
  * @param  stop_swing 1 = 同时停止摇头并把舵机回中位
  */
static void Fan_Shutdown(uint8_t stop_swing)
{
    Fan_SetLevel(0U);

    /* 关风扇就取消倒计时, 避免"风扇已停但倒计时还在跑"的怪状态 */
    g_state.timer.active    = 0U;
    g_state.timer.remain_ms = 0U;
    g_state.timer.set_ms    = 0U;

    if (stop_swing != 0U)
    {
        g_state.cfg.swing       = SWING_OFF;
        g_state.act.servo_phase = SERVO_IDLE;
        Servo_Center();
    }
}

/** @brief 把 g_state 中的目标状态写进硬件 */
static void Fan_ApplyOutputs(void)
{
    if (g_state.act.fan_on != 0U)
    {
        Fan_SetDuty(g_state.act.duty);
    }
    else
    {
        Fan_Stop();
    }
}

/**
  * @brief  处理一条命令
  */
static void Fan_HandleCommand(const CmdMsg_t *msg)
{
    uint32_t now = (uint32_t)osKernelGetTickCount();

    switch (msg->cmd)
    {
    /* ---------------- 风扇开关 ---------------- */
    case CMD_FAN_ON:
        if (Fan_DebounceOk(now) != 0U)
        {
            Fan_Start();
        }
        break;

    case CMD_FAN_OFF:
        if (Fan_DebounceOk(now) != 0U)
        {
            Fan_Shutdown(1U);       /* 关风扇同时停止摇头(用户确认: 都关) */
        }
        break;

    case CMD_FAN_TOGGLE:
        if (Fan_DebounceOk(now) != 0U)
        {
            if (g_state.act.fan_on != 0U)
            {
                Fan_Shutdown(1U);
            }
            else
            {
                Fan_Start();
            }
        }
        break;

    /* ---------------- 档位 ---------------- */
    case CMD_LEVEL_UP:
        if (Fan_DebounceOk(now) != 0U)
        {
            uint8_t next = (uint8_t)(g_state.act.level + 1U);
            if (next > FAN_LEVEL_MAX)
            {
                next = FAN_LEVEL_MAX;   /* 到顶保持, 不循环 */
            }
            Fan_SetLevel(next);
        }
        break;

    case CMD_LEVEL_DOWN:
        if (Fan_DebounceOk(now) != 0U)
        {
            if (g_state.act.level > 0U)
            {
                Fan_SetLevel((uint8_t)(g_state.act.level - 1U));
            }
            else
            {
                Fan_SetLevel(0U);
            }
        }
        break;

    case CMD_LEVEL_SET:
        if (Fan_DebounceOk(now) != 0U)
        {
            Fan_SetLevel((uint8_t)msg->param);
            /* 自动模式下人工干预档位: 让 PID 输出对齐, 免得下一次 PID 运算
               又把它跳回去 */
            if (g_state.cfg.mode == MODE_AUTO)
            {
                AppSensor_PidReset();
            }
        }
        break;

    /* ---------------- PID 恒温调速结果 ----------------
       来自 Task_Sensor 的 CMD_PID_LEVEL: 不做去抖(去抖是为了防止人手连击,
       而 PID 的调用周期本身就有 10s, 天然没有抖动问题), 也不在手动模式生效。 */
    case CMD_PID_LEVEL:
        if (g_state.cfg.mode == MODE_AUTO)
        {
            Fan_SetLevel((uint8_t)msg->param);
        }
        break;

    /* ---------------- 模式 ---------------- */
    case CMD_MODE_SET:
        g_state.cfg.mode = (msg->param == (uint16_t)MODE_AUTO) ? MODE_AUTO : MODE_MANUAL;
        g_state.autom.nobody_ms = 0U;
        g_state.autom.alarm_ms  = 0U;
        g_state.autom.alarm_on  = 0U;
        g_state.autom.pid_run   = 0U;   /* 切模式时退出 PID 调速 */
        /* 切模式时先停风扇, 避免残留状态让人困惑 */
        Fan_Shutdown(0U);
        AppSensor_PidReset();
        break;

    case CMD_MODE_TOGGLE:
        g_state.cfg.mode = (g_state.cfg.mode == MODE_AUTO) ? MODE_MANUAL : MODE_AUTO;
        g_state.autom.nobody_ms = 0U;
        g_state.autom.alarm_ms  = 0U;
        g_state.autom.alarm_on  = 0U;
        g_state.autom.pid_run   = 0U;   /* 切模式时退出 PID 调速 */
        Fan_Shutdown(0U);
        AppSensor_PidReset();
        break;

    case CMD_TEMP_SET:
    {
        uint16_t th = msg->param;
        if (th < TEMP_THRESHOLD_MIN)
        {
            th = TEMP_THRESHOLD_MIN;
        }
        if (th > TEMP_THRESHOLD_MAX)
        {
            th = TEMP_THRESHOLD_MAX;
        }
        g_state.cfg.temp_threshold = (uint8_t)th;
        break;
    }

    /* ---------------- 倒计时 ---------------- */
    case CMD_COUNTDOWN_SET:
        if (msg->param == 0U)
        {
            g_state.timer.active    = 0U;
            g_state.timer.remain_ms = 0U;
            g_state.timer.set_ms    = 0U;
        }
        else
        {
            uint32_t ms = (uint32_t)msg->param * 1000UL;    /* 秒 -> 毫秒 */
            g_state.timer.set_ms    = ms;
            g_state.timer.remain_ms = ms;
            g_state.timer.active    = 1U;
            /* 设置倒计时视为希望风扇运行 */
            Fan_Start();
        }
        break;

    case CMD_COUNTDOWN_CANCEL:
        g_state.timer.active    = 0U;
        g_state.timer.remain_ms = 0U;
        g_state.timer.set_ms    = 0U;
        break;

    /* ---------------- 摇头 ---------------- */
    case CMD_SWING_SET:
        g_state.cfg.swing = (msg->param != 0U) ? SWING_ON : SWING_OFF;
        if (g_state.cfg.swing == SWING_OFF)
        {
            g_state.act.servo_phase = SERVO_IDLE;
            Servo_Center();
        }
        else
        {
            g_state.act.servo_phase = SERVO_GO_LEFT;    /* 从左边开始 */
        }
        break;

    case CMD_SWING_TOGGLE:
        if (g_state.cfg.swing == SWING_ON)
        {
            g_state.cfg.swing = SWING_OFF;
            g_state.act.servo_phase = SERVO_IDLE;
            Servo_Center();
        }
        else
        {
            g_state.cfg.swing = SWING_ON;
            g_state.act.servo_phase = SERVO_GO_LEFT;
        }
        break;

    /* ---------------- 状态查询 ---------------- */
    case CMD_QUERY_STATUS:
        AppUi_RequestReport();
        break;

    default:
        break;
    }
}

/**
  * @brief  自动模式逻辑
  * @note   开: (红外有人 OR 压力坐下) AND 温度 > 阈值
  *         关: (红外无人 AND 压力无) 持续 30s, 或 温度 <= 阈值 - 2℃
  */
static void Fan_AutoLogic(uint32_t tick_ms)
{
    uint8_t  human  = g_state.sensor.human;
    uint8_t  seated = g_state.sensor.seated;
    uint8_t  temp   = g_state.sensor.temp;
    uint8_t  dht_ok = g_state.sensor.dht_ok;
    uint8_t  present;

    /* DHT11 没读到时不要用 0℃ 去关风扇, 保持上一次的判定 */
    if (dht_ok == 0U)
    {
        temp = g_state.cfg.temp_threshold;   /* 视为"刚好到阈值", 不因传感器故障关风扇 */
    }

    present = ((human != 0U) || (seated != 0U)) ? 1U : 0U;

    if (present != 0U)
    {
        g_state.autom.nobody_ms = 0U;
    }
    else
    {
        /* 无人累计, 注意防溢出 */
        if (g_state.autom.nobody_ms < AUTO_NOBODY_TIMEOUT_MS)
        {
            g_state.autom.nobody_ms += FAN_TICK_MS;
        }
    }

    if (g_state.act.fan_on == 0U)
    {
        /* ---- 关闭状态: 判断是否该开 ---- */
        if ((present != 0U) && (temp > g_state.cfg.temp_threshold))
        {
            Fan_SetLevel(1U);       /* 自动开启从 1 档起步, 之后交给 PID 调速 */
            g_state.autom.alarm_ms = 0U;
            g_state.autom.alarm_on = 1U;
            /* 允许 PID 接管: 复位历史误差并让输出对齐当前档位, 避免接管瞬间突跳 */
            g_state.autom.pid_run = 1U;
            AppSensor_PidReset();
        }
    }
    else
    {
        /* ---- 运行状态: 判断是否该关 ---- */
        uint8_t should_stop = 0U;

        if (g_state.autom.nobody_ms >= AUTO_NOBODY_TIMEOUT_MS)
        {
            should_stop = 1U;       /* 人走了 30s */
        }
        if (temp + TEMP_HYSTERESIS <= g_state.cfg.temp_threshold)
        {
            should_stop = 1U;       /* 温度降下来了(带 2℃ 回差) */
        }

        if (should_stop != 0U)
        {
            g_state.autom.alarm_on = 0U;
            g_state.autom.alarm_ms = 0U;
            g_state.autom.pid_run  = 0U;    /* 停止 PID 调速 */
            Fan_Shutdown(1U);
        }
    }
}

/** @brief 倒计时逻辑 */
static void Fan_CountdownLogic(uint32_t tick_ms)
{
    if (g_state.timer.active == 0U)
    {
        return;
    }

    if (g_state.timer.remain_ms > FAN_TICK_MS)
    {
        g_state.timer.remain_ms -= FAN_TICK_MS;
    }
    else
    {
        /* 时间到: 关风扇 + 关摇头(用户确认: 都关) */
        g_state.timer.remain_ms = 0U;
        g_state.timer.active    = 0U;
        Fan_Shutdown(1U);
        g_state.autom.alarm_on = 0U;
        g_state.autom.alarm_ms = 0U;
    }
}

/**
  * @brief  舵机摇头状态机(非阻塞)
  * @note   循环: 左(45°)停留1.2s -> 中位0.3s -> 右(225°)停留1.2s -> 中位0.3s -> 左...
  *         用 s_swing_side 记录"上一次停在左还是右", 因为回中之后
  *         g_state.act.servo_angle 已经被改成 135°, 无法再判断来源侧。
  */
static void Fan_SwingLogic(uint32_t tick_ms)
{
    static uint32_t dwell_ms      = 0U;
    static uint8_t  s_swing_side  = 0U;     /* 0 = 上次停在左, 1 = 上次停在右 */

    if (g_state.cfg.swing == SWING_OFF)
    {
        dwell_ms     = 0U;
        s_swing_side = 0U;
        return;
    }

    switch (g_state.act.servo_phase)
    {
    case SERVO_IDLE:
        /* 摇头刚被打开, 从左边开始 */
        Servo_SetAngle(SERVO_ANGLE_MIN);
        g_state.act.servo_angle = SERVO_ANGLE_MIN;
        s_swing_side = 0U;
        dwell_ms = 0U;
        g_state.act.servo_phase = SERVO_DWELL_LEFT;
        break;

    case SERVO_DWELL_LEFT:
        dwell_ms += FAN_TICK_MS;
        if (dwell_ms >= SERVO_MOVE_DWELL_MS)
        {
            dwell_ms = 0U;
            g_state.act.servo_phase = SERVO_GO_CENTER;
        }
        break;

    case SERVO_GO_CENTER:
        Servo_Center();
        g_state.act.servo_angle = 135U;
        dwell_ms = 0U;
        g_state.act.servo_phase = SERVO_DWELL_CENTER;
        break;

    case SERVO_DWELL_CENTER:
        dwell_ms += FAN_TICK_MS;
        if (dwell_ms >= SERVO_CENTER_DWELL_MS)
        {
            dwell_ms = 0U;
            /* 上次停在左 -> 这回往右; 上次停在右 -> 这回往左 */
            if (s_swing_side == 0U)
            {
                g_state.act.servo_phase = SERVO_GO_RIGHT;
            }
            else
            {
                g_state.act.servo_phase = SERVO_GO_LEFT;
            }
        }
        break;

    case SERVO_GO_RIGHT:
        Servo_SetAngle(SERVO_ANGLE_MAX);
        g_state.act.servo_angle = SERVO_ANGLE_MAX;
        s_swing_side = 1U;
        dwell_ms = 0U;
        g_state.act.servo_phase = SERVO_DWELL_RIGHT;
        break;

    case SERVO_DWELL_RIGHT:
        dwell_ms += FAN_TICK_MS;
        if (dwell_ms >= SERVO_MOVE_DWELL_MS)
        {
            dwell_ms = 0U;
            g_state.act.servo_phase = SERVO_GO_CENTER;
        }
        break;

    case SERVO_GO_LEFT:
        Servo_SetAngle(SERVO_ANGLE_MIN);
        g_state.act.servo_angle = SERVO_ANGLE_MIN;
        s_swing_side = 0U;
        dwell_ms = 0U;
        g_state.act.servo_phase = SERVO_DWELL_LEFT;
        break;

    default:
        g_state.act.servo_phase = SERVO_IDLE;
        break;
    }

    (void)tick_ms;
}

/**
  * @brief  蜂鸣器报警逻辑(A 方案)
  *         滴 200ms 停 1.8s 循环, 报警持续超过 30s 后静音
  */
static void Fan_AlarmLogic(uint32_t tick_ms, uint8_t need_alarm)
{
    static uint32_t phase_ms = 0U;
    static uint8_t  beeping  = 0U;

    if (need_alarm == 0U)
    {
        phase_ms = 0U;
        beeping  = 0U;
        if (g_state.act.buzzer_on != 0U)
        {
            Buzzer_Off();
            g_state.act.buzzer_on = 0U;
        }
        return;
    }

    /* 报警时长累计 */
    if (g_state.autom.alarm_ms < 0xFFFFFFFFUL)
    {
        g_state.autom.alarm_ms += FAN_TICK_MS;
    }

    /* 超过静音时限就不再响 */
    if (g_state.autom.alarm_ms >= BUZZ_ALARM_SILENCE_MS)
    {
        Buzzer_Off();
        g_state.act.buzzer_on = 0U;
        return;
    }

    phase_ms += FAN_TICK_MS;

    if (beeping == 0U)
    {
        if (phase_ms >= BUZZ_BEEP_OFF_MS)
        {
            phase_ms = 0U;
            beeping  = 1U;
            Buzzer_On();
            g_state.act.buzzer_on = 1U;
        }
    }
    else
    {
        if (phase_ms >= BUZZ_BEEP_ON_MS)
        {
            phase_ms = 0U;
            beeping  = 0U;
            Buzzer_Off();
            g_state.act.buzzer_on = 0U;
        }
    }

    (void)tick_ms;
}


/* Exported functions --------------------------------------------------------*/

/** @brief 初始化执行器 */
void AppFan_Init(void)
{
    AppState_Init();

    Buzzer_Init();
    Fan_Init();
    Servo_Init();

    s_last_cmd_tick = 0U;

    /* 上电清空可能残留的命令队列 */
    if (xQueueCmdHandle != NULL)
    {
        (void)osMessageQueueReset(xQueueCmdHandle);
    }
}

/** @brief 任务主循环 */
void AppFan_Task(void *argument)
{
    CmdMsg_t msg;
    uint32_t tick_ms;

    (void)argument;

    for (;;)
    {
        tick_ms = (uint32_t)osKernelGetTickCount();

        /* ---- 1) 处理命令(最多等 20ms, 决定控制节拍) ---- */
        if (Cmd_Fetch(&msg, FAN_QUEUE_TIMEOUT_MS) != 0U)
        {
            Fan_HandleCommand(&msg);

            /* 队列里可能还有积压命令, 一次性处理完 */
            while (Cmd_Fetch(&msg, 0U) != 0U)
            {
                Fan_HandleCommand(&msg);
            }
        }

        /* ---- 2) 按当前模式执行逻辑 ---- */
        if (g_state.cfg.mode == MODE_AUTO)
        {
            Fan_AutoLogic(tick_ms);
        }

        /* ---- 3) 倒计时 ---- */
        Fan_CountdownLogic(tick_ms);

        /* ---- 4) 摇头状态机 ---- */
        Fan_SwingLogic(tick_ms);

        /* ---- 5) 报警 ----
           A 方案: 自动模式下风扇因"有人+温度超阈值"而运行时报警 */
        Fan_AlarmLogic(tick_ms,
                       (uint8_t)((g_state.cfg.mode == MODE_AUTO) &&
                                 (g_state.act.fan_on != 0U)));

        /* ---- 6) 输出到硬件 ---- */
        Fan_ApplyOutputs();
    }
}
