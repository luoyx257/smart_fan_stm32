/**
  ******************************************************************************
  * @file    app_sensor.c
  * @brief   传感器采集任务实现
  * @note    DHT11 由本任务的状态机驱动(每 20ms 推进一步), 不再用软件定时器,
  *          少一个内核对象, 也少一处优先级竞争。
  *          采集结果:
  *            - 更新 g_state.sensor.* (供 Task_MotorFan 决策)
  *            - 打包 SensorMsg_t 投递 xQueueSensorData (供 Task_OLED 显示)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_sensor.h"
#include "app_state.h"
#include "cmd.h"
#include "cmsis_os.h"
#include "bsp_dht11.h"
#include "bsp_sensor.h"
#include "bsp_pid.h"

/* Private define ------------------------------------------------------------*/
#define SENSOR_TICK_MS          20U     /* DHT11 状态机推进节拍 */
#define SENSOR_SAMPLE_TICKS     (SENSOR_PERIOD_MS / SENSOR_TICK_MS)  /* 25 tick = 500ms */

/* 压力坐下判定: 连续 N 次检测到 DO 有效才认为坐下(去抖) */
#define PRESSURE_CONFIRM_CNT    2U

/* Private variables ---------------------------------------------------------*/
static uint16_t s_press_confirm = 0U;   /* 压力 DO 连续有效计数 */
static uint16_t s_press_release = 0U;   /* 压力 DO 连续无效计数 */
static uint32_t s_dht_done_prev = 0U;   /* 上一次见到的 DHT11 完成次数 */

/* 增量式 PID 控制器(温控对象大惯性, 输出为风扇档位) */
static Pid_t    s_pid;
static uint32_t s_pid_elapsed_ms = 0U;  /* 距上次 PID 运算的累计时间 */

/* Private function prototypes -----------------------------------------------*/
static void Sensor_PushMessage(void);
static void Sensor_UpdatePressure(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  读取压力传感器并更新坐下标志
  * @note   主判据用 DO(数字量), 同时把 AO 原始值读进来只为 OLED 显示。
  *         加连续确认计数, 避免压力模块输出抖动导致状态跳变。
  */
static void Sensor_UpdatePressure(void)
{
    uint8_t  do_active = Sensor_PressureDoActive();
    uint16_t raw;

    if (do_active != 0U)
    {
        if (s_press_confirm < 0xFFFFU)
        {
            s_press_confirm++;
        }
        s_press_release = 0U;
    }
    else
    {
        if (s_press_release < 0xFFFFU)
        {
            s_press_release++;
        }
        s_press_confirm = 0U;
    }

    if (s_press_confirm >= PRESSURE_CONFIRM_CNT)
    {
        g_state.sensor.seated = 1U;
    }
    else if (s_press_release >= PRESSURE_CONFIRM_CNT)
    {
        g_state.sensor.seated = 0U;
    }
    /* 未达到确认次数则保持上一次状态 */

    /* AO 原始值 */
    raw = Sensor_PressureRawAdc();
    g_state.sensor.pressure = raw;
}

/** @brief 打包并投递传感器消息(队列满则丢掉最旧的一条, 保证 OLED 拿到最新数据) */
static void Sensor_PushMessage(void)
{
    SensorMsg_t msg;
    uint8_t     flags = 0U;

    if (g_state.sensor.human != 0U)     { flags |= SENSOR_FLAG_HUMAN; }
    if (g_state.sensor.seated != 0U)    { flags |= SENSOR_FLAG_SEATED; }
    if (g_state.sensor.pressure != 0U)  { flags |= SENSOR_FLAG_PRESSURE_OK; }
    if (g_state.sensor.dht_ok != 0U)    { flags |= SENSOR_FLAG_DHT_OK; }

    msg.temp      = g_state.sensor.temp;
    msg.humi      = g_state.sensor.humi;
    msg.flags     = flags;
    msg.fan_level = g_state.act.level;
    msg.pressure  = g_state.sensor.pressure;
    msg.reserved  = 0U;

    if (xQueueSensorDataHandle == NULL)
    {
        return;
    }

    /* 先尝试直接入队 */
    if (osMessageQueuePut(xQueueSensorDataHandle, &msg, 0U, 0U) == osOK)
    {
        return;
    }

    /* 队列满: 丢一条最旧的再放, 保证消费者总能拿到最新数据 */
    {
        SensorMsg_t drop;
        (void)osMessageQueueGet(xQueueSensorDataHandle, &drop, NULL, 0U);
        (void)osMessageQueuePut(xQueueSensorDataHandle, &msg, 0U, 0U);
    }
}

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化传感器 */
void AppSensor_Init(void)
{
    s_press_confirm  = 0U;
    s_press_release  = 0U;
    s_dht_done_prev  = DHT11_GetDoneCount();

    Sensor_Init();      /* 红外/压力 GPIO */
    DHT11_Init();       /* 温湿度单总线 */

    /* ---- 增量式 PID 初始化 ----
       输出限幅取 1~5 档: PID 只在自动模式已满足启动条件后运行,
       最低给 1 档(不负责关机, 关机由 30s 无人超时 / 2℃ 回差决定),
       最高给 5 档, 避免积分把输出顶出档位范围。 */
    PID_Init(&s_pid, PID_KP, PID_KI, PID_KD, 1, (int32_t)FAN_LEVEL_MAX);
    PID_SetDeadband(&s_pid, PID_DEADBAND_X10);
    s_pid_elapsed_ms = 0U;

    g_state.sensor.human    = Sensor_InfraredDetected();
    g_state.sensor.seated   = 0U;
    g_state.sensor.pressure = 0U;
    g_state.sensor.temp     = 0U;
    g_state.sensor.humi     = 0U;
    g_state.sensor.dht_ok   = 0U;
}

/** @brief 复位 PID 状态(进入/退出自动模式时调用) */
void AppSensor_PidReset(void)
{
    PID_Reset(&s_pid);
    s_pid_elapsed_ms = 0U;

    /* 让 PID 输出跟随当前实际档位, 避免接管瞬间档位突跳 */
    PID_SetOutputLevel(&s_pid, (int32_t)g_state.act.level);
}

/** @brief 任务主循环 */
void AppSensor_Task(void *argument)
{
    uint32_t tick_count = 0U;
    uint8_t  t;
    uint8_t  h;

    (void)argument;

    for (;;)
    {
        /* ---- DHT11: 阻塞式完整时序, 内部自带 2s 间隔保护 ----
           未到间隔时几乎不耗时; 到点时阻塞约 23ms(18ms 起始 + 5ms 读 40bit)。
           这 23ms 的阻塞换来的是时序绝对可靠, 比"状态机 + tick 切分"稳得多。 */
        (void)DHT11_Process();

        /* 用"完成次数"判断是否刚结束一次读取(空闲/未到间隔时计数不变) */
        if (DHT11_GetDoneCount() != s_dht_done_prev)
        {
            s_dht_done_prev = DHT11_GetDoneCount();

            if (DHT11_Fetch(&t, &h) != 0U)
            {
                g_state.sensor.temp   = t;
                g_state.sensor.humi   = h;
                g_state.sensor.dht_ok = 1U;
            }
            else
            {
                /* 本次读取失败: 标记数据不可信, 但保留上一次的温湿度值,
                   避免 0℃ 这种假数据把自动模式的判断带跑偏 */
                g_state.sensor.dht_ok = 0U;
                g_state.stat.dht_err++;
            }
        }

        /* ---- 每 500ms 做一次完整采样 ---- */
        tick_count++;
        if (tick_count >= SENSOR_SAMPLE_TICKS)
        {
            tick_count = 0U;

            /* 红外(HC-SR501 自带延时, 不需要软件去抖) */
            g_state.sensor.human = Sensor_InfraredDetected();

            /* 压力(DO + AO) */
            Sensor_UpdatePressure();

            /* 打包给 OLED / 串口屏 */
            Sensor_PushMessage();
        }

        /* ---- 增量式 PID: 每隔 PID_PERIOD_MS 运算一次 ----
           仅在"自动模式 + 已满足启动条件(pid_run)"时运行, 运算结果通过
           xQueueCmd 以 CMD_PID_LEVEL 投递, 由 Task_MotorFan 统一执行 ——
           保持"只有 Task_MotorFan 能操作执行器"的约定, 不在本任务里调速。 */
        s_pid_elapsed_ms += SENSOR_TICK_MS;

        if (s_pid_elapsed_ms >= PID_PERIOD_MS)
        {
            s_pid_elapsed_ms = 0U;

            if ((g_state.cfg.mode == MODE_AUTO) && (g_state.autom.pid_run != 0U))
            {
                int32_t level;

                /* 采不到温度时不要瞎调, 保持当前档位 */
                if (g_state.sensor.dht_ok != 0U)
                {
                    level = PID_Compute(&s_pid,
                                        (int32_t)g_state.sensor.temp,
                                        (int32_t)g_state.cfg.temp_threshold);

                    (void)Cmd_Post((uint8_t)CMD_PID_LEVEL,
                                   (uint16_t)level,
                                   (uint8_t)SRC_INTERNAL);
                }
            }
        }

        osDelay(SENSOR_TICK_MS);
    }
}
