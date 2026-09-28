/**
  ******************************************************************************
  * @file    app_state.h
  * @brief   智能小风扇 —— 全局状态与应用常量
  * @note    所有全局状态统一收敛在 g_state 一个结构体里, 禁止零散全局变量。
  *          STM32F103C8T6 RAM 只有 20KB, 大数组一律全局静态分配。
  ******************************************************************************
  */

#ifndef __APP_STATE_H
#define __APP_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*==============================================================================
 * 一、可调参数（改这里即可，不用翻驱动）
 *============================================================================*/

/* ---- 风扇档位 -> PWM 占空比(千分比) 映射表, 用户指定: 40/55/70/85/100% ---- */
#define FAN_LEVEL_MAX               5U
#define FAN_DUTY_LEVEL0             0U      /* 0 档 = 停 */
#define FAN_DUTY_LEVEL1             400U    /* 40%  */
#define FAN_DUTY_LEVEL2             550U    /* 55%  */
#define FAN_DUTY_LEVEL3             700U    /* 70%  */
#define FAN_DUTY_LEVEL4             850U    /* 85%  */
#define FAN_DUTY_LEVEL5             1000U   /* 100% */

/* ---- 温度阈值 ---- */
#define TEMP_THRESHOLD_DEFAULT      28U     /* 出厂默认开启阈值 ℃ */
#define TEMP_THRESHOLD_MIN          10U     /* 允许用户设置的最小值 */
#define TEMP_THRESHOLD_MAX          45U     /* 允许用户设置的最大值 */
#define TEMP_HYSTERESIS             2U      /* 回差: 低于 (阈值-2)℃ 才自动关 */

/* ---- 自动模式"人离开"判定 ---- */
#define AUTO_NOBODY_TIMEOUT_MS      30000U  /* 无人体且无坐下持续 30s -> 自动关 */

/* ---- 蜂鸣器报警(A 方案: 滴 200ms 停 1.8s 循环, 30s 后静音) ---- */
#define BUZZ_BEEP_ON_MS             200U
#define BUZZ_BEEP_OFF_MS            1800U
#define BUZZ_ALARM_SILENCE_MS       30000U  /* 报警持续超过该时间 -> 静音 */

/* ---- 舵机摇头(非阻塞状态机) ----
 * 维特/华馨京 PWM 数字舵机: 周期 20ms, 脉宽 500~2500us, 中位 1500us
 * 500~2500us 对应 0~270°。软件限幅到安全区间, 避免顶到机械限位烧舵机。 */
#define SERVO_PULSE_MIN_US          500U    /* 对应 0°   */
#define SERVO_PULSE_MAX_US          2500U   /* 对应 270° */
#define SERVO_PULSE_MID_US          1500U   /* 对应 135° */
#define SERVO_ANGLE_MIN             45U     /* 摇头左限幅 45°  */
#define SERVO_ANGLE_MAX             225U    /* 摇头右限幅 225° */
#define SERVO_MOVE_DWELL_MS         1200U   /* 转到一侧后停留时间 */
#define SERVO_CENTER_DWELL_MS       300U    /* 回中停留时间 */

/* ---- 命令去抖(防止串口屏/语音连击导致档位飞跳) ---- */
#define CMD_DEBOUNCE_MS             200U

/* ---- 增量式 PID 恒温调速 ----
 * 控制目标: 自动模式下, 以"当前温度阈值"为设定值, 通过调节风扇档位把温度
 *           维持在设定值附近(而不是"开到底"), 从而降低温度波动、兼顾功耗与噪音。
 * 整定说明:
 *   - 输出量是"档位"(1~5 的离散档), 不是连续占空比, 因此输出限幅就是 1~5 档;
 *   - 被控对象(房间温度 + 风扇)时间常数在分钟级, 增量式 PID 的采样周期取
 *     对象时间常数的 1/10 ~ 1/5, 这里取 10s;
 *   - 死区 ±0.5℃: DHT11 分辨率为 1℃, 若目标恰在两个整数温度之间, 误差会在
 *     ±1 之间来回跳, 没有死区会导致档位反复加减(稳态抖动)。
 *
 *   【重要】DHT11 必须放在远离风扇出风口的位置(测环境温度, 不是测风温),
 *          否则构成正反馈, PID 会一路饱和到最高档无法收敛。
 */
#define PID_PERIOD_MS               10000U  /* 每 10s 执行一次 PID 运算 */
#define PID_KP                      30      /* 比例增益 x100 */
#define PID_KI                      12      /* 积分增益 x100 */
#define PID_KD                      4       /* 微分增益 x100, 抑制温度突变 */
#define PID_DEADBAND_X10            5       /* 死区 ±0.5℃ (单位 0.1℃) */

/* ---- 周期 ---- */
#define SENSOR_PERIOD_MS            500U    /* 传感器采集周期 */
#define LED_PERIOD_MS               200U    /* OLED 刷新周期 */

/*==============================================================================
 * 二、运行模式 / 摇头状态
 *============================================================================*/
typedef enum {
    MODE_AUTO = 0,      /* 自动模式: 红外或压力 + 温度超阈值 -> 开风扇 */
    MODE_MANUAL = 1     /* 手动模式: 用户直接指定开关和档位 */
} FanMode_t;

typedef enum {
    SWING_OFF = 0,      /* 不摇头 */
    SWING_ON  = 1       /* 摇头 */
} SwingState_t;

typedef enum {
    SERVO_IDLE = 0,     /* 停在中位 */
    SERVO_GO_LEFT,      /* 转向左限位 */
    SERVO_DWELL_LEFT,   /* 左停留 */
    SERVO_GO_CENTER,    /* 回中 */
    SERVO_DWELL_CENTER, /* 中位停留 */
    SERVO_GO_RIGHT,     /* 转向右限位 */
    SERVO_DWELL_RIGHT   /* 右停留 */
} ServoPhase_t;

/*==============================================================================
 * 三、消息队列数据结构（必须 <= CubeMX 里配置的队列元素字节数）
 *============================================================================*/

/* xQueueCmd 元素大小 = 8 字节。命令类型 + 参数, UI/蓝牙/语音统一投递 */
typedef struct {
    uint8_t  cmd;        /* 见 cmd.h 的 CMD_xxx */
    uint8_t  source;     /* 见 cmd.h 的 SRC_xxx, 仅用于调试 */
    uint16_t param;      /* 命令参数(温度阈值 / 倒计时秒数 / 档位...) */
    uint32_t tick;       /* 投递时刻, 用于去抖 */
} CmdMsg_t;              /* 8 字节, 刚好 */

/* xQueueSensorData 元素大小 = 8 字节。压进 8 字节以匹配现有 CubeMX 配置 */
typedef struct {
    uint8_t  temp;       /* 温度 ℃, 0~60             */
    uint8_t  humi;       /* 湿度 %, 0~100            */
    uint8_t  flags;      /* 见 SENSOR_FLAG_xxx       */
    uint8_t  fan_level;  /* 采集时刻的档位快照       */
    uint16_t pressure;   /* 压力 ADC 原始值 0~4095   */
    uint16_t reserved;   /* 凑齐 8 字节, 便于以后扩展 */
} SensorMsg_t;

#define SENSOR_FLAG_HUMAN       0x01U   /* 人体红外检测到人 */
#define SENSOR_FLAG_SEATED      0x02U   /* 压力传感器判为坐下 */
#define SENSOR_FLAG_PRESSURE_OK 0x04U   /* 压力通道本次读数有效 */
#define SENSOR_FLAG_DHT_OK      0x08U   /* 温湿度本次读数有效 */

/*==============================================================================
 * 四、全局状态结构体（唯一全局实例 g_state）
 *============================================================================*/
typedef struct {
    /* --- 传感器实时数据 (Task_Sensor 写, 其他任务只读) --- */
    struct {
        uint8_t  temp;          /* 温度 ℃ */
        uint8_t  humi;          /* 湿度 % */
        uint8_t  dht_ok;        /* 1 = 上一次 DHT11 读取成功 */
        uint8_t  human;         /* 1 = 人体红外检测到人 */
        uint8_t  seated;        /* 1 = 压力判为坐下 */
        uint16_t pressure;      /* 压力 ADC 原始值 */
    } sensor;

    /* --- 用户配置 (Task_MotorFan 写, UI 任务只读) --- */
    struct {
        FanMode_t     mode;             /* 自动 / 手动 */
        uint8_t       temp_threshold;   /* 自动模式温度阈值 ℃ */
        SwingState_t  swing;            /* 摇头开/关 */
    } cfg;

    /* --- 执行器当前状态 (Task_MotorFan 独占写) --- */
    struct {
        uint8_t       fan_on;           /* 风扇是否在转 */
        uint8_t       level;            /* 当前档位 0~5 */
        uint16_t      duty;             /* 当前占空比千分比 */
        uint8_t       buzzer_on;        /* 蜂鸣器当前是否发声 */
        ServoPhase_t  servo_phase;      /* 舵机状态机阶段 */
        uint16_t      servo_angle;      /* 舵机当前角度 */
    } act;

    /* --- 倒计时 (仅 Task_MotorFan 读写, 单位 ms, 0 = 未启用) --- */
    struct {
        uint32_t  remain_ms;    /* 剩余时间 */
        uint32_t  set_ms;       /* 本次设置的总时长 */
        uint8_t   active;       /* 1 = 正在倒计时 */
    } timer;

    /* --- 自动模式内部逻辑 --- */
    struct {
        uint32_t  nobody_ms;    /* 无人持续时长, 达到 AUTO_NOBODY_TIMEOUT_MS 自动关 */
        uint32_t  alarm_ms;     /* 报警已持续时长, 超过 BUZZ_ALARM_SILENCE_MS 静音 */
        uint8_t   alarm_on;     /* 是否处于报警状态 */
        uint8_t   pid_run;      /* 1 = 自动模式已满足启动条件, 风扇正由 PID 恒温调速 */
    } autom;

    /* --- 调试统计 --- */
    struct {
        uint32_t  cmd_rx;       /* 收到的有效命令数 */
        uint32_t  cmd_drop;     /* 队列满被丢弃的命令数 */
        uint32_t  dht_err;      /* DHT11 连续失败计数 */
    } stat;
} AppState_t;

/* 全局唯一状态实例 */
extern AppState_t g_state;

/*==============================================================================
 * 五、对外接口
 *============================================================================*/

/* 上电初始化: 把所有字段置为安全初值 */
void AppState_Init(void);

/* 按档位取占空比千分比, level 自动限幅到 0~FAN_LEVEL_MAX */
uint16_t AppState_LevelToDuty(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* __APP_STATE_H */
