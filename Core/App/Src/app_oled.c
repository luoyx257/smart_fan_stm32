/**
  ******************************************************************************
  * @file    app_oled.c
  * @brief   OLED 显示任务实现
  * @note    显示布局(128x64, 4 行 8x16 字符, 每行最多 16 字符):
  *            行0:  T:25C H:60%
  *            行1:  MODE:AUTO FAN:3
  *            行2:  TIME:04:35        (无倒计时时显示 SPEED:70%)
  *            行3:  SWING:ON  P:1234
  *          "脏行"机制: 内容没变就不刷该行, 省 I2C 带宽。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_oled.h"
#include "app_state.h"
#include "cmd.h"            /* 只为拿到 xQueueSensorDataHandle 声明 */
#include "cmsis_os.h"
#include "bsp_oled.h"

/* Private define ------------------------------------------------------------*/
#define OLED_ROWS           4U
#define OLED_COLS           16U     /* 每行 16 个 8 像素宽字符 */
#define OLED_ROW_CHARS      17U     /* 含结束符 */

/* Private variables ---------------------------------------------------------*/
static uint8_t  s_oled_ok = 0U;                     /* OLED 是否在线 */
static char     s_line[OLED_ROWS][OLED_ROW_CHARS];  /* 当前显示内容 */
static char     s_prev[OLED_ROWS][OLED_ROW_CHARS];  /* 上一次已刷内容, 用于比较 */

/* 传感器数据本地缓存(队列每 500ms 更新一次, 两次之间沿用旧值) */
static uint8_t  s_temp      = 0U;
static uint8_t  s_humi      = 0U;
static uint8_t  s_human     = 0U;
static uint8_t  s_seated    = 0U;
static uint8_t  s_dht_ok    = 0U;

/* Private function prototypes -----------------------------------------------*/
static void Oled_UpdateStrings(void);
static void Oled_Refresh(void);
static void Oled_FormatMmSs(char *dst, uint32_t ms);
static void Oled_PutStr(char *dst, const char *src, uint8_t *pos);

/* Private functions ---------------------------------------------------------*/

/** @brief 把某一行补齐到固定 16 字符宽(右侧补空格)。
  *        必须补成等宽, 否则行与行比较长度不同会导致误判"内容变了"而反复刷屏。 */
static void Oled_PadRow(uint8_t row)
{
    uint8_t i;

    for (i = 0U; i < OLED_COLS; i++)
    {
        if (s_line[row][i] == '\0')
        {
            s_line[row][i] = ' ';
        }
    }
    s_line[row][OLED_COLS] = '\0';
}

/** @brief 往行缓冲追加字符串 */
static void Oled_PutStr(char *dst, const char *src, uint8_t *pos)
{
    while ((*src != '\0') && (*pos < OLED_COLS))
    {
        dst[*pos] = *src;
        (*pos)++;
        src++;
    }
    dst[*pos] = '\0';
}

/** @brief 往行缓冲追加数字 */
static void Oled_PutNum(char *dst, uint32_t value, uint8_t width, uint8_t *pos)
{
    char tmp[11];
    uint8_t len = 0U;
    uint8_t i;

    if (value == 0U)
    {
        tmp[len++] = '0';
    }
    else
    {
        while ((value > 0U) && (len < sizeof(tmp)))
        {
            tmp[len++] = (char)('0' + (value % 10U));
            value /= 10U;
        }
    }
    while ((len < width) && (len < sizeof(tmp)))
    {
        tmp[len++] = '0';
    }
    for (i = 0U; i < len; i++)
    {
        if (*pos >= OLED_COLS)
        {
            break;
        }
        dst[*pos] = tmp[len - 1U - i];
        (*pos)++;
    }
    dst[*pos] = '\0';
}

/** @brief 把毫秒格式化成 mm:ss */
static void Oled_FormatMmSs(char *dst, uint32_t ms)
{
    uint32_t total_s = ms / 1000UL;
    uint32_t mm = total_s / 60UL;
    uint32_t ss = total_s % 60UL;

    if (mm > 99UL)
    {
        mm = 99UL;
    }

    dst[0] = (char)('0' + (mm / 10UL));
    dst[1] = (char)('0' + (mm % 10UL));
    dst[2] = ':';
    dst[3] = (char)('0' + (ss / 10UL));
    dst[4] = (char)('0' + (ss % 10UL));
    dst[5] = '\0';
}

/** @brief 根据 g_state 组装 4 行显示内容 */
static void Oled_UpdateStrings(void)
{
    uint8_t pos;

    /* ---------------- 行 0: 温湿度 ---------------- */
    pos = 0U;
    s_line[0][0] = '\0';
    Oled_PutStr(s_line[0], (s_dht_ok != 0U) ? "T:" : "T?", &pos);
    Oled_PutNum(s_line[0], s_temp, 2U, &pos);
    Oled_PutStr(s_line[0], "C H:", &pos);
    Oled_PutNum(s_line[0], s_humi, 2U, &pos);
    Oled_PutStr(s_line[0], "%", &pos);

    /* ---------------- 行 1: 模式 + 档位 + 人体 ---------------- */
    pos = 0U;
    s_line[1][0] = '\0';
    if (g_state.cfg.mode == MODE_AUTO)
    {
        /* PID 接管时用 "PID" 区分, 方便直观看到恒温调速是否在工作 */
        Oled_PutStr(s_line[1], (g_state.autom.pid_run != 0U) ? "PID " : "AUTO", &pos);
    }
    else
    {
        Oled_PutStr(s_line[1], "MANU", &pos);
    }
    Oled_PutStr(s_line[1], " F:", &pos);
    Oled_PutNum(s_line[1], g_state.act.level, 1U, &pos);
    Oled_PutStr(s_line[1], (s_human != 0U) ? " P" : " -", &pos);
    Oled_PutStr(s_line[1], (s_seated != 0U) ? "S" : "-", &pos);

    /* ---------------- 行 2: 倒计时 / 速度 ---------------- */
    pos = 0U;
    s_line[2][0] = '\0';
    if (g_state.timer.active != 0U)
    {
        char mmss[8];
        Oled_FormatMmSs(mmss, g_state.timer.remain_ms);
        Oled_PutStr(s_line[2], "TMR ", &pos);
        Oled_PutStr(s_line[2], mmss, &pos);
    }
    else
    {
        Oled_PutStr(s_line[2], "SPD ", &pos);
        Oled_PutNum(s_line[2], g_state.act.duty / 10U, 3U, &pos);    /* 千分比 -> 百分比 */
        Oled_PutStr(s_line[2], "%", &pos);
    }
    Oled_PutStr(s_line[2], " TH", &pos);
    Oled_PutNum(s_line[2], g_state.cfg.temp_threshold, 2U, &pos);

    /* ---------------- 行 3: 摇头 + 舵机角度 ---------------- */
    pos = 0U;
    s_line[3][0] = '\0';
    Oled_PutStr(s_line[3], "SWING:", &pos);
    Oled_PutStr(s_line[3], (g_state.cfg.swing == SWING_ON) ? "ON " : "OFF", &pos);
    Oled_PutStr(s_line[3], " A", &pos);
    Oled_PutNum(s_line[3], g_state.act.servo_angle, 3U, &pos);

    /* ---------------- 统一补齐到 16 字符 ---------------- */
    Oled_PadRow(0U);
    Oled_PadRow(1U);
    Oled_PadRow(2U);
    Oled_PadRow(3U);
}

/** @brief 只刷内容变化的行 */
static void Oled_Refresh(void)
{
    uint8_t row;

    if (s_oled_ok == 0U)
    {
        return;
    }

    for (row = 0U; row < OLED_ROWS; row++)
    {
        uint8_t i;
        uint8_t same = 1U;

        for (i = 0U; i <= OLED_COLS; i++)
        {
            if (s_line[row][i] != s_prev[row][i])
            {
                same = 0U;
                break;
            }
        }
        if (same != 0U)
        {
            continue;       /* 该行没变化, 不刷 */
        }

        /* 行内容已是等宽 16 字符, 直接整行重画(页式刷新必须整行覆盖) */
        OLED_DrawString(0U, row, s_line[row]);

        /* 记录已刷内容 */
        for (i = 0U; i <= OLED_COLS; i++)
        {
            s_prev[row][i] = s_line[row][i];
        }
    }
}

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化 OLED */
void AppOled_Init(void)
{
    uint8_t row;
    uint8_t i;

    s_oled_ok = 0U;

    for (row = 0U; row < OLED_ROWS; row++)
    {
        for (i = 0U; i <= OLED_COLS; i++)
        {
            s_line[row][i] = ' ';
            s_prev[row][i] = 0x00U;     /* 保证第一轮全部刷新 */
        }
        s_line[row][OLED_COLS] = '\0';
        s_prev[row][OLED_COLS] = '\0';
    }

    /* 常见 SSD1306 模块两种地址都试一遍 */
    if (OLED_Init(OLED_I2C_ADDR_0X3C) != 0U)
    {
        s_oled_ok = 1U;
    }
    else if (OLED_Init(OLED_I2C_ADDR_0X3D) != 0U)
    {
        s_oled_ok = 1U;
    }
    else
    {
        s_oled_ok = 0U;     /* 没接到屏也不影响其它功能 */
    }
}

/** @brief 任务主循环 */
void AppOled_Task(void *argument)
{
    SensorMsg_t msg;

    (void)argument;

    for (;;)
    {
        /* 等传感器数据, 最长等 LED_PERIOD_MS。超时也刷一次,
           目的是让模式/档位/倒计时等"非传感器"变化也能及时显示 */
        if (osMessageQueueGet(xQueueSensorDataHandle, &msg, NULL,
                              LED_PERIOD_MS) == osOK)
        {
            s_temp     = msg.temp;
            s_humi     = msg.humi;
            s_human    = ((msg.flags & SENSOR_FLAG_HUMAN) != 0U) ? 1U : 0U;
            s_seated   = ((msg.flags & SENSOR_FLAG_SEATED) != 0U) ? 1U : 0U;
            s_dht_ok   = ((msg.flags & SENSOR_FLAG_DHT_OK) != 0U) ? 1U : 0U;
        }

        Oled_UpdateStrings();
        Oled_Refresh();
    }
}
