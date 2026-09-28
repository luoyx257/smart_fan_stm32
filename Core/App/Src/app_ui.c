/**
  ******************************************************************************
  * @file    app_ui.c
  * @brief   串口屏任务实现
  * @note    串口屏 -> 单片机 的帧格式(自定义, 见 USER_PROTOCOL.md):
  *            '@' CMD(1 字节) VALUE(4 字节 ASCII 数字, 高位在前)
  *          例如点击"自动"按钮:  '@' '3' "0000"
  *          CMD 为 ASCII 字符, 与 USER_PROTOCOL.md 中的"命令码"一致。
  *
  *          在 TJC 上位机里, 按钮的"触摸按下事件"用下面这种代码产生:
  *            printh 40 33 30 30 30 30 0D 0A
  *          (40='@', 33='3', 30 30 30 30='0000', 0D 0A=回车换行)
  *          上面这行就是"切换到自动模式"。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_ui.h"
#include "app_state.h"
#include "cmd.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "bsp_screen.h"

/* Private define ------------------------------------------------------------*/
#define UI_FRAME_PREFIX         '@'
#define UI_VALUE_DIGITS         4U      /* 参数字段固定 4 位 ASCII 数字 */
#define UI_MAX_VALUES           4U      /* 一个 CMD 最多携带 4 个参数 */
#define UI_INTERLEAVE_MS        5000U   /* 强制整体刷新周期 */

/* 上报节奏: 请求上报后至少间隔这么久再发, 避免状态频繁跳变导致刷屏太密 */
#define UI_REPORT_MIN_GAP_MS    150U

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    UI_RX_IDLE = 0,
    UI_RX_CMD,          /* 已收到 '@', 等命令字符 */
    UI_RX_VALUE,        /* 收参数 ASCII 数字 */
    UI_RX_DELIM         /* 值班等待分隔符(逗号/结束符) */
} UiRxState_t;

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t s_report_req = 0U;     /* 上报请求计数器 */
static uint32_t          s_report_done = 0U;    /* 已处理到的请求号 */
static uint32_t          s_last_report_tick = 0U;
static uint32_t          s_last_full_tick = 0U;

static UiRxState_t s_rx_state = UI_RX_IDLE;
static uint8_t     s_rx_cmd = 0U;
static uint32_t    s_rx_values[UI_MAX_VALUES];
static uint8_t     s_rx_value_count = 0U;
static uint8_t     s_rx_digit_count = 0U;
static uint8_t     s_rx_has_value = 0U;        /* 当前正在收集的那一项有无数字 */

/* Private function prototypes -----------------------------------------------*/
static void Ui_ResetFrame(void);
static void Ui_FeedByte(uint8_t byte);
static void Ui_Dispatch(uint8_t cmd, const uint32_t *values, uint8_t count);
static void Ui_ReportAll(void);
static void Ui_ReportRuntime(void);
static const char *Ui_ModeName(void);

/* Private functions ---------------------------------------------------------*/

/** @brief 重置解析状态 */
static void Ui_ResetFrame(void)
{
    s_rx_state       = UI_RX_IDLE;
    s_rx_cmd         = 0U;
    s_rx_value_count = 0U;
    s_rx_digit_count = 0U;
    s_rx_has_value   = 0U;
}

/** @brief 把一个参数收尾 */
static void Ui_CloseValue(void)
{
    if ((s_rx_has_value != 0U) && (s_rx_value_count < UI_MAX_VALUES))
    {
        s_rx_value_count++;
    }
    s_rx_has_value   = 0U;
    s_rx_digit_count = 0U;
}

/**
  * @brief  字节级帧解析
  * @note   支持 "@CMD v1,v2" 形式; 参数可选, 缺省按 0 处理。
  *         结束条件是任意非数字非逗号字符(通常是 '\n' / 0xFF)。
  */
static void Ui_FeedByte(uint8_t byte)
{
    switch (s_rx_state)
    {
    case UI_RX_IDLE:
        if (byte == (uint8_t)UI_FRAME_PREFIX)
        {
            Ui_ResetFrame();
            s_rx_state = UI_RX_CMD;
        }
        break;

    case UI_RX_CMD:
        /* 命令字符: 取可打印字符 */
        if ((byte >= 0x21U) && (byte <= 0x7EU))
        {
            s_rx_cmd   = byte;
            s_rx_state = UI_RX_DELIM;
        }
        else
        {
            Ui_ResetFrame();
        }
        break;

    case UI_RX_DELIM:
        if ((byte >= '0') && (byte <= '9'))
        {
            /* 跳过可能存在的空格, 直接进数字收集 */
            Ui_CloseValue();
            if (s_rx_value_count < UI_MAX_VALUES)
            {
                s_rx_values[s_rx_value_count] = (uint32_t)(byte - '0');
                s_rx_digit_count = 1U;
                s_rx_has_value   = 1U;
                s_rx_state       = UI_RX_VALUE;
            }
            else
            {
                Ui_ResetFrame();
            }
        }
        else if ((byte == ' ') || (byte == '='))
        {
            /* 忽略分隔空白 */
        }
        else if ((byte == '\n') || (byte == '\r') || (byte == 0xFFU))
        {
            /* 无参数命令 */
            Ui_Dispatch(s_rx_cmd, s_rx_values, s_rx_value_count);
            Ui_ResetFrame();
        }
        else
        {
            /* 其它字符当作噪声, 丢弃整帧 */
            Ui_ResetFrame();
        }
        break;

    case UI_RX_VALUE:
        if ((byte >= '0') && (byte <= '9'))
        {
            if (s_rx_digit_count < 9U)
            {
                s_rx_values[s_rx_value_count] =
                    (s_rx_values[s_rx_value_count] * 10U) + (uint32_t)(byte - '0');
                s_rx_digit_count++;
            }
            /* 位数超限就忽略多余位, 不丢帧 */
        }
        else if (byte == ',')
        {
            Ui_CloseValue();
            s_rx_state = UI_RX_DELIM;
        }
        else if ((byte == '\n') || (byte == '\r') || (byte == 0xFFU))
        {
            Ui_CloseValue();
            Ui_Dispatch(s_rx_cmd, s_rx_values, s_rx_value_count);
            Ui_ResetFrame();
        }
        else
        {
            Ui_ResetFrame();
        }
        break;

    default:
        Ui_ResetFrame();
        break;
    }
}

/**
  * @brief  把解析出的屏幕命令翻译成统一命令
  * @note   命令码定义见 USER_PROTOCOL.md, 由用户在上位机里对应配置
  */
static void Ui_Dispatch(uint8_t cmd, const uint32_t *values, uint8_t count)
{
    uint32_t v0 = (count > 0U) ? values[0] : 0U;

    switch (cmd)
    {
    case '0':       /* 风扇开 */
        (void)Cmd_Post((uint8_t)CMD_FAN_ON, 0U, (uint8_t)SRC_UART_UI);
        break;

    case '1':       /* 风扇关 */
        (void)Cmd_Post((uint8_t)CMD_FAN_OFF, 0U, (uint8_t)SRC_UART_UI);
        break;

    case '2':       /* 手动模式 */
        (void)Cmd_Post((uint8_t)CMD_MODE_SET, (uint16_t)MODE_MANUAL, (uint8_t)SRC_UART_UI);
        break;

    case '3':       /* 自动模式 */
        (void)Cmd_Post((uint8_t)CMD_MODE_SET, (uint16_t)MODE_AUTO, (uint8_t)SRC_UART_UI);
        break;

    case '4':       /* 档位直接设定: 参数 0~5 */
        (void)Cmd_Post((uint8_t)CMD_LEVEL_SET, (uint16_t)v0, (uint8_t)SRC_UART_UI);
        break;

    case '5':       /* 温度阈值设定: 参数 10~45 */
        (void)Cmd_Post((uint8_t)CMD_TEMP_SET, (uint16_t)v0, (uint8_t)SRC_UART_UI);
        break;

    case '6':       /* 倒计时: 参数为秒, 0 = 取消 */
        (void)Cmd_Post((uint8_t)CMD_COUNTDOWN_SET, (uint16_t)v0, (uint8_t)SRC_UART_UI);
        break;

    case '7':       /* 摇头开 */
        (void)Cmd_Post((uint8_t)CMD_SWING_SET, (uint16_t)SWING_ON, (uint8_t)SRC_UART_UI);
        break;

    case '8':       /* 摇头关 */
        (void)Cmd_Post((uint8_t)CMD_SWING_SET, (uint16_t)SWING_OFF, (uint8_t)SRC_UART_UI);
        break;

    case '9':       /* 档位 +1 */
        (void)Cmd_Post((uint8_t)CMD_LEVEL_UP, 0U, (uint8_t)SRC_UART_UI);
        break;

    case 'A':       /* 档位 -1 */
        (void)Cmd_Post((uint8_t)CMD_LEVEL_DOWN, 0U, (uint8_t)SRC_UART_UI);
        break;

    case 'B':       /* 模式切换 */
        (void)Cmd_Post((uint8_t)CMD_MODE_TOGGLE, 0U, (uint8_t)SRC_UART_UI);
        break;

    case 'C':       /* 摇头切换 */
        (void)Cmd_Post((uint8_t)CMD_SWING_TOGGLE, 0U, (uint8_t)SRC_UART_UI);
        break;

    case 'D':       /* 风扇开关切换 */
        (void)Cmd_Post((uint8_t)CMD_FAN_TOGGLE, 0U, (uint8_t)SRC_UART_UI);
        break;

    case 'E':       /* 取消倒计时 */
        (void)Cmd_Post((uint8_t)CMD_COUNTDOWN_CANCEL, 0U, (uint8_t)SRC_UART_UI);
        break;

    case 'F':       /* 请求上报一次状态 */
        AppUi_RequestReport();
        break;

    default:
        /* 未知命令码: 忽略 */
        break;
    }
}

/** @brief 模式名称 */
static const char *Ui_ModeName(void)
{
    return (g_state.cfg.mode == MODE_AUTO) ? "AUTO" : "MANU";
}

/**
  * @brief  整体上报一次(切页面后或开机时用)
  * @note   控件名与上位机约定, 见 USER_PROTOCOL.md:
  *           t0 = 温度(数字)   t1 = 湿度(数字)
  *           t2 = 模式(文本)   t3 = 档位(数字)
  *           t4 = 剩余时间秒   t5 = 摇头状态(文本)
  *           t6 = 温度阈值(数字) t7 = 风扇状态(文本)
  */
static void Ui_ReportAll(void)
{
    Screen_SetNumber("t0", (int32_t)g_state.sensor.temp);
    Screen_SetNumber("t1", (int32_t)g_state.sensor.humi);
    Screen_SetText("t2", Ui_ModeName());
    Screen_SetNumber("t3", (int32_t)g_state.act.level);
    Screen_SetNumber("t4", (int32_t)(g_state.timer.remain_ms / 1000UL));
    Screen_SetText("t5", (g_state.cfg.swing == SWING_ON) ? "ON" : "OFF");
    Screen_SetNumber("t6", (int32_t)g_state.cfg.temp_threshold);
    Screen_SetText("t7", (g_state.act.fan_on != 0U) ? "ON" : "OFF");
}

/**
  * @brief  只上报会变化的关键项(周期刷新用, 报文更短)
  */
static void Ui_ReportRuntime(void)
{
    Screen_SetNumber("t0", (int32_t)g_state.sensor.temp);
    Screen_SetNumber("t1", (int32_t)g_state.sensor.humi);
    Screen_SetText("t2", Ui_ModeName());
    Screen_SetNumber("t3", (int32_t)g_state.act.level);
    Screen_SetNumber("t4", (int32_t)(g_state.timer.remain_ms / 1000UL));
    Screen_SetText("t5", (g_state.cfg.swing == SWING_ON) ? "ON" : "OFF");
    Screen_SetText("t7", (g_state.act.fan_on != 0U) ? "ON" : "OFF");
}

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化 */
void AppUi_Init(void)
{
    Screen_Init();
    BspUart_Flush(UART_ID_UI);
    Ui_ResetFrame();

    s_report_req        = 1U;       /* 开机强制上报一次 */
    s_report_done       = 0U;
    s_last_report_tick  = 0U;
    s_last_full_tick    = 0U;
}

/** @brief 请求上报 */
void AppUi_RequestReport(void)
{
    s_report_req++;
}

/** @brief 任务主循环 */
void AppUi_Task(void *argument)
{
    uint8_t  byte;
    uint32_t now;

    (void)argument;

    for (;;)
    {
        /* ---- 1) 把接收缓冲里的字节全部喂给解析器 ---- */
        while (BspUart_GetByte(UART_ID_UI, &byte) != 0U)
        {
            Ui_FeedByte(byte);
        }

        now = (uint32_t)osKernelGetTickCount();

        /* ---- 2) 按需上报 ---- */
        if ((s_report_req != s_report_done) &&
            ((uint32_t)(now - s_last_report_tick) >= UI_REPORT_MIN_GAP_MS))
        {
            s_report_done      = s_report_req;
            s_last_report_tick = now;
            Ui_ReportRuntime();
        }

        /* ---- 3) 周期性整体刷新(含阈值等不常变项) ---- */
        if ((uint32_t)(now - s_last_full_tick) >= UI_INTERLEAVE_MS)
        {
            s_last_full_tick = now;
            Ui_ReportAll();
        }

        osDelay(20U);
    }
}
