/**
  ******************************************************************************
  * @file    app_bt.c
  * @brief   HC-05 蓝牙文本行协议解析任务实现
  * @note    命令表(大小写不敏感, 以 '\n'/'\r' 结尾):
  *            ON            开风扇
  *            OFF           关风扇
  *            AUTO          自动模式
  *            MANUAL        手动模式
  *            MODE          模式切换
  *            L+ / L-       档位 +/- 1
  *            LV0 ~ LV5     直接设定档位
  *            TH20          温度阈值设为 20
  *            TIME300       倒计时 300 秒(0 表示取消)
  *            TCANCEL       取消倒计时
  *            SWING         摇头开
  *            NOSWING       摇头关
  *            SW            摇头切换
  *            STATUS        请求上报一次状态
  *          每收到一条完整命令后会回一行 "ACK:<命令>" 或 "ERR", 方便 APP 调试。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_bt.h"
#include "app_state.h"
#include "cmd.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "usart.h"

/* Private define ------------------------------------------------------------*/
#define BT_LINE_MAX         24U     /* 单行最大长度(含结束符) */
#define BT_TASK_TICK_MS     20U
#define BT_TX_TIMEOUT_MS    100U

/* Private variables ---------------------------------------------------------*/
static char s_line[BT_LINE_MAX];
static uint8_t s_line_len = 0U;

/* Private function prototypes -----------------------------------------------*/
static void Bt_HandleLine(char *line);
static void Bt_Reply(const char *text);
static uint8_t Bt_StrEq(const char *a, const char *b);
static uint8_t Bt_IsDigit(char c);
static uint32_t Bt_ParseUint(const char *s, uint8_t *ok);

/* Private functions ---------------------------------------------------------*/

/** @brief 大小写不敏感比较(必须以 '\0' 同时结束) */
static uint8_t Bt_StrEq(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0'))
    {
        char ca = *a;
        char cb = *b;
        if ((ca >= 'a') && (ca <= 'z')) { ca = (char)(ca - 32); }
        if ((cb >= 'a') && (cb <= 'z')) { cb = (char)(cb - 32); }
        if (ca != cb)
        {
            return 0U;
        }
        a++;
        b++;
    }
    return ((*a == '\0') && (*b == '\0')) ? 1U : 0U;
}

/** @brief 判断十进制字符 */
static uint8_t Bt_IsDigit(char c)
{
    return ((c >= '0') && (c <= '9')) ? 1U : 0U;
}

/** @brief 解析无符号整数 */
static uint32_t Bt_ParseUint(const char *s, uint8_t *ok)
{
    uint32_t value = 0U;
    uint8_t  any   = 0U;

    while (Bt_IsDigit(*s) != 0U)
    {
        if (value < 1000000U)       /* 防溢出, 本项目参数远小于该值 */
        {
            value = (value * 10U) + (uint32_t)(*s - '0');
        }
        any = 1U;
        s++;
    }

    *ok = any;
    return value;
}

/** @brief 回一行文本(仅用于调试, HC-05 连手机即可看到) */
static void Bt_Reply(const char *text)
{
    if (text == NULL)
    {
        return;
    }
    {
        char buf[BT_LINE_MAX + 4U];
        uint8_t i = 0U;
        while ((text[i] != '\0') && (i < BT_LINE_MAX))
        {
            buf[i] = text[i];
            i++;
        }
        buf[i++] = '\r';
        buf[i++] = '\n';
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, i, BT_TX_TIMEOUT_MS);
    }
}

/**
  * @brief  命令分发
  * @note   形如 LV3 / TH25 / TIME120 的命令, 取前缀字母后跟数字的方式解析
  */
static void Bt_HandleLine(char *line)
{
    uint8_t  ok = 0U;
    uint32_t num;

    /* 去掉首部空格 */
    while ((*line == ' ') || (*line == '\t'))
    {
        line++;
    }
    if (*line == '\0')
    {
        return;     /* 空行忽略, 不回 ERR(避免刷屏) */
    }

    /* ---------------- 纯文本命令 ---------------- */
    if (Bt_StrEq(line, "ON") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_FAN_ON, 0U, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:ON");
    }
    else if (Bt_StrEq(line, "OFF") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_FAN_OFF, 0U, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:OFF");
    }
    else if (Bt_StrEq(line, "AUTO") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_MODE_SET, (uint16_t)MODE_AUTO, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:AUTO");
    }
    else if (Bt_StrEq(line, "MANUAL") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_MODE_SET, (uint16_t)MODE_MANUAL, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:MANUAL");
    }
    else if (Bt_StrEq(line, "MODE") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_MODE_TOGGLE, 0U, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:MODE");
    }
    else if (Bt_StrEq(line, "L+") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_LEVEL_UP, 0U, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:L+");
    }
    else if (Bt_StrEq(line, "L-") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_LEVEL_DOWN, 0U, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:L-");
    }
    else if (Bt_StrEq(line, "SWING") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_SWING_SET, (uint16_t)SWING_ON, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:SWING");
    }
    else if (Bt_StrEq(line, "NOSWING") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_SWING_SET, (uint16_t)SWING_OFF, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:NOSWING");
    }
    else if (Bt_StrEq(line, "SW") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_SWING_TOGGLE, 0U, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:SW");
    }
    else if (Bt_StrEq(line, "TCANCEL") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_COUNTDOWN_CANCEL, 0U, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:TCANCEL");
    }
    else if (Bt_StrEq(line, "STATUS") != 0U)
    {
        (void)Cmd_Post((uint8_t)CMD_QUERY_STATUS, 0U, (uint8_t)SRC_BLUETOOTH);
        Bt_Reply("ACK:STATUS");
    }
    /* ---------------- 带数字参数的命令 ---------------- */
    else if (((line[0] == 'L') || (line[0] == 'l')) &&
             ((line[1] == 'V') || (line[1] == 'v')))
    {
        num = Bt_ParseUint(&line[2], &ok);
        if (ok != 0U)
        {
            (void)Cmd_Post((uint8_t)CMD_LEVEL_SET, (uint16_t)num, (uint8_t)SRC_BLUETOOTH);
            Bt_Reply("ACK:LV");
        }
        else
        {
            Bt_Reply("ERR:LV");
        }
    }
    else if (((line[0] == 'T') || (line[0] == 't')) &&
             ((line[1] == 'H') || (line[1] == 'h')))
    {
        num = Bt_ParseUint(&line[2], &ok);
        if (ok != 0U)
        {
            (void)Cmd_Post((uint8_t)CMD_TEMP_SET, (uint16_t)num, (uint8_t)SRC_BLUETOOTH);
            Bt_Reply("ACK:TH");
        }
        else
        {
            Bt_Reply("ERR:TH");
        }
    }
    else if (((line[0] == 'T') || (line[0] == 't')) &&
             ((line[1] == 'I') || (line[1] == 'i')))
    {
        num = Bt_ParseUint(&line[4], &ok);
        if (ok != 0U)
        {
            (void)Cmd_Post((uint8_t)CMD_COUNTDOWN_SET, (uint16_t)num, (uint8_t)SRC_BLUETOOTH);
            Bt_Reply("ACK:TIME");
        }
        else
        {
            Bt_Reply("ERR:TIME");
        }
    }
    else
    {
        Bt_Reply("ERR");
    }
}

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化 */
void AppBt_Init(void)
{
    s_line_len = 0U;
    s_line[0]  = '\0';
    BspUart_Flush(UART_ID_BT);
}

/** @brief 任务主循环 */
void AppBt_Task(void *argument)
{
    uint8_t byte;

    (void)argument;

    for (;;)
    {
        while (BspUart_GetByte(UART_ID_BT, &byte) != 0U)
        {
            if ((byte == '\n') || (byte == '\r'))
            {
                if (s_line_len > 0U)
                {
                    s_line[s_line_len] = '\0';
                    Bt_HandleLine(s_line);
                    s_line_len = 0U;
                    s_line[0]  = '\0';
                }
            }
            else if (s_line_len < (BT_LINE_MAX - 1U))
            {
                s_line[s_line_len] = (char)byte;
                s_line_len++;
            }
            else
            {
                /* 行太长: 丢弃整行, 防止乱码后一直拼不出完整命令 */
                s_line_len = 0U;
                s_line[0]  = '\0';
            }
        }

        osDelay(BT_TASK_TICK_MS);
    }
}
