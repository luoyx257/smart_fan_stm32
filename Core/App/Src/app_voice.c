/**
  ******************************************************************************
  * @file    app_voice.c
  * @brief   SU-03T 语音模块解析任务实现
  * @note    帧格式(5 字节): 0xAA 0x55 CMD VALUE XOR
  *            CMD   : 命令码
  *            VALUE : 参数(温度阈值 / 档位 / 分钟数), 无参数填 0
  *            XOR   : 前 4 字节异或校验
  *          SU-03T 那边用"串口输出"词条配置 16 进制数据即可, 详见
  *          USER_PROTOCOL.md 里的命令码表。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_voice.h"
#include "app_state.h"
#include "cmd.h"
#include "cmsis_os.h"
#include "bsp_uart.h"

/* Private define ------------------------------------------------------------*/
#define VOICE_HEAD0         0xAAU   /* 帧头第 1 字节 */
#define VOICE_HEAD1         0x55U   /* 帧头第 2 字节 */
#define VOICE_FRAME_LEN     5U      /* 固定 5 字节帧 */

#define VOICE_TASK_TICK_MS  20U

/* 命令码(与 SU-03T 词条一一对应) */
#define VC_FAN_ON           0x01U   /* 打开风扇 */
#define VC_FAN_OFF          0x02U   /* 关闭风扇 */
#define VC_MODE_AUTO        0x03U   /* 自动模式 */
#define VC_MODE_MANUAL      0x04U   /* 手动模式 */
#define VC_MODE_TOGGLE      0x05U   /* 切换模式 */
#define VC_LEVEL_UP         0x06U   /* 调高一档 */
#define VC_LEVEL_DOWN       0x07U   /* 调低一档 */
#define VC_LEVEL_SET        0x08U   /* 设置到第 VALUE 档 */
#define VC_TEMP_SET         0x09U   /* 温度阈值设为 VALUE */
#define VC_TEMP_UP          0x0AU   /* 温度阈值 +1 */
#define VC_TEMP_DOWN        0x0BU   /* 温度阈值 -1 */
#define VC_TIME_SET         0x0CU   /* 倒计时 VALUE 分钟 */
#define VC_TIME_CANCEL      0x0DU   /* 取消倒计时 */
#define VC_SWING_ON         0x0EU   /* 开始摇头 */
#define VC_SWING_OFF        0x0FU   /* 停止摇头 */
#define VC_SWING_TOGGLE     0x10U   /* 摇头开关切换 */
#define VC_QUERY_STATUS     0x11U   /* 播报/上报当前状态 */

/* Private variables ---------------------------------------------------------*/
static uint8_t s_frame[VOICE_FRAME_LEN];
static uint8_t s_frame_idx = 0U;

/* Private function prototypes -----------------------------------------------*/
static void Voice_FeedByte(uint8_t byte);
static void Voice_Dispatch(uint8_t cmd, uint8_t value);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  逐个字节喂入解析器
  * @note   用"逐字节收 + 状态机"的方式:
  *           1) 先对齐帧头 0xAA 0x55
  *           2) 收满 5 字节后校验 XOR
  *           3) 校验失败就把缓冲区移位重新找帧头, 不直接丢弃后续字节
  */
static void Voice_FeedByte(uint8_t byte)
{
    uint8_t i;
    uint8_t xorv;

    if (s_frame_idx == 0U)
    {
        /* 等第一个帧头 */
        if (byte == VOICE_HEAD0)
        {
            s_frame[0] = byte;
            s_frame_idx = 1U;
        }
        return;
    }

    if (s_frame_idx == 1U)
    {
        if (byte == VOICE_HEAD1)
        {
            s_frame[1] = byte;
            s_frame_idx = 2U;
        }
        else if (byte == VOICE_HEAD0)
        {
            /* 连续两个 0xAA, 保持等待第二个帧头 */
            s_frame_idx = 1U;
        }
        else
        {
            s_frame_idx = 0U;
        }
        return;
    }

    /* 收 CMD / VALUE / XOR */
    s_frame[s_frame_idx] = byte;
    s_frame_idx++;

    if (s_frame_idx < VOICE_FRAME_LEN)
    {
        return;
    }

    /* ---- 一帧收满, 校验 ---- */
    s_frame_idx = 0U;

    xorv = (uint8_t)(s_frame[0] ^ s_frame[1] ^ s_frame[2] ^ s_frame[3]);
    if (xorv != s_frame[4])
    {
        return;     /* 校验失败, 丢弃这一帧 */
    }

    Voice_Dispatch(s_frame[2], s_frame[3]);

    (void)i;
}

/** @brief 命令分发 */
static void Voice_Dispatch(uint8_t cmd, uint8_t value)
{
    switch (cmd)
    {
    case VC_FAN_ON:
        (void)Cmd_Post((uint8_t)CMD_FAN_ON, 0U, (uint8_t)SRC_VOICE);
        break;

    case VC_FAN_OFF:
        (void)Cmd_Post((uint8_t)CMD_FAN_OFF, 0U, (uint8_t)SRC_VOICE);
        break;

    case VC_MODE_AUTO:
        (void)Cmd_Post((uint8_t)CMD_MODE_SET, (uint16_t)MODE_AUTO, (uint8_t)SRC_VOICE);
        break;

    case VC_MODE_MANUAL:
        (void)Cmd_Post((uint8_t)CMD_MODE_SET, (uint16_t)MODE_MANUAL, (uint8_t)SRC_VOICE);
        break;

    case VC_MODE_TOGGLE:
        (void)Cmd_Post((uint8_t)CMD_MODE_TOGGLE, 0U, (uint8_t)SRC_VOICE);
        break;

    case VC_LEVEL_UP:
        (void)Cmd_Post((uint8_t)CMD_LEVEL_UP, 0U, (uint8_t)SRC_VOICE);
        break;

    case VC_LEVEL_DOWN:
        (void)Cmd_Post((uint8_t)CMD_LEVEL_DOWN, 0U, (uint8_t)SRC_VOICE);
        break;

    case VC_LEVEL_SET:
        (void)Cmd_Post((uint8_t)CMD_LEVEL_SET, (uint16_t)value, (uint8_t)SRC_VOICE);
        break;

    case VC_TEMP_SET:
        (void)Cmd_Post((uint8_t)CMD_TEMP_SET, (uint16_t)value, (uint8_t)SRC_VOICE);
        break;

    case VC_TEMP_UP:
    {
        /* 语音无法直接发"当前阈值+1", 因此这里用特殊约定:
           VALUE 为 0 时表示 +1, 为 1 时表示 -1(由 SU-03T 词条决定) */
        uint16_t th = (uint16_t)g_state.cfg.temp_threshold + 1U;
        (void)Cmd_Post((uint8_t)CMD_TEMP_SET, th, (uint8_t)SRC_VOICE);
        break;
    }

    case VC_TEMP_DOWN:
    {
        uint16_t th = (g_state.cfg.temp_threshold > 0U) ?
                      (uint16_t)(g_state.cfg.temp_threshold - 1U) : 0U;
        (void)Cmd_Post((uint8_t)CMD_TEMP_SET, th, (uint8_t)SRC_VOICE);
        break;
    }

    case VC_TIME_SET:
        /* VALUE 单位为"分钟", 换算成秒 */
        (void)Cmd_Post((uint8_t)CMD_COUNTDOWN_SET,
                       (uint16_t)((uint16_t)value * 60U),
                       (uint8_t)SRC_VOICE);
        break;

    case VC_TIME_CANCEL:
        (void)Cmd_Post((uint8_t)CMD_COUNTDOWN_CANCEL, 0U, (uint8_t)SRC_VOICE);
        break;

    case VC_SWING_ON:
        (void)Cmd_Post((uint8_t)CMD_SWING_SET, (uint16_t)SWING_ON, (uint8_t)SRC_VOICE);
        break;

    case VC_SWING_OFF:
        (void)Cmd_Post((uint8_t)CMD_SWING_SET, (uint16_t)SWING_OFF, (uint8_t)SRC_VOICE);
        break;

    case VC_SWING_TOGGLE:
        (void)Cmd_Post((uint8_t)CMD_SWING_TOGGLE, 0U, (uint8_t)SRC_VOICE);
        break;

    case VC_QUERY_STATUS:
        (void)Cmd_Post((uint8_t)CMD_QUERY_STATUS, 0U, (uint8_t)SRC_VOICE);
        break;

    default:
        break;
    }
}

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化 */
void AppVoice_Init(void)
{
    s_frame_idx = 0U;
    BspUart_Flush(UART_ID_VOICE);
}

/** @brief 任务主循环 */
void AppVoice_Task(void *argument)
{
    uint8_t byte;

    (void)argument;

    for (;;)
    {
        while (BspUart_GetByte(UART_ID_VOICE, &byte) != 0U)
        {
            Voice_FeedByte(byte);
        }

        osDelay(VOICE_TASK_TICK_MS);
    }
}
