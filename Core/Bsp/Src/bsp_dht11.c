/**
  ******************************************************************************
  * @file    bsp_dht11.c
  * @brief   DHT11 温湿度传感器驱动实现(单总线, PA1)
  * @note    ★ 关于时序的关键说明 ★
  *          DHT11 是纯时序器件: 主机的起始信号需要拉低 >=18ms, 之后传感器的
  *          应答脉冲只有 80us, 每个数据位的高电平只有 27us / 70us。
  *          因此"发送起始信号"和"读取应答"必须在**同一次连续调用**里完成,
  *          绝不能被 RTOS 的 tick 切开(否则等下一次调度进来时, 应答早就过去了,
  *          必然读不到数据 —— 这是 DHT11 驱动最常见的坑)。
  *
  *          本驱动的处理办法:
  *            DHT11_Process() 内部做完整的 23ms 读时序(18ms 起始 + 5ms 读 40bit),
  *            只在"距上次读取 >=2s"时才真正执行, 其余时间几乎不耗时。
  *            读 40bit 那 4ms 内关中断, 保证 27/70us 的判定不出错。
  *          调用代价: 每 2 秒阻塞约 23ms。任务优先级安排上已考虑该占用。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_dht11.h"
#include "bsp_delay.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define DHT11_GPIO_PORT         GPIOA
#define DHT11_GPIO_PIN          GPIO_PIN_1

#define DHT11_MIN_INTERVAL_MS   2000U   /* DHT11 两次读取至少间隔 1s, 这里取 2s */
#define DHT11_START_LOW_MS      18U     /* 起始信号拉低 18ms */
#define DHT11_START_HIGH_US     30U     /* 释放后保持高电平 20~40us */

#define DHT11_TIMEOUT_US        150U    /* 单步电平等待超时(应答 80us 留足余量) */
#define DHT11_BIT_TIMEOUT_US    200U    /* 单个 bit 高电平超时 */

#define DHT11_BIT_ONE_THRESHOLD 45U     /* 高电平 > 45us 判为 1 (0≈27us, 1≈70us) */

/* Private variables ---------------------------------------------------------*/
static uint8_t  s_temp;                 /* 解析后的温度 */
static uint8_t  s_humi;                 /* 解析后的湿度 */
static uint8_t  s_data_ready;           /* 1 = 有新数据待取走 */
static uint8_t  s_ok;                   /* 最近一次是否成功 */
static uint32_t s_last_tick_ms;         /* 上次读取结束时刻(ms) */
static uint32_t s_done_count;           /* 读取完成次数(无论成败) */

/* Private function prototypes -----------------------------------------------*/
static void     DHT_PinOutput(void);
static void     DHT_PinInput(void);
static uint8_t  DHT_ReadByte(uint8_t *ok);
static uint8_t  DHT_WaitLevel(GPIO_PinState level, uint32_t timeout_us);

/* Private functions ---------------------------------------------------------*/

/** @brief 把 PA1 切成推挽输出(发送起始信号用) */
static void DHT_PinOutput(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = DHT11_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &gpio);
}

/** @brief 把 PA1 切回上拉输入(读取用)。
  *        DHT11 模块自带上拉电阻, 这里再开内部上拉可提高抗干扰能力 */
static void DHT_PinInput(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin  = DHT11_GPIO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &gpio);
}

/**
  * @brief  等待引脚变成指定电平
  * @param  level      期望电平
  * @param  timeout_us 超时
  * @retval 1 = 已到达该电平, 0 = 超时
  */
static uint8_t DHT_WaitLevel(GPIO_PinState level, uint32_t timeout_us)
{
    uint32_t t0 = BSP_Micros();

    while (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) != level)
    {
        if ((uint32_t)(BSP_Micros() - t0) > timeout_us)
        {
            return 0U;
        }
    }
    return 1U;
}

/** @brief 读取 1 个字节(8bit, MSB 先出), 调用时已关中断
  *  @param ok 输出: 1 = 正常读完, 0 = 超时失败(此时返回值无意义)  */
static uint8_t DHT_ReadByte(uint8_t *ok)
{
    uint8_t i;
    uint8_t byte = 0U;
    uint32_t t0;
    uint32_t high_us;

    *ok = 1U;

    for (i = 0U; i < 8U; i++)
    {
        byte >>= 1;     /* 先右移, 因为下面用 |= 0x80 填高位 */

        /* --- 等待 50us 低电平结束(上升沿) --- */
        if (DHT_WaitLevel(GPIO_PIN_SET, DHT11_BIT_TIMEOUT_US) == 0U)
        {
            *ok = 0U;
            return 0U;
        }

        /* --- 测量高电平持续时间 --- */
        t0 = BSP_Micros();
        if (DHT_WaitLevel(GPIO_PIN_RESET, DHT11_BIT_TIMEOUT_US) == 0U)
        {
            *ok = 0U;
            return 0U;
        }
        high_us = (uint32_t)(BSP_Micros() - t0);

        if (high_us > DHT11_BIT_ONE_THRESHOLD)
        {
            byte |= 0x80U;
        }
    }
    return byte;
}

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化 PA1 为空闲态 */
void DHT11_Init(void)
{
    s_temp         = 0U;
    s_humi         = 0U;
    s_data_ready   = 0U;
    s_ok           = 0U;
    s_last_tick_ms = 0U;
    s_done_count   = 0U;

    DHT_PinInput();     /* 空闲时保持输入, 由外部上拉拉高 */
}

/** @brief 最近一次读取是否成功 */
uint8_t DHT11_IsOk(void)
{
    return s_ok;
}

/** @brief 读取完成次数 */
uint32_t DHT11_GetDoneCount(void)
{
    return s_done_count;
}

/** @brief 取回数据 */
uint8_t DHT11_Fetch(uint8_t *temp, uint8_t *humi)
{
    if (s_data_ready == 0U)
    {
        return 0U;
    }

    if (temp != NULL)
    {
        *temp = s_temp;
    }
    if (humi != NULL)
    {
        *humi = s_humi;
    }
    s_data_ready = 0U;
    return 1U;
}

/**
  * @brief  执行一次完整的 DHT11 读取(阻塞约 23ms)
  * @retval 1 = 本次真的读了并成功, 0 = 未到间隔/读取失败
  */
uint8_t DHT11_Read(void)
{
    uint8_t  raw[5];
    uint8_t  i;
    uint8_t  byte_ok;
    uint8_t  all_ok;
    uint8_t  checksum;

    /* ---- 间隔保护: 距上次读取不足 2s 直接返回 ---- */
    if (s_last_tick_ms != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - s_last_tick_ms) < DHT11_MIN_INTERVAL_MS)
        {
            return 0U;
        }
    }

    /* ---- 1) 起始信号: 拉低 18ms ---- */
    DHT_PinOutput();
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET);
    HAL_Delay(DHT11_START_LOW_MS);

    /* ---- 2) 释放总线, 保持高 30us, 切输入 ---- */
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_SET);
    DHT_PinInput();
    BSP_DelayUs(DHT11_START_HIGH_US);

    /* ---- 3) 等传感器应答: 先低 80us, 再高 80us ----
       注意: 这三步必须紧跟起始信号, 不能有任何 tick 间隔 */
    s_ok = 0U;

    if (DHT_WaitLevel(GPIO_PIN_RESET, DHT11_TIMEOUT_US) == 0U)
    {
        s_last_tick_ms = HAL_GetTick();
        s_done_count++;
        return 0U;
    }
    if (DHT_WaitLevel(GPIO_PIN_SET, DHT11_TIMEOUT_US) == 0U)
    {
        s_last_tick_ms = HAL_GetTick();
        s_done_count++;
        return 0U;
    }
    if (DHT_WaitLevel(GPIO_PIN_RESET, DHT11_TIMEOUT_US) == 0U)
    {
        s_last_tick_ms = HAL_GetTick();
        s_done_count++;
        return 0U;
    }

    /* ---- 4) 收 40bit: 约 4ms 内关中断保证时序 ---- */
    __disable_irq();

    all_ok = 1U;
    for (i = 0U; i < 5U; i++)
    {
        raw[i] = DHT_ReadByte(&byte_ok);
        if (byte_ok == 0U)
        {
            all_ok = 0U;
            break;
        }
    }

    __enable_irq();

    s_last_tick_ms = HAL_GetTick();
    s_done_count++;

    if (all_ok == 0U)
    {
        return 0U;
    }

    /* ---- 5) 校验: 前 4 字节之和的低 8 位 == 第 5 字节 ---- */
    checksum = (uint8_t)(raw[0] + raw[1] + raw[2] + raw[3]);
    if (checksum != raw[4])
    {
        return 0U;
    }

    s_humi       = raw[0];      /* 湿度整数部分 */
    s_temp       = raw[2];      /* 温度整数部分 */
    s_ok         = 1U;
    s_data_ready = 1U;
    return 1U;
}

/**
  * @brief  周期调用入口
  * @note   由 Task_Sensor 每 20ms 调一次; 内部会自己做 2s 间隔控制,
  *         因此调用频率高一些也只会带来极小的判断开销。
  *         不做任何跨调用状态机 —— 时序完整性优先。
  */
void DHT11_Process(void)
{
    (void)DHT11_Read();
}

/** @brief 当前是否空闲(本驱动为阻塞式, 恒为 1, 保留接口兼容性) */
uint8_t DHT11_IsIdle(void)
{
    return 1U;
}

/* 占位: 保持与原接口一致, 便于上层不改代码 */
uint8_t DHT11_Start(void)
{
    return DHT11_Read();
}
