/**
  ******************************************************************************
  * @file    bsp_oled.c
  * @brief   SSD1306 OLED 驱动实现(页式局部刷新, 不用全屏显存)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_oled.h"
#include "i2c.h"

/* Private define ------------------------------------------------------------*/
#define OLED_I2C_HANDLE         (&hi2c1)
#define OLED_I2C_TIMEOUT_MS     50U     /* 一次传输超时; 单帧 129 字节 @100kHz 约 12ms */

/* SSD1306 控制字节 */
#define OLED_CTRL_CMD           0x00U   /* 后续为命令 */
#define OLED_CTRL_DATA          0x40U   /* 后续为显存数据 */

/* I2C 故障计数达到该值后重新初始化 I2C 外设, 尝试从总线锁死中恢复 */
#define OLED_I2C_ERR_LIMIT      5U

/* Private variables ---------------------------------------------------------*/
static uint8_t s_addr8 = 0x78U;         /* 8 位地址(7 位左移 1 位) */
static uint8_t s_ready  = 0U;           /* 1 = 初始化过 */
static uint8_t s_i2c_err = 0U;

/* 行缓冲: 1 页 = 128 列 x 8 行 = 128 字节。只在刷某一行时占用, 传输完即失效。 */
static uint8_t s_rowbuf[OLED_WIDTH];

/* Private function prototypes -----------------------------------------------*/
static uint8_t OLED_WriteCmd(uint8_t cmd);
static uint8_t OLED_WriteData(const uint8_t *data, uint16_t len);
static void    OLED_RecoverI2c(void);
static uint8_t OLED_SetPage(uint8_t page);
static void    OLED_FlushRow(uint8_t page);
static uint8_t OLED_ReadRow(uint8_t page);
static void    OLED_FlushChar(uint8_t x, uint8_t page, const uint8_t *bitmap);

/* Private functions ---------------------------------------------------------*/

/** @brief 发送一条命令 */
static uint8_t OLED_WriteCmd(uint8_t cmd)
{
    if (HAL_I2C_Mem_Write(OLED_I2C_HANDLE, (uint16_t)s_addr8, OLED_CTRL_CMD,
                          I2C_MEMADD_SIZE_8BIT, &cmd, 1U, OLED_I2C_TIMEOUT_MS) != HAL_OK)
    {
        s_i2c_err++;
        OLED_RecoverI2c();
        return 0U;
    }
    s_i2c_err = 0U;
    return 1U;
}

/** @brief 发送显存数据 */
static uint8_t OLED_WriteData(const uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Write(OLED_I2C_HANDLE, (uint16_t)s_addr8, OLED_CTRL_DATA,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len,
                          OLED_I2C_TIMEOUT_MS) != HAL_OK)
    {
        s_i2c_err++;
        OLED_RecoverI2c();
        return 0U;
    }
    s_i2c_err = 0U;
    return 1U;
}

/**
  * @brief  I2C 连续出错时重新初始化外设
  * @note   SSD1306 在电源异常或线松时可能拉住 SCL 不放, HAL 会一直返回 TIMEOUT,
  *         重新 DeInit/Init 能把外设状态机复位(若从机仍拉死总线则无法恢复,
  *         那种情况只能硬件断电, 这里只做尽力恢复, 避免整个任务卡死)。
  */
static void OLED_RecoverI2c(void)
{
    if (s_i2c_err >= OLED_I2C_ERR_LIMIT)
    {
        (void)HAL_I2C_DeInit(OLED_I2C_HANDLE);
        (void)HAL_I2C_Init(OLED_I2C_HANDLE);
        s_i2c_err = 0U;
    }
}

/** @brief 设置写入起始页与列 0 */
static uint8_t OLED_SetPage(uint8_t page)
{
    if (page >= OLED_PAGES)
    {
        return 0U;
    }
    if (OLED_WriteCmd((uint8_t)(0xB0U | page)) == 0U)       /* 页地址 */
    {
        return 0U;
    }
    if (OLED_WriteCmd(0x00U) == 0U)                          /* 低 4 位列地址 = 0 */
    {
        return 0U;
    }
    if (OLED_WriteCmd(0x10U) == 0U)                          /* 高 4 位列地址 = 0 */
    {
        return 0U;
    }
    return 1U;
}

/** @brief 把 s_rowbuf 整页刷到 OLED */
static void OLED_FlushRow(uint8_t page)
{
    if (OLED_SetPage(page) == 0U)
    {
        return;
    }
    (void)OLED_WriteData(s_rowbuf, OLED_WIDTH);
}

/**
  * @brief  从 OLED 读回一页到 s_rowbuf
  * @retval 1 = 读成功
  * @note   只给画线/画框用(需要读-改-写)。SSD1306 支持 I2C 读。
  */
static uint8_t OLED_ReadRow(uint8_t page)
{
    if (OLED_SetPage(page) == 0U)
    {
        return 0U;
    }
    if (HAL_I2C_Mem_Read(OLED_I2C_HANDLE, (uint16_t)s_addr8, OLED_CTRL_DATA,
                         I2C_MEMADD_SIZE_8BIT, s_rowbuf, OLED_WIDTH,
                         OLED_I2C_TIMEOUT_MS) != HAL_OK)
    {
        s_i2c_err++;
        OLED_RecoverI2c();
        return 0U;
    }
    s_i2c_err = 0U;
    return 1U;
}

/**
  * @brief  把一个 8x16 字符的 32 字节点阵贴进指定位置
  * @note   字符跨两页: 上半 16 字节进 page, 下半 16 字节进 page+1。
  *         先读回整行, 改 8 列, 再整行写回。
  */
static void OLED_FlushChar(uint8_t x, uint8_t page, const uint8_t *bitmap)
{
    uint8_t i;
    uint8_t col;

    if ((page + 1U) >= OLED_PAGES)
    {
        return;             /* 放不下一个完整字符就放弃 */
    }

    /* ---- 上半页 ---- */
    if (OLED_ReadRow(page) != 0U)
    {
        for (i = 0U; i < OLED_FONT_BYTES_PER_PAGE; i++)
        {
            col = (uint8_t)(x + i);
            if (col < OLED_WIDTH)
            {
                s_rowbuf[col] = bitmap[i];
            }
        }
        OLED_FlushRow(page);
    }

    /* ---- 下半页 ---- */
    if (OLED_ReadRow((uint8_t)(page + 1U)) != 0U)
    {
        for (i = 0U; i < OLED_FONT_BYTES_PER_PAGE; i++)
        {
            col = (uint8_t)(x + i);
            if (col < OLED_WIDTH)
            {
                s_rowbuf[col] = bitmap[OLED_FONT_BYTES_PER_PAGE + i];
            }
        }
        OLED_FlushRow((uint8_t)(page + 1U));
    }
}

/* Exported functions --------------------------------------------------------*/

/** @brief 探测器件是否存在 */
uint8_t OLED_Probe(uint8_t addr7)
{
    uint8_t addr8 = (uint8_t)(addr7 << 1);

    if (HAL_I2C_IsDeviceReady(OLED_I2C_HANDLE, (uint16_t)addr8, 2U,
                              OLED_I2C_TIMEOUT_MS) == HAL_OK)
    {
        return 1U;
    }
    return 0U;
}

/**
  * @brief  SSD1306 初始化
  * @note   0.96" 128x64 模块常见初始化序列
  */
uint8_t OLED_Init(uint8_t addr7)
{
    static const uint8_t init_seq[] = {
        0xAEU,          /* 关闭显示 */
        0x20U, 0x02U,   /* 页寻址模式 */
        0xB0U,          /* 页起始地址 0 */
        0xC8U,          /* COM 扫描方向: 反序(上下翻转校正) */
        0x00U,          /* 低列地址 */
        0x10U,          /* 高列地址 */
        0x40U,          /* 显示起始行 0 */
        0x81U, 0xCFU,   /* 对比度 0xCF */
        0xA1U,          /* 段重映射: 左右翻转校正 */
        0xA6U,          /* 正常显示(非反色) */
        0xA8U, 0x3FU,   /* 多路复用比 64 */
        0xA4U,          /* 输出跟随显存 */
        0xD3U, 0x00U,   /* 显示偏移 0 */
        0xD5U, 0x80U,   /* 时钟分频 */
        0xD9U, 0xF1U,   /* 预充电周期 */
        0xDAU, 0x12U,   /* COM 引脚配置 */
        0xDBU, 0x40U,   /* VCOMH 电压 */
        0x8DU, 0x14U,   /* 使能电荷泵 */
        0xAFU           /* 打开显示 */
    };
    uint8_t i;

    s_addr8 = (uint8_t)(addr7 << 1);
    s_i2c_err = 0U;

    /* 先探测, 没应答就直接返回失败, 避免后面刷屏全部超时把任务拖慢 */
    if (OLED_Probe(addr7) == 0U)
    {
        s_ready = 0U;
        return 0U;
    }

    for (i = 0U; i < (uint8_t)(sizeof(init_seq) / sizeof(init_seq[0])); i++)
    {
        if (OLED_WriteCmd(init_seq[i]) == 0U)
        {
            s_ready = 0U;
            return 0U;
        }
    }

    s_ready = 1U;
    OLED_Clear();
    return 1U;
}

/** @brief 清屏 */
void OLED_Clear(void)
{
    uint16_t i;
    uint8_t  page;

    if (s_ready == 0U)
    {
        return;
    }

    for (i = 0U; i < OLED_WIDTH; i++)
    {
        s_rowbuf[i] = 0x00U;
    }

    for (page = 0U; page < OLED_PAGES; page++)
    {
        OLED_FlushRow(page);
    }
}

/** @brief 全屏点亮(接线自检用) */
void OLED_Fill(void)
{
    uint16_t i;
    uint8_t  page;

    if (s_ready == 0U)
    {
        return;
    }

    for (i = 0U; i < OLED_WIDTH; i++)
    {
        s_rowbuf[i] = 0xFFU;
    }

    for (page = 0U; page < OLED_PAGES; page++)
    {
        OLED_FlushRow(page);
    }
}

/** @brief 画字符 */
void OLED_DrawChar(uint8_t x, uint8_t page, char ch)
{
    uint8_t code;
    const uint8_t *bitmap;

    if (s_ready == 0U)
    {
        return;
    }
    if ((page + 1U) >= OLED_PAGES)
    {
        return;
    }
    if ((uint8_t)ch > (uint8_t)0x7F)        /* 暂时只支持 ASCII */
    {
        ch = '?';
    }

    code = (uint8_t)ch;
    if (code < OLED_FONT_FIRST_CHAR)
    {
        code = (uint8_t)' ';
    }
    if (code > OLED_FONT_LAST_CHAR)
    {
        code = (uint8_t)'?';
    }

    bitmap = &OLED_Font8x16[code - OLED_FONT_FIRST_CHAR][0];
    OLED_FlushChar(x, page, bitmap);
}

/** @brief 写字符串 */
void OLED_DrawString(uint8_t x, uint8_t page, const char *str)
{
    uint8_t col = x;

    if (str == NULL)
    {
        return;
    }

    while ((*str != '\0') && (col + OLED_FONT_WIDTH) <= OLED_WIDTH)
    {
        OLED_DrawChar(col, page, *str);
        col = (uint8_t)(col + OLED_FONT_WIDTH);
        str++;
    }
}

/** @brief 居中写字符串 */
void OLED_DrawStringCenter(uint8_t page, const char *str)
{
    uint8_t len = 0U;
    uint8_t x;

    if (str == NULL)
    {
        return;
    }

    while ((str[len] != '\0') && (len < (OLED_WIDTH / OLED_FONT_WIDTH)))
    {
        len++;
    }

    x = (uint8_t)((OLED_WIDTH - (uint16_t)len * OLED_FONT_WIDTH) / 2U);
    OLED_DrawString(x, page, str);
}

/** @brief 写数字 */
uint8_t OLED_DrawNumber(uint8_t x, uint8_t page, uint32_t value,
                        uint8_t width, uint8_t pad_zero)
{
    char    tmp[11];        /* uint32 最大 4294967295 共 10 位 */
    uint8_t len = 0U;
    uint8_t i;
    uint8_t col = x;

    /* 反向取位 */
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

    /* 补齐宽度 */
    while ((width > 0U) && (len < width) && (len < sizeof(tmp)))
    {
        tmp[len++] = (pad_zero != 0U) ? '0' : ' ';
    }

    /* 反向输出 */
    for (i = 0U; i < len; i++)
    {
        char ch = tmp[len - 1U - i];
        if ((col + OLED_FONT_WIDTH) > OLED_WIDTH)
        {
            break;
        }
        OLED_DrawChar(col, page, ch);
        col = (uint8_t)(col + OLED_FONT_WIDTH);
    }

    return col;
}

/** @brief 画水平线 */
void OLED_HLine(uint8_t x0, uint8_t x1, uint8_t y)
{
    uint8_t page;
    uint8_t bit;
    uint8_t col;

    if ((s_ready == 0U) || (y >= OLED_HEIGHT) || (x0 > x1) || (x0 >= OLED_WIDTH))
    {
        return;
    }
    if (x1 >= OLED_WIDTH)
    {
        x1 = OLED_WIDTH - 1U;
    }

    page = (uint8_t)(y / 8U);
    bit  = (uint8_t)(y % 8U);

    if (OLED_ReadRow(page) == 0U)
    {
        return;
    }

    for (col = x0; col <= x1; col++)
    {
        s_rowbuf[col] |= (uint8_t)(1U << bit);
    }
    OLED_FlushRow(page);
}

/** @brief 画空心矩形 */
void OLED_Rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    /* 四条边: 上/下水平线, 左右两条竖线按页分段画 */
    uint8_t page;
    uint8_t bit;
    uint8_t col;

    if ((s_ready == 0U) || (x0 > x1) || (y0 > y1) || (x1 >= OLED_WIDTH) || (y1 >= OLED_HEIGHT))
    {
        return;
    }

    OLED_HLine(x0, x1, y0);
    OLED_HLine(x0, x1, y1);

    /* 竖线: 按页分段, 每页只读改写一次 */
    for (page = (uint8_t)(y0 / 8U); page <= (uint8_t)(y1 / 8U); page++)
    {
        uint8_t bit_lo = 0U;
        uint8_t bit_hi = 7U;
        uint8_t mask = 0U;

        if (page == (uint8_t)(y0 / 8U))
        {
            bit_lo = (uint8_t)(y0 % 8U);
        }
        if (page == (uint8_t)(y1 / 8U))
        {
            bit_hi = (uint8_t)(y1 % 8U);
        }

        for (bit = bit_lo; bit <= bit_hi; bit++)
        {
            mask |= (uint8_t)(1U << bit);
        }
        if (mask == 0U)
        {
            continue;
        }

        if (OLED_ReadRow(page) == 0U)
        {
            return;
        }
        s_rowbuf[x0] |= mask;
        s_rowbuf[x1] |= mask;
        OLED_FlushRow(page);
    }

    (void)col;
}
