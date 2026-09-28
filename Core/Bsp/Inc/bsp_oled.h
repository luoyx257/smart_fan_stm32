/**
  ******************************************************************************
  * @file    bsp_oled.h
  * @brief   0.96" SSD1306 OLED 驱动(I2C1: PB6=SCL, PB7=SDA)
  * @note    采用"页式局部刷新", 不做 1024 字节全屏显存, 只为 DMA/内存考虑:
  *          STM32F103C8T6 只有 20KB SRAM, 省下 1KB 很值。
  *          代价是每行必须整行重画, 因此本驱动提供"先清行再写"的行接口。
  ******************************************************************************
  */

#ifndef __BSP_OLED_H
#define __BSP_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "oled_font.h"

#define OLED_WIDTH      128U
#define OLED_HEIGHT     64U
#define OLED_PAGES      (OLED_HEIGHT / 8U)      /* 8 页 */

/* 常用 SSD1306 I2C 地址（7 位） */
#define OLED_I2C_ADDR_0X3C      0x3CU
#define OLED_I2C_ADDR_0X3D      0x3DU

/**
  * @brief  初始化 OLED
  * @param  addr7  7 位从机地址, 通常 0x3C 或 0x3D
  * @retval 1 = 初始化成功(收到了 ACK), 0 = 失败
  */
uint8_t OLED_Init(uint8_t addr7);

/**
  * @brief  探测 I2C 上是否存在该地址的器件
  * @retval 1 = 有应答
  */
uint8_t OLED_Probe(uint8_t addr7);

/** 清屏(全黑) */
void OLED_Clear(void);

/** 全屏点亮(自检用) */
void OLED_Fill(void);

/**
  * @brief  在指定像素坐标画一个 8x16 字符
  * @param  x     列 0~120
  * @param  page  页 0~3 (每页 16 像素高: 页0=行0~15, 页1=行16~31...)
  * @param  ch    ASCII 字符
  */
void OLED_DrawChar(uint8_t x, uint8_t page, char ch);

/**
  * @brief  在指定页写一个字符串(不自动换行, 超出部分截断)
  * @param  x     起始列 0~127
  * @param  page  页 0~3
  * @param  str   ASCII 字符串
  */
void OLED_DrawString(uint8_t x, uint8_t page, const char *str);

/**
  * @brief  在页上居中写一个字符串
  * @param  page 页 0~3
  * @param  str  ASCII 字符串
  */
void OLED_DrawStringCenter(uint8_t page, const char *str);

/**
  * @brief  在页上写一个无符号整数(十进制)
  * @param  x      起始列
  * @param  page   页
  * @param  value  数值
  * @param  width  固定宽度, 右对齐补空格; 0 表示不补
  * @param  pad_zero 1 = 用 '0' 补位, 0 = 用空格补位
  * @return 写完后的下一列位置
  */
uint8_t OLED_DrawNumber(uint8_t x, uint8_t page, uint32_t value,
                        uint8_t width, uint8_t pad_zero);

/**
  * @brief  画一条水平线(用于分隔行)
  * @param  x0 起始列, x1 结束列(含)
  * @param  y  行坐标 0~63
  */
void OLED_HLine(uint8_t x0, uint8_t x1, uint8_t y);

/**
  * @brief  画一个空心矩形边框(用于做进度条/边框)
  */
void OLED_Rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_OLED_H */
