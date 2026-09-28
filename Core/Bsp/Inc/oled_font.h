/**
  ******************************************************************************
  * @file    oled_font.h
  * @brief   8x16 ASCII 点阵字库声明
  ******************************************************************************
  */

#ifndef __OLED_FONT_H
#define __OLED_FONT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define OLED_FONT_FIRST_CHAR        0x20U   /* 字库第一个字符: 空格 */
#define OLED_FONT_LAST_CHAR         0x7EU   /* 字库最后一个字符: ~   */
#define OLED_FONT_CHARS             (OLED_FONT_LAST_CHAR - OLED_FONT_FIRST_CHAR + 1U)

#define OLED_FONT_WIDTH             8U      /* 字符宽 8 像素 */
#define OLED_FONT_HEIGHT            16U     /* 字符高 16 像素 */
/* 每个字符占 2 个页(8+8 行), 每页 8 列 -> 16 字节/页 -> 共 32 字节 */
#define OLED_FONT_BYTES_PER_PAGE    8U
#define OLED_FONT_BYTES_PER_CHAR    32U

/** 字库: [95][32], 排列方式与 SSD1306 页显存一致 */
extern const uint8_t OLED_Font8x16[OLED_FONT_CHARS][OLED_FONT_BYTES_PER_CHAR];

#ifdef __cplusplus
}
#endif

#endif /* __OLED_FONT_H */
