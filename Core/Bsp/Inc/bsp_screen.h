/**
  ******************************************************************************
  * @file    bsp_screen.h
  * @brief   串口屏 TJC4827X243_011C(USART2, 115200) 底层收发
  ******************************************************************************
  */

#ifndef __BSP_SCREEN_H
#define __BSP_SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 初始化(仅清一次接收缓冲) */
void Screen_Init(void);

/** 发送字符串 */
void Screen_Send(const char *str);

/** 发送任意数据 */
void Screen_SendBuf(const uint8_t *data, uint16_t len);

/** 以 "xx.val=数字" 形式写屏 */
void Screen_SetNumber(const char *obj, int32_t value);

/** 以 "xx.txt=\"字符串\"" 形式写屏, 字符串内不能含引号 */
void Screen_SetText(const char *obj, const char *text);

/**
  * @brief  发送结束符(每批指令末尾发 3 个 0xFF)
  * @note   TJC/Nextion 协议用 0xFF 0xFF 0xFF 作为指令结束标志, 少数老固件
  *         会忽略尾部结束符, 但补上更稳妥。
  */
void Screen_SendEnd(void);

/** 把屏幕切到指定页面 */
void Screen_GotoPage(uint8_t page_id);

/**
  * @brief  结束一次"批量写屏"
  * @note   调用示例:
  *           Screen_SetNumber("t0", 25);
  *           Screen_SetText("t1", "25");
  *           Screen_Commit();
  */
void Screen_Commit(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SCREEN_H */
