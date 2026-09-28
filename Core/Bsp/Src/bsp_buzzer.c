/**
  ******************************************************************************
  * @file    bsp_buzzer.c
  * @brief   有源蜂鸣器实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_buzzer.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define BUZZER_PORT     GPIOB
#define BUZZER_PIN      GPIO_PIN_8

/* Private variables ---------------------------------------------------------*/
static uint8_t s_on = 0U;

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化, 保证静音 */
void Buzzer_Init(void)
{
    Buzzer_Off();
}

/** @brief 响 */
void Buzzer_On(void)
{
    /* MH-FMD 低电平触发 */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    s_on = 1U;
}

/** @brief 停 */
void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
    s_on = 0U;
}

/** @brief 按参数设置 */
void Buzzer_Set(uint8_t on)
{
    if (on != 0U)
    {
        Buzzer_On();
    }
    else
    {
        Buzzer_Off();
    }
}

/** @brief 当前状态 */
uint8_t Buzzer_IsOn(void)
{
    return s_on;
}
