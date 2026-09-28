/**
  ******************************************************************************
  * @file    bsp_fan.c
  * @brief   TB6612 风扇调速实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_fan.h"
#include "main.h"
#include "tim.h"

/* Private define ------------------------------------------------------------*/
#define AIN1_PORT       GPIOB
#define AIN1_PIN        GPIO_PIN_0
#define AIN2_PORT       GPIOB
#define AIN2_PIN        GPIO_PIN_1

/* TIM3 定时器时钟 72MHz, ARR = 3599 -> PWM 频率 20kHz */
#define FAN_PWM_PERIOD  3599U

/* Private variables ---------------------------------------------------------*/
static uint8_t s_reverse = 0U;

/* Private function prototypes -----------------------------------------------*/
static void Fan_ApplyDirection(uint16_t duty_permille);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  根据当前方向和占空比设置 AIN1/AIN2
  * @note   占空比为 0 时直接让 AIN1=AIN2=0 滑行, 避免电机在 0 占空比下
  *         仍被驱动器加电压产生微小发热
  */
static void Fan_ApplyDirection(uint16_t duty_permille)
{
    if (duty_permille == 0U)
    {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
        return;
    }

    if (s_reverse == 0U)
    {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);      /* AIN1 = 1 */
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);    /* AIN2 = 0 -> 正转 */
    }
    else
    {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);      /* 反转 */
    }
}

/* Exported functions --------------------------------------------------------*/

/** @brief 初始化风扇通道 */
void Fan_Init(void)
{
    s_reverse = 0U;

    /* 先把两个方向脚拉低, 保证上电不转 */
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);

    /* 启动 TIM3_CH2 PWM, 比较值先清零 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

/** @brief 设置转速 */
void Fan_SetDuty(uint16_t duty_permille)
{
    uint32_t compare;

    if (duty_permille > 1000U)
    {
        duty_permille = 1000U;      /* 限幅 */
    }

    /* 千分比 -> 比较值: compare = duty * (ARR+1) / 1000 */
    compare = ((uint32_t)duty_permille * (FAN_PWM_PERIOD + 1U)) / 1000U;
    if (compare > FAN_PWM_PERIOD)
    {
        compare = FAN_PWM_PERIOD;
    }

    Fan_ApplyDirection(duty_permille);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
}

/** @brief 停止(滑行) */
void Fan_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
}

/** @brief 刹车 */
void Fan_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
}

/** @brief 设置转向 */
void Fan_SetReverse(uint8_t reverse)
{
    s_reverse = (reverse != 0U) ? 1U : 0U;
}
