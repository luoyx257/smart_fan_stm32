/**
  ******************************************************************************
  * @file    bsp_sensor.c
  * @brief   人体红外与压力传感器驱动实现
  * @note    压力传感器模块通常有 3 个电位器/输出:
  *            DO - 数字量, 超过电位器设定阈值输出有效电平(本项目作主判据)
  *            AO - 模拟量 0~3.3V, 反映压力大小(本项目只做 OLED 显示)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_sensor.h"
#include "main.h"
#include "adc.h"

/* Private define ------------------------------------------------------------*/
#define IR_PORT             GPIOA
#define IR_PIN              GPIO_PIN_0

#define PRESSURE_DO_PORT    GPIOA
#define PRESSURE_DO_PIN     GPIO_PIN_5

#define ADC_TIMEOUT_MS      20U

/* Private variables ---------------------------------------------------------*/
static uint8_t  s_adc_started = 0U;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化传感器 GPIO
  * @note   重新把 PA0 / PA5 配置为"上拉输入"。CubeMX 生成的是 NOPULL,
  *          HC-SR501 与压力模块都是推挽/开漏输出, 加上拉无害且能防悬空。
  */
void Sensor_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = IR_PIN;
    HAL_GPIO_Init(IR_PORT, &gpio);

    gpio.Pin = PRESSURE_DO_PIN;
    HAL_GPIO_Init(PRESSURE_DO_PORT, &gpio);

    s_adc_started = 0U;
}

/** @brief 人体红外状态 */
uint8_t Sensor_InfraredDetected(void)
{
    /* HC-SR501: 检测到人输出高电平 */
    return (HAL_GPIO_ReadPin(IR_PORT, IR_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

/** @brief 压力 DO 状态 */
uint8_t Sensor_PressureDoActive(void)
{
    /* 压力模块 DO 一般为"超过阈值输出高电平";
       若你的模块是低电平有效, 把这里的判断反过来即可 */
    return (HAL_GPIO_ReadPin(PRESSURE_DO_PORT, PRESSURE_DO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

/**
  * @brief  读取 PA4(ADC1_IN4) 原始值
  * @note   使用 1 次转换的软件触发模式, 每次重新 Start/Stop。
  *         ADC 时钟 = PCLK2/6 = 12MHz, 采样时间 1.5 周期。
  */
uint16_t Sensor_PressureRawAdc(void)
{
    uint16_t value = 0U;

    if (s_adc_started == 0U)
    {
        /* ADC 校准只需做一次 */
        if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
        {
            return 0U;
        }
        s_adc_started = 1U;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) == HAL_OK)
    {
        value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    (void)HAL_ADC_Stop(&hadc1);
    return value;
}
