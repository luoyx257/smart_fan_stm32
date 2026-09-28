/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_state.h"
#include "app_fan.h"
#include "app_sensor.h"
#include "app_oled.h"
#include "app_ui.h"
#include "app_bt.h"
#include "app_voice.h"
#include "bsp_delay.h"
#include "bsp_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_MotorFan */
osThreadId_t Task_MotorFanHandle;
const osThreadAttr_t Task_MotorFan_attributes = {
  .name = "Task_MotorFan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_Sensor */
osThreadId_t Task_SensorHandle;
const osThreadAttr_t Task_Sensor_attributes = {
  .name = "Task_Sensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_Uart_UI */
osThreadId_t Task_Uart_UIHandle;
const osThreadAttr_t Task_Uart_UI_attributes = {
  .name = "Task_Uart_UI",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_Bluetooth */
osThreadId_t Task_BluetoothHandle;
const osThreadAttr_t Task_Bluetooth_attributes = {
  .name = "Task_Bluetooth",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_Voice */
osThreadId_t Task_VoiceHandle;
const osThreadAttr_t Task_Voice_attributes = {
  .name = "Task_Voice",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_OLED */
osThreadId_t Task_OLEDHandle;
const osThreadAttr_t Task_OLED_attributes = {
  .name = "Task_OLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xQueueCmd */
osMessageQueueId_t xQueueCmdHandle;
const osMessageQueueAttr_t xQueueCmd_attributes = {
  .name = "xQueueCmd"
};
/* Definitions for xQueueSensorData */
osMessageQueueId_t xQueueSensorDataHandle;
const osMessageQueueAttr_t xQueueSensorData_attributes = {
  .name = "xQueueSensorData"
};
/* Definitions for oled_mutex */
osMutexId_t oled_mutexHandle;
const osMutexAttr_t oled_mutex_attributes = {
  .name = "oled_mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorFanTask(void *argument);
void StartSensorTask(void *argument);
void StartUart_UITask(void *argument);
void StartBluetoothTask(void *argument);
void StartVoiceTask(void *argument);
void StartOLEDTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* 应用初始化分两步:
     1) AppFan_Init  : 全局状态复位 + 执行器初始化(风扇/舵机/蜂鸣器),
                        先把 PWM 置为安全状态, 保证创建任务前硬件不会乱动
     2) BspUart_Init : 打开三个串口的接收中断, 之后才允许数据进来
     其它外设初始化(Sensor/OLED/协议)放在各自任务里完成, 因为它们不必抢在
     调度器启动前执行, 放在任务里还能用 osDelay 等模块上电稳定。 */
  AppFan_Init();
  BspUart_Init();
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of oled_mutex */
  oled_mutexHandle = osMutexNew(&oled_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xQueueCmd */
  xQueueCmdHandle = osMessageQueueNew (16, 8, &xQueueCmd_attributes);

  /* creation of xQueueSensorData */
  xQueueSensorDataHandle = osMessageQueueNew (8, 8, &xQueueSensorData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_MotorFan */
  Task_MotorFanHandle = osThreadNew(StartMotorFanTask, NULL, &Task_MotorFan_attributes);

  /* creation of Task_Sensor */
  Task_SensorHandle = osThreadNew(StartSensorTask, NULL, &Task_Sensor_attributes);

  /* creation of Task_Uart_UI */
  Task_Uart_UIHandle = osThreadNew(StartUart_UITask, NULL, &Task_Uart_UI_attributes);

  /* creation of Task_Bluetooth */
  Task_BluetoothHandle = osThreadNew(StartBluetoothTask, NULL, &Task_Bluetooth_attributes);

  /* creation of Task_Voice */
  Task_VoiceHandle = osThreadNew(StartVoiceTask, NULL, &Task_Voice_attributes);

  /* creation of Task_OLED */
  Task_OLEDHandle = osThreadNew(StartOLEDTask, NULL, &Task_OLED_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 本任务承担"上电初始化"职责, 做完后永久挂起, 不再占 CPU。
     不删除它的原因: 任务定义在 .ioc 里, 删掉后重新生成 CubeMX 代码又会冒出来,
     保留并挂起最稳妥。 */
  BSP_DelayInit();        /* DWT 周期计数器, 给 DHT11 提供 1us 精度延时 */
  AppSensor_Init();       /* 红外 / 压力 / DHT11 */
  AppOled_Init();         /* OLED, 没插屏也不会卡住 */
  AppUi_Init();           /* 串口屏协议 */
  AppBt_Init();           /* 蓝牙协议 */
  AppVoice_Init();        /* 语音协议 */

  vTaskSuspend(NULL);     /* 初始化完成, 交还 CPU */
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorFanTask */
/**
* @brief Function implementing the Task_MotorFan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorFanTask */
void StartMotorFanTask(void *argument)
{
  /* USER CODE BEGIN StartMotorFanTask */
  /* Task_MotorFan 主体在 App 层实现; 该函数内部是死循环, 不返回 */
  AppFan_Task(argument);
  /* USER CODE END StartMotorFanTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the Task_Sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Task_Sensor 主体在 App 层实现; 该函数内部是死循环, 不返回 */
  AppSensor_Task(argument);
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartUart_UITask */
/**
* @brief Function implementing the Task_Uart_UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart_UITask */
void StartUart_UITask(void *argument)
{
  /* USER CODE BEGIN StartUart_UITask */
  /* Task_Uart_UI 主体在 App 层实现; 该函数内部是死循环, 不返回 */
  AppUi_Task(argument);
  /* USER CODE END StartUart_UITask */
}

/* USER CODE BEGIN Header_StartBluetoothTask */
/**
* @brief Function implementing the Task_Bluetooth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetoothTask */
void StartBluetoothTask(void *argument)
{
  /* USER CODE BEGIN StartBluetoothTask */
  /* Task_Bluetooth 主体在 App 层实现; 该函数内部是死循环, 不返回 */
  AppBt_Task(argument);
  /* USER CODE END StartBluetoothTask */
}

/* USER CODE BEGIN Header_StartVoiceTask */
/**
* @brief Function implementing the Task_Voice thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVoiceTask */
void StartVoiceTask(void *argument)
{
  /* USER CODE BEGIN StartVoiceTask */
  /* Task_Voice 主体在 App 层实现; 该函数内部是死循环, 不返回 */
  AppVoice_Task(argument);
  /* USER CODE END StartVoiceTask */
}

/* USER CODE BEGIN Header_StartOLEDTask */
/**
* @brief Function implementing the Task_OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLEDTask */
void StartOLEDTask(void *argument)
{
  /* USER CODE BEGIN StartOLEDTask */
  /* Task_OLED 主体在 App 层实现; 该函数内部是死循环, 不返回 */
  AppOled_Task(argument);
  /* USER CODE END StartOLEDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

