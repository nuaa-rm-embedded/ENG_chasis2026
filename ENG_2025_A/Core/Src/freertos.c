/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
/* Definitions for Arm_Ctrl */
osThreadId_t Arm_CtrlHandle;
const osThreadAttr_t Arm_Ctrl_attributes = {
  .name = "Arm_Ctrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Moto */
osThreadId_t MotoHandle;
const osThreadAttr_t Moto_attributes = {
  .name = "Moto",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Mode */
osThreadId_t ModeHandle;
const osThreadAttr_t Mode_attributes = {
  .name = "Mode",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Transmit */
osThreadId_t TransmitHandle;
const osThreadAttr_t Transmit_attributes = {
  .name = "Transmit",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Lift */
osThreadId_t LiftHandle;
const osThreadAttr_t Lift_attributes = {
  .name = "Lift",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for USB */
osThreadId_t USBHandle;
const osThreadAttr_t USB_attributes = {
  .name = "USB",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SelfCheck */
osThreadId_t SelfCheckHandle;
const osThreadAttr_t SelfCheck_attributes = {
  .name = "SelfCheck",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UI */
osThreadId_t UIHandle;
const osThreadAttr_t UI_attributes = {
  .name = "UI",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Arm_Ctrl_Task(void *argument);
void MotoTask(void *argument);
void ModeTask(void *argument);
void TransmitTask(void *argument);
void LiftTask(void *argument);
void usb_task(void *argument);
void SelfCheck_mode(void *argument);
void ui_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Arm_Ctrl */
  Arm_CtrlHandle = osThreadNew(Arm_Ctrl_Task, NULL, &Arm_Ctrl_attributes);

  /* creation of Moto */
  MotoHandle = osThreadNew(MotoTask, NULL, &Moto_attributes);

  /* creation of Mode */
  ModeHandle = osThreadNew(ModeTask, NULL, &Mode_attributes);

  /* creation of Transmit */
  TransmitHandle = osThreadNew(TransmitTask, NULL, &Transmit_attributes);

  /* creation of Lift */
  LiftHandle = osThreadNew(LiftTask, NULL, &Lift_attributes);

  /* creation of USB */
  USBHandle = osThreadNew(usb_task, NULL, &USB_attributes);

  /* creation of SelfCheck */
  SelfCheckHandle = osThreadNew(SelfCheck_mode, NULL, &SelfCheck_attributes);

  /* creation of UI */
  UIHandle = osThreadNew(ui_task, NULL, &UI_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Arm_Ctrl_Task */
/**
  * @brief  Function implementing the Arm_Ctrl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Arm_Ctrl_Task */
__weak void Arm_Ctrl_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Arm_Ctrl_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Arm_Ctrl_Task */
}

/* USER CODE BEGIN Header_MotoTask */
/**
* @brief Function implementing the Moto thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotoTask */
__weak void MotoTask(void *argument)
{
  /* USER CODE BEGIN MotoTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotoTask */
}

/* USER CODE BEGIN Header_ModeTask */
/**
* @brief Function implementing the Mode thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModeTask */
__weak void ModeTask(void *argument)
{
  /* USER CODE BEGIN ModeTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ModeTask */
}

/* USER CODE BEGIN Header_TransmitTask */
/**
* @brief Function implementing the Transmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TransmitTask */
__weak void TransmitTask(void *argument)
{
  /* USER CODE BEGIN TransmitTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TransmitTask */
}

/* USER CODE BEGIN Header_LiftTask */
/**
* @brief Function implementing the Lift thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LiftTask */
__weak void LiftTask(void *argument)
{
  /* USER CODE BEGIN LiftTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LiftTask */
}

/* USER CODE BEGIN Header_usb_task */
/**
* @brief Function implementing the USB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usb_task */
__weak void usb_task(void *argument)
{
  /* USER CODE BEGIN usb_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usb_task */
}

/* USER CODE BEGIN Header_SelfCheck_mode */
/**
* @brief Function implementing the SelfCheck thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SelfCheck_mode */
__weak void SelfCheck_mode(void *argument)
{
  /* USER CODE BEGIN SelfCheck_mode */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SelfCheck_mode */
}

/* USER CODE BEGIN Header_ui_task */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ui_task */
__weak void ui_task(void *argument)
{
  /* USER CODE BEGIN ui_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ui_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

