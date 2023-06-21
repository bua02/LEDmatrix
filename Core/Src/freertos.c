/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "c7x10r_font.h"
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
extern SPI_HandleTypeDef hspi3;
uint8_t rxDone = 3;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for i2cTask */
osThreadId_t i2cTaskHandle;
const osThreadAttr_t i2cTask_attributes = {
  .name = "i2cTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for i2cMessageQueue */
osMessageQueueId_t i2cMessageQueueHandle;
const osMessageQueueAttr_t i2cMessageQueue_attributes = {
  .name = "i2cMessageQueue"
};
/* Definitions for dataReceived */
osEventFlagsId_t dataReceivedHandle;
const osEventFlagsAttr_t dataReceived_attributes = {
  .name = "dataReceived"
};
/* Definitions for i2cAddressed */
osEventFlagsId_t i2cAddressedHandle;
const osEventFlagsAttr_t i2cAddressed_attributes = {
  .name = "i2cAddressed"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void displayTask_App(void *argument);
void i2cTask_App(void *argument);

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

  /* Create the queue(s) */
  /* creation of i2cMessageQueue */
  i2cMessageQueueHandle = osMessageQueueNew (3, sizeof(uint8_t), &i2cMessageQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of displayTask */
  displayTaskHandle = osThreadNew(displayTask_App, NULL, &displayTask_attributes);

  /* creation of i2cTask */
  i2cTaskHandle = osThreadNew(i2cTask_App, NULL, &i2cTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of dataReceived */
  dataReceivedHandle = osEventFlagsNew(&dataReceived_attributes);

  /* creation of i2cAddressed */
  i2cAddressedHandle = osEventFlagsNew(&i2cAddressed_attributes);

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_displayTask_App */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayTask_App */
void displayTask_App(void *argument)
{
  /* USER CODE BEGIN displayTask_App */
	uint8_t pulse = 0;
	Numbers singleDigits;
	osStatus_t result;
  /* Infinite loop */
  for(;;)
  {
	  result = osMessageQueueGet(i2cMessageQueueHandle, &pulse, 0U, 0U);
	  if(result == osOK){
		  singleDigits = getSingleDigits(pulse);
	  }else if(pulse == 0){
		  singleDigits.tens = 87 - '0';
		  singleDigits.ones = 8;
	  }

	  for(int i = 0; i<=7; i++){
	  		  if(i==0){
	  			  HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
	  			  HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);
	  		  }

	  		  while(HAL_SPI_GetState(&hspi3)!=HAL_SPI_STATE_READY);
	  		  HAL_SPI_Transmit_IT(&hspi3, &c7x10r_font[(singleDigits.ones + '0')*8+i], 1);
	  		  while(HAL_SPI_GetState(&hspi3)!=HAL_SPI_STATE_READY);
	  		  HAL_SPI_Transmit_IT(&hspi3, &c7x10r_font[(singleDigits.tens + '0')*8+i], 1);
	  		  HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, SET);
	  		  HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, RESET);
	  		 if(i > 0){
	  				HAL_GPIO_WritePin(RClock_GPIO_Port, RClock_Pin, GPIO_PIN_SET);
	  				HAL_GPIO_WritePin(RClock_GPIO_Port, RClock_Pin, GPIO_PIN_RESET);
	  		 }
	  	  }
	  	  HAL_GPIO_WritePin(R_Reset_GPIO_Port, R_Reset_Pin, GPIO_PIN_SET);
	  	  HAL_GPIO_WritePin(R_Reset_GPIO_Port, R_Reset_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END displayTask_App */
}

/* USER CODE BEGIN Header_i2cTask_App */
/**
* @brief Function implementing the i2cTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_i2cTask_App */
void i2cTask_App(void *argument)
{
  /* USER CODE BEGIN i2cTask_App */
	uint8_t rxdata [] = {0};
	extern I2C_HandleTypeDef hi2c1;
  /* Infinite loop */
  for(;;)
  {
	  if(rxDone == 3){
		  HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)rxdata, 8);
	  }else if(rxDone == 1){
		  rxDone = 0;
		  osMessageQueuePut(i2cMessageQueueHandle, (uint8_t *)rxdata, 0U, 0U);
		  HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)rxdata, 8);
	  }
  }
  /* USER CODE END i2cTask_App */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	 rxDone = 1;
}
/* USER CODE END Application */

