/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // spritf
#include <unistd.h>
#include "dht11.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"
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
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask08Queue */
osThreadId_t myTask08QueueHandle;
const osThreadAttr_t myTask08Queue_attributes = {
  .name = "myTask08Queue",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask09QueueRX */
osThreadId_t myTask09QueueRXHandle;
const osThreadAttr_t myTask09QueueRX_attributes = {
  .name = "myTask09QueueRX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask10_contro */
osThreadId_t myTask10_controHandle;
const osThreadAttr_t myTask10_contro_attributes = {
  .name = "myTask10_contro",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask11 */
osThreadId_t myTask11Handle;
const osThreadAttr_t myTask11_attributes = {
  .name = "myTask11",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask12 */
osThreadId_t myTask12Handle;
const osThreadAttr_t myTask12_attributes = {
  .name = "myTask12",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Q_dht11_control */
osMessageQueueId_t Q_dht11_controlHandle;
const osMessageQueueAttr_t Q_dht11_control_attributes = {
  .name = "Q_dht11_control"
};
/* Definitions for Q_waterLevel_control */
osMessageQueueId_t Q_waterLevel_controlHandle;
const osMessageQueueAttr_t Q_waterLevel_control_attributes = {
  .name = "Q_waterLevel_control"
};
/* Definitions for Q_control_TX */
osMessageQueueId_t Q_control_TXHandle;
const osMessageQueueAttr_t Q_control_TX_attributes = {
  .name = "Q_control_TX"
};
/* Definitions for Q_RX_control */
osMessageQueueId_t Q_RX_controlHandle;
const osMessageQueueAttr_t Q_RX_control_attributes = {
  .name = "Q_RX_control"
};
/* USER CODE BEGIN PV */
osEventFlagsId_t flag_Event;
uint8_t mode = 0;
SemaphoreHandle_t delaySemaphore;
osMessageQueueId_t myQueue; // Mensaje para mandar datos de task8 -> task9

uint8_t buffRX[8] = {0};
uint16_t buff_len = 8;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08Queue(void *argument);
void StartTask09QueueRX(void *argument);
void StartTask10_controlQUEUE(void *argument);
void StartTask11(void *argument);
void StartTask12(void *argument);

/* USER CODE BEGIN PFP */
void DHT11_Delay_f(uint32_t uSeg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	mode ++;
	if(mode == 3){
		mode = 0;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of Q_dht11_control */
  Q_dht11_controlHandle = osMessageQueueNew (3, sizeof(uint32_t), &Q_dht11_control_attributes);

  /* creation of Q_waterLevel_control */
  Q_waterLevel_controlHandle = osMessageQueueNew (3, sizeof(uint32_t), &Q_waterLevel_control_attributes);

  /* creation of Q_control_TX */
  Q_control_TXHandle = osMessageQueueNew (3, sizeof(uint32_t), &Q_control_TX_attributes);

  /* creation of Q_RX_control */
  Q_RX_controlHandle = osMessageQueueNew (3, sizeof(uint32_t), &Q_RX_control_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  // Crear una cola de mensajes con capacidad para 10 mensajes, cada uno de 4 bytes
  	  myQueue = osMessageQueueNew(10, 4, NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, NULL, &myTask06_attributes);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);

  /* creation of myTask08Queue */
  myTask08QueueHandle = osThreadNew(StartTask08Queue, NULL, &myTask08Queue_attributes);

  /* creation of myTask09QueueRX */
  myTask09QueueRXHandle = osThreadNew(StartTask09QueueRX, NULL, &myTask09QueueRX_attributes);

  /* creation of myTask10_contro */
  myTask10_controHandle = osThreadNew(StartTask10_controlQUEUE, NULL, &myTask10_contro_attributes);

  /* creation of myTask11 */
  myTask11Handle = osThreadNew(StartTask11, NULL, &myTask11_attributes);

  /* creation of myTask12 */
  myTask12Handle = osThreadNew(StartTask12, NULL, &myTask12_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 84-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 1-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 84-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart1){

	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart2){
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	osStatus_t status_Q_RX_control;
	uint32_t sendMsg_Q_RX_control;
	uint8_t priMsg_Q_RX_control;
	status_Q_RX_control = osMessageQueuePut(Q_RX_controlHandle, &sendMsg_Q_RX_control, &priMsg_Q_RX_control, osWaitForever);
	if (status_Q_RX_control == osOK) {
		osEventFlagsSet(flag_Event, 0x20);
	}else {
		; // Error mesaje
	}

	// Habilitar interrupcion (8 datos salta interrupcion)
	HAL_UART_Receive_IT(&huart2, buffRX, buff_len);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim11){
		osEventFlagsSet(flag_Event, 0x01);
		HAL_TIM_Base_Stop_IT(&htim11);
	}
}

void DHT11_Delay_f(uint32_t uSeg) {

	 htim11.Instance = TIM11;
	  htim11.Init.Prescaler = (uSeg - 44) -1;
	  htim11.Init.Period = 84-1;
	  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  // Deshabilitar la interrupción de desbordamiento en TIM11 usando HAL
	  __HAL_TIM_DISABLE_IT(&htim11, TIM_IT_UPDATE);
	  // Borrar la bandera de desbordamiento en TIM11 usando HAL
	  __HAL_TIM_CLEAR_FLAG(&htim11, TIM_FLAG_UPDATE);
	  __HAL_TIM_SET_COUNTER(&htim11, 0);

	osEventFlagsClear(flag_Event, 0x01); // Iniciar valor de flag

	HAL_TIM_Base_Start_IT(&htim11); // iniciar timer

	osEventFlagsWait(flag_Event, 0x01, osFlagsWaitAll, osWaitForever);

	// Detener el temporizador
	HAL_TIM_Base_Stop_IT(&htim11);

	// Limpiar el flag de evento
	osEventFlagsClear(flag_Event, 0x01);
}

void Delay_us(uint32_t us)
{
    // Convertir microsegundos a milisegundos
    uint32_t ms = (us + 999) / 1000;

    // Esperar en milisegundos
    vTaskDelay(ms);
}

void delayWithNop(uint32_t delayTime) {
	delayTime = delayTime*7;

	for (uint32_t i = 0; i < delayTime; ++i) {
        __NOP();
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	uint8_t buffTX[40] = "HOLA";
	uint16_t buff_len = 40;

	osStatus_t status_Q_control_TX;

	uint32_t recMsg_Q_control_TX;
	uint8_t priMsg_Q_control_TX;

	uint8_t hum, temp, watLev;

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
  for(;;)
  {

		osEventFlagsWait(flag_Event, 0x10, osFlagsWaitAll, osWaitForever);
		status_Q_control_TX = osMessageQueueGet(Q_control_TXHandle, &recMsg_Q_control_TX, &priMsg_Q_control_TX, osWaitForever);
		if (status_Q_control_TX == osOK) {
			// Mensaje bien recibido

			hum = (recMsg_Q_control_TX & 0xFF);
			temp = ((recMsg_Q_control_TX>>8) & 0xFF);
			watLev = ((recMsg_Q_control_TX>>16) & 0xFF);

			buff_len = sprintf(&buffTX, "hum %d, temp %d, watLev %d\n", hum, temp, watLev);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffTX, buff_len, 100);
		}


	  //HAL_UART_Transmit_IT(&huart1, bufferEnvio, tam);
    osDelay(1000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	// La funcion " HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart1)" tiene la logica del programa

	osStatus_t status_Q_RX_control;

	uint32_t sendMsg_Q_RX_control;
	uint8_t priMsg_Q_RX_control;

	uint8_t codigo;

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	// Start interrupt reception
	HAL_UART_Receive_IT(&huart2, buffRX, buff_len); // Variables globales cambian el funcionamiento
  for(;;)
  {
	  HAL_UART_Receive_IT(&huart2, buffRX, buff_len);
     osDelay(100); // Cada 100 milisegundos habilita interrupción
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	uint8_t buff_erre[2] = "XX";
		uint8_t humidity, temperature;
		uint8_t buff_print[24] ;


//		// Crear variable y cargar valores inicialies de puerto
//		DHT11_TypeDef dht11;
//		dht11.GPIO_Port = DHT11_GPIO_Port;  // Reemplaza 'x' con el puerto GPIO que estás utilizando
//		dht11.GPIO_Pin = DHT11_Pin;  // Reemplaza 'x' con el número del pin GPIO que estás utilizando
//		DHT11_Init(&dht11);

		osDelay(1000);
  for(;;)
  {
	  //HAL_UART_Transmit(&huart2, buff_print, 23, 10U);
//	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//
//	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//	  	 osDelay(1000);
//	  	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//	  	 osDelay(1000);
//	  	 if (DHT11_ReadData_prueba(&dht11, &humidity, &temperature) == HAL_OK) {
//	  		// Los datos se leyeron correctamente
//	  		// Utiliza las variables 'humidity' y 'temperature' según tus necesidades
//
//	   		  formatSensorData(buff_print,  humidity,  temperature);
//
//	  		  //HAL_UART_Transmit(&huart2, buff_print, 23, 10U);
//	  	  } else {
//	  		// Hubo un error al leer los datos del sensor
//	  		//HAL_UART_Transmit(&huart2, buff_erre, 2, 10U);
//	  	  }

    osDelay(100);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {

	  if (mode == 0){
	  		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  		  osDelay(100);
	  	  }
	  	  if (mode == 1){
	  		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  		  		  osDelay(1000);
	  	  }
	  	  if (mode == 2){
	  		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  		  osDelay(2000);
	  	  }

  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */


  /* Infinite loop */

	  char uart_buf[50];
	  int uart_buf_len;
	  uint16_t timer_val;
	  uint16_t timer_val2;
	  // Say something
	    uart_buf_len = sprintf(uart_buf, "Timer Test\r\n");

	    //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	    // Start timer
	    HAL_TIM_Base_Start(&htim10);
  for(;;)
  {
	  	  osDelay(1000);
	  	HAL_TIM_Base_Start(&htim10);
	  	  timer_val = __HAL_TIM_GET_COUNTER(&htim10);

        // Wait for 50 ms
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    	//DHT11_Delay_f(100);
    	delayWithNop(7*20);
    	//osDelay(1);
    	//Delay_us(1);
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

        // Get time elapsed
        timer_val = __HAL_TIM_GET_COUNTER(&htim10) - timer_val;
        timer_val2 = __HAL_TIM_GET_COUNTER(&htim11);
        // Show elapsed time
        HAL_TIM_Base_Stop_IT(&htim10);
        uart_buf_len = sprintf(uart_buf, "%u us\r\n", timer_val);
        //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
        uart_buf_len = sprintf(uart_buf, "CNT:%u | ARR 83\r\n\n", timer_val2);
        //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
  }

  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
	/*
	char uart_buf[50];
		  int uart_buf_len;
		  uint16_t timer_val;
		  uint16_t timer_val2;
		  // Say something
		    uart_buf_len = sprintf(uart_buf, "Timer Test\r\n");
		    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

		    // Start timer
		    HAL_TIM_Base_Start(&htim10);

	InitDelayTimer();
  for(;;)
  {


	  timer_val = __HAL_TIM_GET_COUNTER(&htim10);

	          // Wait for 50 ms
	          //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  Delay_us(10000);
	          //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	          // Get time elapsed
	          timer_val = __HAL_TIM_GET_COUNTER(&htim10) - timer_val;
	          timer_val2 = __HAL_TIM_GET_COUNTER(&htim11);
	          // Show elapsed time
	          uart_buf_len = sprintf(uart_buf, "%u us\r\n", timer_val);
	          HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
	          uart_buf_len = sprintf(uart_buf, "CNT:%u | ARR 83\r\n\n", timer_val2);
	          HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);*/
	for(;;)
	  {osDelay(1000);
  }
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08Queue */
/**
* @brief Function implementing the myTask08Queue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08Queue */
void StartTask08Queue(void *argument)
{
  /* USER CODE BEGIN StartTask08Queue */
  /* Infinite loop */
	uint32_t myMessage = 42;
  for(;;)
  {
	  osStatus_t status = osMessageQueuePut(myQueue, &myMessage, 0, osWaitForever);
	  if (status == osOK) {
	      // El mensaje fue colocado exitosamente
		  myMessage --;
	  } else {
	      // Hubo un problema al colocar el mensaje
	  }

    osDelay(1000);
  }
  /* USER CODE END StartTask08Queue */
}

/* USER CODE BEGIN Header_StartTask09QueueRX */
/**
* @brief Function implementing the myTask09QueueRX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask09QueueRX */
void StartTask09QueueRX(void *argument)
{
  /* USER CODE BEGIN StartTask09QueueRX */
  /* Infinite loop */
	uint32_t receivedMessage;
	uint8_t messagePriority;

	char uart_buf[30];
	int uart_buf_len = 30;
  for(;;)
  {

	  osStatus_t status = osMessageQueueGet(myQueue, &receivedMessage, &messagePriority, osWaitForever);

	  if (status == osOK) {
	      // Mensaje recibido exitosamente
		  uart_buf_len = sprintf(uart_buf, "mes = %u prio = %uc\r\n", receivedMessage, messagePriority);
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	  } else {
	      // Hubo un problema al obtener el mensaje
	  }
    osDelay(800);
  }
  /* USER CODE END StartTask09QueueRX */
}

/* USER CODE BEGIN Header_StartTask10_controlQUEUE */
/**
* @brief Function implementing the myTask10_contro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask10_controlQUEUE */
void StartTask10_controlQUEUE(void *argument)
{
  /* USER CODE BEGIN StartTask10_controlQUEUE */
  /* Infinite loop */
	flag_Event = osEventFlagsNew(NULL);

	uint8_t cod = 0x01;

	osStatus_t status_Q_RX_control;
	osStatus_t status_Q_dht11_control;
	osStatus_t status_Q_waterLevel_control;
	osStatus_t status_Q_control_TX;

	uint32_t recMsg_Q_RX_control;
	uint8_t priMsg_Q_RX_control;

	uint32_t recMsg_Q_dht11_control, recMsg_Q_waterLevel_control;
	uint8_t priMsg_Q_dht11_control, priMsg_Q_waterLevel_control;

	// Recived value
	uint8_t temp, hum, watLev;

	// envio mensaje
	uint32_t sendMsg_Q_control_TX;
  for(;;)
  {

	  // Recivir mensaje (RX)
	  osEventFlagsWait(flag_Event, 0x20, osFlagsWaitAll, osWaitForever);
	  status_Q_RX_control = osMessageQueueGet(Q_RX_controlHandle, &recMsg_Q_RX_control, &priMsg_Q_waterLevel_control, osWaitForever);
	  if (status_Q_RX_control == osOK) {
     	 // Todo bien leido
		  cod = (recMsg_Q_RX_control >> 24) & 0xFF;

		  switch(cod){
			  case 'a': // Valor de codigo que se necesite ajustar
				  // Activar dht11
				osEventFlagsSet(flag_Event, 0x01);
				  // Activar waterLevel
				osEventFlagsSet(flag_Event, 0x02);
				// Lectura de humedity / temperature / waterLevel

				osEventFlagsWait(flag_Event, 0x04 | 0x08, osFlagsWaitAll, osWaitForever);
				status_Q_dht11_control = osMessageQueueGet(Q_dht11_controlHandle, &recMsg_Q_dht11_control, &priMsg_Q_dht11_control, osWaitForever);
				status_Q_waterLevel_control = osMessageQueueGet(Q_waterLevel_controlHandle, &recMsg_Q_waterLevel_control, &priMsg_Q_waterLevel_control, osWaitForever);
				if ((status_Q_dht11_control & status_Q_waterLevel_control) == osOK) {
					// Todo bien leido
					hum = (recMsg_Q_dht11_control & 0xFF); // 0x0a (Example)
					temp = ((recMsg_Q_dht11_control>>8) & 0xFF); //0x14
					watLev = (recMsg_Q_waterLevel_control & 0xFF); // 0x1e
					sprintf(& sendMsg_Q_control_TX,"%c%c%c%c", hum, temp, watLev, 0); // 0x001e140a (0/wat/temp/hum)
					// msg enviar TX
					status_Q_control_TX = osMessageQueuePut(Q_control_TXHandle, &sendMsg_Q_control_TX, 0, osWaitForever);
					if (status_Q_control_TX == osOK) {
						// Avisar TASK
						osEventFlagsSet(flag_Event, 0x10);
						// Bien enviado
						;
					}else{
						// Mal enviado
						;
					}
				}else{
					;// Mala recepción
				}



			  }
		}else {
		  ;  // Error
		}


	  	//			  // Iniciar flag / borrar / esperar
	  	//			osEventFlagsSet(flag_Event, 0x01);
	  	//			osEventFlagsClear(flag_Event, 0x01); // Iniciar valor de flag
	  	//			osEventFlagsWait(flag_Event, 0x01, osFlagsWaitAll, osWaitForever);
	  	//			  break;
	  	//			osStatus_t status = osMessageQueuePut(myQueue, &myMessage, 0, osWaitForever);
	  	//			if (status == osOK) {;}else{;}
	  	//			osStatus_t status = osMessageQueueGet(myQueue, &receivedMessage, &messagePriority, osWaitForever);
	  	//			if (status == osOK) {;}else{;}

	  osDelay(100);
  }
  /* USER CODE END StartTask10_controlQUEUE */
}

/* USER CODE BEGIN Header_StartTask11 */
/**
* @brief Function implementing the myTask11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask11 */
void StartTask11(void *argument)
{
  /* USER CODE BEGIN StartTask11 */
	//////////////////////// DHT11 TASK
  /* Infinite loop */

	// Crear variable y cargar valores inicialies de puerto
	DHT11_TypeDef dht11;
	dht11.GPIO_Port = DHT11_GPIO_Port;  // Reemplaza 'x' con el puerto GPIO que estás utilizando
	dht11.GPIO_Pin = DHT11_Pin;  // Reemplaza 'x' con el número del pin GPIO que estás utilizando
	DHT11_Init(&dht11);

	osStatus_t status_Q_dht11_control;

	uint32_t sendMsg_Q_dht11_control;

	uint8_t humidity, temperature;

  for(;;)
  {
	  // Desbloquear lectura y humedad
	  osEventFlagsWait(flag_Event, 0x01, osFlagsWaitAll, osWaitForever);
	  //if (DHT11_ReadData_prueba(&dht11, &humidity, &temperature) == HAL_OK) { // Se podria hacer una media de lecturas
	  	if (1){
	  	  // Datos bien recividos
		  humidity = 70; // 0x46
		  temperature = 18; // 0x12
		  sprintf(&sendMsg_Q_dht11_control, "%c%c%c%c", humidity, temperature, 0, 0); // 0x00001246 (0/0/temp/hum)
		  status_Q_dht11_control = osMessageQueuePut(Q_dht11_controlHandle, &sendMsg_Q_dht11_control, 0, osWaitForever);
			if (status_Q_dht11_control == osOK) {
				// Bien enviado
				osEventFlagsSet(flag_Event, 0x04);
			}else{
				// Mal enviado
				;
			}


	  } else {
		  ;// Datos mal leidos
	  }


    osDelay(1);
  }
  /* USER CODE END StartTask11 */
}

/* USER CODE BEGIN Header_StartTask12 */
/**
* @brief Function implementing the myTask12 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask12 */
void StartTask12(void *argument)
{
  /* USER CODE BEGIN StartTask12 */
  /* Infinite loop */
	osStatus_t status_Q_waterLevel_control;

	uint32_t sendMsg_Q_waterLevel_control;

	uint32_t potWaterLevel;
	uint8_t waterLevel;

	for(;;)
	{
		// Desbloquear lectura y humedad
		osEventFlagsWait(flag_Event, 0x02, osFlagsWaitAll, osWaitForever);
		// Posible hacer mas medidas de una y hacer media
		//potWaterLevel = readAnalogA0 (); // Falta por implementar
		//waterLevel = getWaterLevel (potWaterLevel);
		waterLevel = 60;

		  sprintf(&sendMsg_Q_waterLevel_control, "%c%c%c%c", waterLevel, 0, 0, 0); //(0/0/0/watLev)
		  status_Q_waterLevel_control = osMessageQueuePut(Q_waterLevel_controlHandle, &sendMsg_Q_waterLevel_control, 0, osWaitForever);
			if (status_Q_waterLevel_control == osOK) {
				// Bien enviado
				osEventFlagsSet(flag_Event, 0x08);
			}else{
				// Mal enviado
				;
			}



		osDelay(1);
	}
  /* USER CODE END StartTask12 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
