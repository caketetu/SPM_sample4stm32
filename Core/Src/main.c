/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  *Simple Programable Module（SPM）スレーブデバイスのSTM32用サンプル
  *プロトコルはSPM_Modbus.hに格納。SPM_ModbusTaskを適切に実行すれば勝手にレジスタの読み書きが実行される。
  *コイル16点　インプットレジスタ32点　保持レジスタ16点
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <SPM_Modbus.h>

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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int16_t ro_value[16];
int16_t rw_value[16];
uint8_t dio=0;

uint8_t recv_data;
uint8_t rx_buf[50];
int rx_len=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ro_value[0] = 48;   //モデルナンバー
  ro_value[1] = 0002; //バージョン
  ro_value[2] = 0000; //サブバージョン

  //各レジスタ初期化
  p_coils[0] = &dio;//&rw_value[0];
  p_coils[1] = &dio;//&rw_value[1];

  for(int i=0; i<16; i++){
    p_holding_regs[i] = &rw_value[i];
    p_input_regs[i] = &ro_value[i];
  }
  for(int i=16; i<32; i++){
    p_input_regs[i] = &rw_value[i-16];
  }

  //Cyclic Function0登録
  cycfunc0.rx_len = 1;
  cycfunc0.rx_adr[0] = &rw_value[0];
  cycfunc0.tx_len = 1;
  cycfunc0.tx_adr[0] = &rw_value[0];

  //Cyclic Function1登録
  cycfunc1.rx_len = 2;
  cycfunc1.rx_adr[0] = &rw_value[0];
  cycfunc1.rx_adr[1] = &rw_value[1];
  cycfunc1.tx_len = 2;
  cycfunc1.tx_adr[0] = &rw_value[0];
  cycfunc1.tx_adr[1] = &rw_value[1];

  //Cyclic Function2登録
  cycfunc2.rx_len = 3;
  cycfunc2.rx_adr[0] = &rw_value[0];
  cycfunc2.rx_adr[1] = &rw_value[1];
  cycfunc2.rx_adr[2] = &rw_value[2];
  cycfunc2.tx_len = 3;
  cycfunc2.tx_adr[0] = &rw_value[0];
  cycfunc2.tx_adr[1] = &rw_value[1];
  cycfunc2.tx_adr[2] = &rw_value[2];

  //Cyclic Function3登録
  cycfunc3.rx_len = 1;
  cycfunc3.rx_adr[0] = &rw_value[0];
  cycfunc3.tx_len = 2;
  cycfunc3.tx_adr[0] = &rw_value[0];
  cycfunc3.tx_adr[1] = &rw_value[1];

  //Cyclic Function4登録
  cycfunc4.rx_len = 8;
  cycfunc4.rx_adr[0] = &rw_value[0];
  cycfunc4.rx_adr[1] = &rw_value[1];
  cycfunc4.rx_adr[2] = &rw_value[2];
  cycfunc4.rx_adr[3] = &rw_value[3];
  cycfunc4.rx_adr[4] = &rw_value[4];
  cycfunc4.rx_adr[5] = &rw_value[5];
  cycfunc4.rx_adr[6] = &rw_value[6];
  cycfunc4.rx_adr[7] = &rw_value[7];
  cycfunc4.tx_len = 8;
  cycfunc4.tx_adr[0] = &rw_value[0];
  cycfunc4.tx_adr[1] = &rw_value[1];
  cycfunc4.tx_adr[2] = &rw_value[2];
  cycfunc4.tx_adr[3] = &rw_value[3];
  cycfunc4.tx_adr[4] = &rw_value[4];
  cycfunc4.tx_adr[5] = &rw_value[5];
  cycfunc4.tx_adr[6] = &rw_value[6];
  cycfunc4.tx_adr[7] = &rw_value[7];

  SPM_ModbusSetAddress(22);

  HAL_UART_Receive_IT(&huart2, &recv_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(rx_len>0){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		  HAL_Delay(10);
		  uint8_t tx_buf[50];
		  int tx_len = SPM_ModbusTask(rx_buf, rx_len, tx_buf);
		  HAL_UART_Transmit(&huart2, tx_buf, tx_len, 100);
		  rx_len=0;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART2)
   {
	   rx_buf[rx_len] = recv_data;
	   rx_len++;
       HAL_UART_Receive_IT(&huart2, &recv_data, 1);
   }
}

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
