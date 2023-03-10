/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t counter = 0;
uint8_t temp1, temp2, temp3, temp4;

#define D1_HIGH() HAL_GPIO_WritePin(GPIOB, Dig1_Pin, GPIO_PIN_SET)
#define D1_LOW() HAL_GPIO_WritePin(GPIOB, Dig1_Pin, GPIO_PIN_RESET)
#define D2_HIGH() HAL_GPIO_WritePin(GPIOB, Dig2_Pin, GPIO_PIN_SET)
#define D2_LOW() HAL_GPIO_WritePin(GPIOB, Dig2_Pin, GPIO_PIN_RESET)
#define D3_HIGH() HAL_GPIO_WritePin(GPIOB, Dig3_Pin, GPIO_PIN_SET)
#define D3_LOW() HAL_GPIO_WritePin(GPIOB, Dig3_Pin, GPIO_PIN_RESET)
#define D4_HIGH() HAL_GPIO_WritePin(GPIOB, Dig4_Pin, GPIO_PIN_SET)
#define D4_LOW() HAL_GPIO_WritePin(GPIOB, Dig4_Pin, GPIO_PIN_RESET)
uint8_t segmentNumber[10] = {
        0xc0,  // 0
        0xf9,  // 1
        0xa4,  // 2
        0xb0,  // 3
        0x99,  // 4
        0x92,  // 5
        0x82,  // 6
        0xf8,  // 7
        0x80,  // 8
        0x90   // 9
};
void SevenSegment_Update(uint8_t number){
	HAL_GPIO_WritePin(GPIOA, A_Pin, ((number>>0)&0x01));
	HAL_GPIO_WritePin(GPIOA, B_Pin, ((number>>1)&0x01));
	HAL_GPIO_WritePin(GPIOA, C_Pin, ((number>>2)&0x01));
	HAL_GPIO_WritePin(GPIOA, D_Pin, ((number>>3)&0x01));
	HAL_GPIO_WritePin(GPIOA, E_Pin, ((number>>4)&0x01));
	HAL_GPIO_WritePin(GPIOA, F_Pin, ((number>>5)&0x01));
	HAL_GPIO_WritePin(GPIOA, G_Pin, ((number>>6)&0x01));
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		temp1 = counter/1000; //1 - 1st digit
	  temp2 = ((counter/100)%10); //2 - 2nd digit
	  temp3 = ((counter/10)%10); //3 - 3rd digit
	  temp4 = (counter%10); //4 - 4th digit

	  SevenSegment_Update(segmentNumber[temp1]);
		D1_HIGH();
		HAL_Delay(70);
	  D1_LOW();

	  SevenSegment_Update(segmentNumber[temp2]);
	  D2_HIGH();
	  HAL_Delay(70);
	  D2_LOW();

	  SevenSegment_Update(segmentNumber[temp3]);
	  D3_HIGH();
	  HAL_Delay(70);
	  D3_LOW();

	  SevenSegment_Update(segmentNumber[temp4]);
	  D4_HIGH();
	  HAL_Delay(70);
	  D4_LOW();

	  counter++;

	  if (counter >= 1000) {
		counter = 0;
	}
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DP_Pin|G_Pin|F_Pin|E_Pin
                          |D_Pin|C_Pin|B_Pin|A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Dig1_Pin|Dig2_Pin|Dig3_Pin|Dig4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DP_Pin G_Pin F_Pin E_Pin
                           D_Pin C_Pin B_Pin A_Pin */
  GPIO_InitStruct.Pin = DP_Pin|G_Pin|F_Pin|E_Pin
                          |D_Pin|C_Pin|B_Pin|A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Dig1_Pin Dig2_Pin Dig3_Pin Dig4_Pin */
  GPIO_InitStruct.Pin = Dig1_Pin|Dig2_Pin|Dig3_Pin|Dig4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
