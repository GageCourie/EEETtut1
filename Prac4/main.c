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
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS			128		// Number of samples in LUT
#define TIM2CLK		8000000	// STM Clock frequency
#define F_SIGNAL	1000	// Frequency of output analog signal

#define SINIDX		0			//Index to be used for SIN
#define TRIIDX		1			//Index to be used for TRI
#define SAWIDX		2			//Index to be used for SAW
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

uint32_t Sin_LUT[NS] = {
		0x200,
		0x219,
		0x232,
		0x24b,
		0x263,
		0x27c,
		0x294,
		0x2ac,
		0x2c3,
		0x2da,
		0x2f1,
		0x306,
		0x31c,
		0x330,
		0x344,
		0x357,
		0x369,
		0x37a,
		0x38b,
		0x39a,
		0x3a9,
		0x3b6,
		0x3c3,
		0x3ce,
		0x3d8,
		0x3e1,
		0x3e9,
		0x3f0,
		0x3f5,
		0x3f9,
		0x3fd,
		0x3fe,
		0x3ff,
		0x3fe,
		0x3fd,
		0x3f9,
		0x3f5,
		0x3f0,
		0x3e9,
		0x3e1,
		0x3d8,
		0x3ce,
		0x3c3,
		0x3b6,
		0x3a9,
		0x39a,
		0x38b,
		0x37a,
		0x369,
		0x357,
		0x344,
		0x330,
		0x31c,
		0x306,
		0x2f1,
		0x2da,
		0x2c3,
		0x2ac,
		0x294,
		0x27c,
		0x263,
		0x24b,
		0x232,
		0x219,
		0x200,
		0x1e6,
		0x1cd,
		0x1b4,
		0x19c,
		0x183,
		0x16b,
		0x153,
		0x13c,
		0x125,
		0x10e,
		0xf9,
		0xe3,
		0xcf,
		0xbb,
		0xa8,
		0x96,
		0x85,
		0x74,
		0x65,
		0x56,
		0x49,
		0x3c,
		0x31,
		0x27,
		0x1e,
		0x16,
		0xf,
		0xa,
		0x6,
		0x2,
		0x1,
		0x0,
		0x1,
		0x2,
		0x6,
		0xa,
		0xf,
		0x16,
		0x1e,
		0x27,
		0x31,
		0x3c,
		0x49,
		0x56,
		0x65,
		0x74,
		0x85,
		0x96,
		0xa8,
		0xbb,
		0xcf,
		0xe3,
		0xf9,
		0x10e,
		0x125,
		0x13c,
		0x153,
		0x16b,
		0x183,
		0x19c,
		0x1b4,
		0x1cd,
		0x1e6};

uint32_t saw_LUT[NS] = {
		0x000,
		0x008,
		0x010,
		0x018,
		0x020,
		0x028,
		0x030,
		0x038,
		0x040,
		0x048,
		0x050,
		0x058,
		0x060,
		0x068,
		0x070,
		0x078,
		0x080,
		0x088,
		0x090,
		0x098,
		0x0a0,
		0x0a8,
		0x0b0,
		0x0b8,
		0x0c0,
		0x0c8,
		0x0d0,
		0x0d8,
		0x0e0,
		0x0e8,
		0x0f0,
		0x0f8,
		0x100,
		0x108,
		0x110,
		0x118,
		0x120,
		0x128,
		0x130,
		0x138,
		0x140,
		0x148,
		0x150,
		0x158,
		0x160,
		0x168,
		0x170,
		0x178,
		0x180,
		0x188,
		0x190,
		0x198,
		0x1a0,
		0x1a8,
		0x1b0,
		0x1b8,
		0x1c0,
		0x1c8,
		0x1d0,
		0x1d8,
		0x1e0,
		0x1e8,
		0x1f0,
		0x1f8,
		0x200,
		0x208,
		0x210,
		0x218,
		0x220,
		0x228,
		0x230,
		0x238,
		0x240,
		0x248,
		0x250,
		0x258,
		0x260,
		0x268,
		0x270,
		0x278,
		0x280,
		0x288,
		0x290,
		0x298,
		0x2a0,
		0x2a8,
		0x2b0,
		0x2b8,
		0x2c0,
		0x2c8,
		0x2d0,
		0x2d8,
		0x2e0,
		0x2e8,
		0x2f0,
		0x2f8,
		0x300,
		0x308,
		0x310,
		0x318,
		0x320,
		0x328,
		0x330,
		0x338,
		0x340,
		0x348,
		0x350,
		0x358,
		0x360,
		0x368,
		0x370,
		0x378,
		0x380,
		0x388,
		0x390,
		0x398,
		0x3a0,
		0x3a8,
		0x3b0,
		0x3b8,
		0x3c0,
		0x3c8,
		0x3d0,
		0x3d8,
		0x3e0,
		0x3e8,
		0x3f0,
		0x3f8};

uint32_t triangle_LUT[NS] = {
		0x10,
		0x20,
		0x30,
		0x40,
		0x50,
		0x60,
		0x70,
		0x80,
		0x90,
		0xa0,
		0xb0,
		0xc0,
		0xd0,
		0xe0,
		0xf0,
		0x100,
		0x110,
		0x120,
		0x130,
		0x140,
		0x150,
		0x160,
		0x170,
		0x180,
		0x190,
		0x1a0,
		0x1b0,
		0x1c0,
		0x1d0,
		0x1e0,
		0x1f0,
		0x200,
		0x20f,
		0x21f,
		0x22f,
		0x23f,
		0x24f,
		0x25f,
		0x26f,
		0x27f,
		0x28f,
		0x29f,
		0x2af,
		0x2bf,
		0x2cf,
		0x2df,
		0x2ef,
		0x2ff,
		0x30f,
		0x31f,
		0x32f,
		0x33f,
		0x34f,
		0x35f,
		0x36f,
		0x37f,
		0x38f,
		0x39f,
		0x3af,
		0x3bf,
		0x3cf,
		0x3df,
		0x3ef,
		0x3ff,
		0x3ef,
		0x3df,
		0x3cf,
		0x3bf,
		0x3af,
		0x39f,
		0x38f,
		0x37f,
		0x36f,
		0x35f,
		0x34f,
		0x33f,
		0x32f,
		0x31f,
		0x30f,
		0x2ff,
		0x2ef,
		0x2df,
		0x2cf,
		0x2bf,
		0x2af,
		0x29f,
		0x28f,
		0x27f,
		0x26f,
		0x25f,
		0x24f,
		0x23f,
		0x22f,
		0x21f,
		0x20f,
		0x200,
		0x1f0,
		0x1e0,
		0x1d0,
		0x1c0,
		0x1b0,
		0x1a0,
		0x190,
		0x180,
		0x170,
		0x160,
		0x150,
		0x140,
		0x130,
		0x120,
		0x110,
		0x100,
		0xf0,
		0xe0,
		0xd0,
		0xc0,
		0xb0,
		0xa0,
		0x90,
		0x80,
		0x70,
		0x60,
		0x50,
		0x40,
		0x30,
		0x20,
		0x10,
		0x0};

char SIN[] = "Sine";
char TRI[] = "Triangle";
char SAW[] = "Saw";

char* waveNames[] = {SIN, TRI, SAW};
uint32_t* waveLUT[] = {Sin_LUT, triangle_LUT, saw_LUT};

uint32_t nextIdx = TRIIDX;
uint32_t curr_millis = 0;
uint32_t prev_millis = 0;

// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = 7800;//(1/TIM2CLK)+1; // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
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
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  HAL_DMA_Start_IT(&hdma_tim2_ch1, Sin_LUT, DestAddress, 128);

  // TODO: Write current waveform to LCD ("Sine")
  delay(3000);
  lcd_putstring(SIN);

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	// TODO: Debounce using HAL_GetTick()
	curr_millis = HAL_GetTick();

			if((curr_millis-prev_millis)>300){
				prev_millis = curr_millis;
				lcd_command(CLEAR);
				lcd_putstring(waveNames[nextIdx]);

				// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
				// HINT: Consider using C's "switch" function to handle LUT changes
				__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
				HAL_DMA_Abort_IT(&hdma_tim2_ch1);
				HAL_DMA_Start_IT(&hdma_tim2_ch1, waveLUT[nextIdx++], DestAddress, 128);
				__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);


				if(nextIdx==SAWIDX+1){
					nextIdx=SINIDX;
					}
				}



	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
