/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
  /* USER CODE END Header */

  /* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define kk 0.0000596
uint32_t led_time = 500;
uint32_t countled = 0;
bool flag_led = false;
bool flag_signal = false;
uint32_t duty = 0;

uint8_t aRxBuffer;			//接收中断缓冲

uint8_t a, b, c = 0;
float color1, color2, color3 = 0;

uint8_t signal_num = 0;
uint8_t signal_quality = 0;
int32_t signal_delta = 0;
int32_t signal_theta = 0;
int32_t signal_low_alpha = 0;
int32_t signal_high_alpha = 0;
int32_t signal_low_beta = 0;
int32_t signal_high_beta = 0;
int32_t signal_low_gamma = 0;
int32_t signal_mid_gamma = 0;

int32_t signalb_delta = 0;
int32_t signalb_theta = 0;
int32_t signalb_low_alpha = 0;
int32_t signalb_high_alpha = 0;
int32_t signalb_low_beta = 0;
int32_t signalb_high_beta = 0;
int32_t signalb_low_gamma = 0;
int32_t signalb_mid_gamma = 0;

int fifo_in(uint8_t);
uint8_t fifo_out();
typedef struct node_st {
	uint8_t data;
	struct node_st* next, * prev;
} node_t;

typedef struct {
	node_t head, end;
	int size;
} llist_t;

llist_t llist;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
void llist_init();
uint8_t fifo_out();
int fifo_in(uint8_t data);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
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
	llist_init();
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	SysTick_Config(SystemCoreClock / 1000);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_TIM14_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*)& aRxBuffer, 1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	fifo_in(0x00);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (flag_signal)
		{
			flag_signal = false;
			signalb_delta = signal_delta;
			signalb_theta = signal_theta;
			signalb_low_alpha = signal_low_alpha;
			signalb_high_alpha = signal_high_alpha;
			signalb_low_beta = signal_low_beta;
			signalb_high_beta = signal_high_beta;
			signalb_low_gamma = signal_low_gamma;
			signalb_mid_gamma = signal_mid_gamma;

			signal_quality = fifo_out();
			fifo_out(); fifo_out(); fifo_out();
			a = fifo_out();b = fifo_out();c = fifo_out();
			//16???????
			signal_delta = (a << 16) | (b << 8) | c;
			a = fifo_out(); b = fifo_out(); c = fifo_out(); 
			signal_theta = (a << 16) | (b << 8) | c;
			a = fifo_out(); b = fifo_out(); c = fifo_out();
			signal_low_alpha = (a << 16) | (b << 8) | c;
			a = fifo_out(); b = fifo_out(); c = fifo_out();
			signal_high_alpha = (a << 16) | (b << 8) | c;
			a = fifo_out(); b = fifo_out(); c = fifo_out();
			signal_low_beta = (a << 16) | (b << 8) | c;
			a = fifo_out(); b = fifo_out(); c = fifo_out();
			signal_high_beta = (a << 16) | (b << 8) | c;
			a = fifo_out(); b = fifo_out(); c = fifo_out();
			signal_low_gamma = (a << 16) | (b << 8) | c;
			a = fifo_out(); b = fifo_out(); c = fifo_out();
			signal_mid_gamma = (a << 16) | (b << 8) | c;
			a = fifo_out(); b = fifo_out(); c = fifo_out();
                        duty = 0;
		}

		if (flag_led)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			flag_led = false;
		}
		if (duty <= 999)
		{
			//color1 = (signalb_low_alpha + 0.001 * duty * (signalb_low_alpha - signal_low_alpha)) * 0.00000596;
                        color1 = signalb_low_alpha * kk + 0.001 * duty * (signalb_low_alpha * kk - signal_low_alpha * kk);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, color1);
                        //color2 = (signalb_low_beta + 0.001 * duty * (signalb_low_beta - signal_low_beta)) * 0.00000596;
                        color2 = signalb_low_beta * kk + 0.001 * duty * (signalb_low_beta * kk - signal_low_beta * kk);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, color2);
                        //color3 = (signalb_low_gamma + 0.001 * duty * (signalb_low_gamma - signal_low_gamma)) * 0.00000596;
                        color3 = signalb_low_gamma * kk + 0.001 * duty * (signalb_low_gamma * kk - signal_low_gamma * kk);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, color3);
		}
		/*
		if (duty <= 999)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
		}
		else if ((duty <= (999 * 2)) && (duty > 999))
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (999 - (duty - 999)));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
		}
		else if ((duty <= (999 * 3)) && (duty > (999 * 2)))
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty - 999 * 2);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
		}
		else if ((duty <= (999 * 4)) && (duty > (999 * 3)))
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999 - (duty - 999 * 3));
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
		}
		else if ((duty <= (999 * 5)) && (duty > (999 * 4)))
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, duty - 999 * 4);
		}
		else if ((duty <= (999 * 6)) && (duty > (999 * 5)))
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 999 - (duty - 999 * 5));
		}
		*/
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
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

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 47;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 47;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 999;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */
	HAL_TIM_MspPostInit(&htim14);

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
	huart1.Init.BaudRate = 57600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void llist_init() { llist.size = 0; }

int fifo_in(uint8_t data) {
	node_t* nodeptr;
	nodeptr = malloc(sizeof(node_t));
	nodeptr->data = data;
	if (llist.size > 0 && llist.size < 28) {
		nodeptr->next = &llist.end;
		nodeptr->prev = llist.end.prev;
		llist.end.prev->next = nodeptr;
		llist.end.prev = nodeptr;
		llist.size += 1;
	}
	else if (llist.size == 0) {
		nodeptr->prev = &llist.head;
		nodeptr->next = &llist.end;
		llist.head.next = nodeptr;
		llist.end.prev = nodeptr;
		llist.size += 1;
	}
	else if (llist.size > 27) {
		fifo_out();
		nodeptr->next = &llist.end;
		nodeptr->prev = llist.end.prev;
		llist.end.prev->next = nodeptr;
		llist.end.prev = nodeptr;
		llist.size += 1;
	}
	return 0;
}

uint8_t fifo_out() {
	uint8_t data;
	if (llist.size != 1) {
		data = llist.head.next->data;
		llist.head.next = llist.head.next->next;
		free(llist.head.next->prev);
		llist.head.next->prev = &llist.head;
		llist.size -= 1;
	}
	else if (llist.size == 0) {
		return 0;
	}
	else if (llist.size == 1) {
		return 255;
	}
	return data;
}

void SysTick_Handler()
{
	if (!flag_led)countled++;
	duty++;
	//if (duty > 999 * 6)duty = 0;
	//if (duty > 999)duty = 0;
	if (countled >= led_time)
	{
		countled = 0;
		flag_led = true;
	}
}

void USART1_IRQHandler(void)
{
	/* USER CODE BEGIN USART1_IRQn 0 */

	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */

	/* USER CODE END USART1_IRQn 1 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	uint8_t buff = 0;
	static uint8_t index = 0;
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
	/* NOTE: This function Should not be modified, when the callback is needed,
			 the HAL_UART_TxCpltCallback could be implemented in the user file
	*/
	buff = aRxBuffer;
	switch (index)
	{
	case 0:
	{
		if (buff == 0xAA)
		{
			index = 1;
			goto RTB;
		}
	}
	case 1:
	{
		if (buff == 0xAA)
		{
			index = 2;
			goto RTB;
		}
		else
		{
			index = 0;
			goto RTB;
		}
	}
	case 2:
	{
		if (buff >= 0xAA)
		{
			index = 0;
			goto RTB;
		}
		else
		{
			if (buff == 4)
			{
				index = 0;
				goto RTB;
			}
			else
			{
				index = 3;
				while (llist.size - 1)
				{
					fifo_out();
				}
				goto RTB;
			}
		}
	}
	case 3:
	{
		if (llist.size < 28)
		{
			fifo_in(buff);
			goto RTB;
		}
		else
		{
			index = 0;
			flag_signal = true;
			goto RTB;
		}
	}
	default:goto RTB;
	}
	//HAL_UART_Transmit(&huart1, (uint8_t *)&buff, 1, 0xFFFF); 
RTB:
	HAL_UART_Receive_IT(&huart1, (uint8_t*)& aRxBuffer, 1);
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
void assert_failed(char* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	   /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
