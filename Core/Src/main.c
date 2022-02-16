/* USER CODE BEGIN Header */
/* This software has been designed for driving an electric motor.
 * Its intended to be used only on Voltbro hardware.  STM32F446RETx
 * Capabilities:
 * 1. Controlling motor speed via PWM.
 * 2. Reading motor's real speed via an encoder.
 * 3. Receiving the desired speed in rad/s via CAN and using a PD controller to reach it.
 * 4. Transmitting current speed in rad/s via CAN bus.
 * Most of the functions are bound to TIM7 timer. That allows for accurate speed measurements and periodic
 * CAN transmissions. By default TIM7 runs at 20Hz.
 * All environmental variables can be changed in main.h, as they're defined there.
 *
 * CAN MESSAGE STRUCTURE:
 * Default CAN ID - 10
 * byte 0 : message type, set to 0 for motor control. No other functions ar implemented so far.
 * byte 1,2,3,4 - speed, floating point value.
 * byte 5,6,7 - not used.
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


int Motor_PWM_Start();	// Starts PWM timers necessary for motor control
int Motor_Set_Speed(int pulse);	// sets motor speed. Pulse is a value in range [0, max PWM counter value]. Half the period is stop.
int Motor_Get_Odom();	// returns current value of our odometru counter
int CAN_Header_Config(CAN_TxHeaderTypeDef* TxHeader);	// Configures CAN message header.
int CAN_Filter_Config(CAN_FilterTypeDef* canfilterconfig);	// COnfigures filter for CAN message reception
int CAN_Starter(CAN_HandleTypeDef* hcan,CAN_FilterTypeDef* canfilterconfig);	//Starts CAN transmittions and receptions

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);

int pulse = 4500;			// asserts values from 0 to 8000. Controls motor speed.

uint8_t RxData[8];		// data received from can bus
CAN_RxHeaderTypeDef   RxHeader;		// header received by can bus

int ID_Accepted = 0;				// Flag set to 1 if correct
// Odometry counters. Previous and current read. Velocity is a difference between prev and curr
int odom_prev = 0, odom_curr = 0, odom_velocity = 0, odom_prev_velocity = 0, odom_acceleration = 0;
int odom_target_velocity = 150;	//desired velocity in
float odom_rs_velocity = 0;


// CAN Variables
uint8_t TxData[8];					// Data to be sent via CAN
uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
CAN_TxHeaderTypeDef TxHeader;		// Header for can message
CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages

int main(void)
{
	int currCounter = 0;	// odometry counter

	CAN_Header_Config(&TxHeader);		// Sets header values
	CAN_Filter_Config(&canfilterconfig);// Seds filter values

	HAL_Init();


	SystemClock_Config();


	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_TIM8_Init();
	MX_TIM3_Init();
	MX_TIM7_Init();


	CAN_Starter(&hcan1, &canfilterconfig);
	Motor_PWM_Start();
	while (1){
		Motor_Set_Speed(pulse);

	}
}

int Motor_PWM_Start(){
	HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL_1);
	HAL_TIM_PWM_Start(&MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&ENCODER_TIMER, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&USER_GENERAL_PURPOSE_TIMER);
	HAL_GPIO_WritePin(SLEEP_PIN_GROUP, SLEEP_PIN_NUMBER, GPIO_PIN_SET);
	HAL_GPIO_WritePin(VREF_PIN_GROUP, VREF_PIN_NUMBER, GPIO_PIN_SET);
	return 1;
}

int Motor_Set_Speed(int pulse){
	__HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL_1, pulse);
	__HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL_2, pulse);
	return 1;
}

int Motor_Get_Odom(){
//	return TIM3->CNT;
	return __HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
}

int CAN_Header_Config(CAN_TxHeaderTypeDef* TxHeader){
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = MY_CAN_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = 8;
	return 1;
}

int CAN_Filter_Config(CAN_FilterTypeDef* canfilterconfig){
	canfilterconfig->FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig->FilterBank = 0;  // which filter bank to use from the assigned ones
	canfilterconfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig->FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig->FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig->FilterIdHigh = 0x0000;
	canfilterconfig->FilterIdLow = 0x0000;
	canfilterconfig->FilterMaskIdHigh = 0x0000;
	canfilterconfig->FilterMaskIdLow = 0x0000;
	canfilterconfig->SlaveStartFilterBank = 20;
	return 1;
}

int CAN_Starter(CAN_HandleTypeDef* hcan,CAN_FilterTypeDef* canfilterconfig){
	if (HAL_CAN_ConfigFilter(hcan, canfilterconfig) != HAL_OK){		// attempting to register can filter
			Error_Handler();
		}
		if (HAL_CAN_Start(hcan) != HAL_OK){		// attempting to start CAN
			Error_Handler();
		}
		if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){	// attempting to enable CAN interrupts.
			Error_Handler();																// CAN1_RX0_IRQHandler() is called when an interrupt is triggered
		}
	return 1;
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 249;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 15999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 3500;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
