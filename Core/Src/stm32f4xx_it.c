/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"

extern int pulse;			// asserts values from 0 to 8000

// CAN Variables
extern uint8_t TxData[8];					// Data to be sent via CAN
extern uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
extern CAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages

extern uint8_t RxData[8];		// data received from can bus
extern CAN_RxHeaderTypeDef   RxHeader;		// header received by can bus

//extern const uint8_t MY_CAN_ID;
//extern const int MY_MOTOR_PWM_PERIOD ;	// Maximum timer value of the motor PWM
extern int odom_prev;	// previous odom measurement
extern int odom_curr;	// last odom measurement
extern int odom_velocity, odom_prev_velocity, odom_acceleration;	// acceleration is the difference between previous and current velocity
extern int odom_target_velocity;	// desired velocity
extern float odom_rs_velocity;	// current velocity in rad/s




extern int ID_Accepted;	// flag. =1 if at least a single CAN message with this device's ID has been received



void My_CAN_Rx_Handler( CAN_RxHeaderTypeDef* RxHeader, uint8_t* RxData);	// Executes when CAN message is received
int My_PD_Controller(int velocity, int acceleration, int target_velocity, int prev_pd_val);	// calculates PD value
int Velocity_To_CAN_Data(float velocity, uint8_t* TxData);	// Converts vellcity in rad/s to CAN message
float velocity_ticks_to_rs(int velocity_ticks);	// converts velocity from encoder ticks to rad/s
int velocity_rs_to_ticks(float velocity_rs);	// converts velocity from rad/s to encoder ticks
extern int Motor_Get_Odom();			// get odom counter value

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim7;


int PD_Value = 0;
int flag = 0;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */

// executes at CAN message reception
void CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);	// reads CAN message
	My_CAN_Rx_Handler(&RxHeader, RxData);	// My handler function
	HAL_CAN_IRQHandler(&hcan1);
}

extern uint8_t RxData[8];		// data received from can bus
extern CAN_RxHeaderTypeDef   RxHeader;

void My_CAN_Rx_Handler( CAN_RxHeaderTypeDef* RxHeader, uint8_t* RxData){
	if(RxHeader->StdId == MY_CAN_ID){	// checks received ID against this device's ID
		ID_Accepted = 1;
		if(RxData[0] == 0){		// first byte is the message type. 0 required for motor control.
			float* velocity_rs = RxData + 1;
		//	float vel_rs = -0.1;
		//	RxData  = &vel_rs;
			flag = 1;
			float temp = *velocity_rs;
			flag = 2;
			odom_target_velocity = velocity_rs_to_ticks(temp);
			flag = odom_target_velocity;
		}
	}
	return;
}


void TIM7_IRQHandler(void)
{
	// PD controller calculations
	odom_prev = odom_curr;
	odom_curr = Motor_Get_Odom();
	// checks if encoder counter hasn't been reset between ticks
	if(odom_curr - odom_prev < 10000 && odom_curr - odom_prev > -10000){
		odom_prev_velocity = odom_velocity;
		odom_velocity = (odom_curr - odom_prev ) % (ENCODER_PERIOD);
		odom_acceleration = odom_velocity - odom_prev_velocity;
	}
	PD_Value = My_PD_Controller(odom_velocity, odom_acceleration, odom_target_velocity, PD_Value);
	pulse = (PD_Value) * MY_MOTOR_PWM_PERIOD / 255 + MY_MOTOR_PWM_PERIOD/2;

	odom_rs_velocity = velocity_ticks_to_rs(odom_velocity);
	Velocity_To_CAN_Data(odom_rs_velocity, TxData);
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0){
		if ( HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			Error_Handler();
		}
	}
	HAL_TIM_IRQHandler(&htim7);
}

int My_PD_Controller(int velocity, int acceleration, int target_velocity,  int prev_pd_val){
	double k1 = 0.5, k2 = -0.05;
	if(velocity < -200){
		velocity = -200;
	}
	if(velocity > 200){
		velocity = 200;
	}

	int res = prev_pd_val + ( target_velocity - velocity)*k1  + acceleration * k2 ;  /* (255.0 / (float)(abs(velocity) + 10))*/
	if(res > 127){
		res = 127;
	}
	if(res < -127){
		res = -127;
	}
	return res;
}


int Velocity_To_CAN_Data(float velocity, uint8_t* TxData){
	char* temp_char = &velocity;
	TxData[0] = temp_char[0];
	TxData[1] = temp_char[1];
	TxData[2] = temp_char[2];
	TxData[3] = temp_char[3];
	return 1;
}

float velocity_ticks_to_rs(int velocity_ticks){
	return (float)velocity_ticks * USER_GENERAL_PURPOSE_TIMER_FREQUENCY / ODOM_TICKS_PER_RADIAN;
	// multiplied by frequency of the timer to get speed in a second and divided by "ticks for 1 radian" coefficient.
}

int velocity_rs_to_ticks(float velocity_rs){
	return  (int)(velocity_rs * ODOM_TICKS_PER_RADIAN / USER_GENERAL_PURPOSE_TIMER_FREQUENCY);
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
