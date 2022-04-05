/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
__fp16 f16_tester = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// CAN Variables
extern uint8_t TxData[8];					// Data to be sent via CAN
extern uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
extern CAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages

extern uint8_t RxData[8];					// data received from can bus
extern CAN_RxHeaderTypeDef   RxHeader;		// header received by can bus

//CAlled upon CAN message recption
void My_CAN_Rx_Handler( CAN_RxHeaderTypeDef* RxHeader, uint8_t* RxData);	// Executes when CAN message is received

// encoder current and previous angle
extern float angle_rad;
float angle_rad_prev = 0;
extern float target_velocity;	// targeted velocity
extern float target_angle; 		//target angle
extern float velocity;			//current velocity


extern char goal_status;		// set to 1  if the last goal has been reached.
extern char stall_status; 	// set to 1  if the stepper stall has been detected.

//denominator for stepper step.
extern int stepper_step_denominator;

//just a bunch of flags for the debugger
extern int flag;
extern int flag_tim7;
extern int flag_tim2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

extern void CTRL_Reg_Set();

float read_angle();	// reads current encoder angle (converted to rad)

float Ticks_To_Angle(int ticks);		// convert encoder ticks to absolute angle
int Angle_To_CAN_Data(float angle, float speed, char goal_status, char stall_status ,uint8_t* TxData); // convert angle to CAN message
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//extern system variables
extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

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
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);	// reads CAN message
	My_CAN_Rx_Handler(&RxHeader, RxData);	// My handler function
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	flag_tim2++;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	// read current angle from the encoder
	int timer_period = 0, target_velocity_ticks, flag_2;
	angle_rad_prev = angle_rad;
	angle_rad = read_angle();
	// calculate velocity
	if(abs(angle_rad_prev - angle_rad) < M_PI ){
		velocity = (angle_rad - angle_rad_prev) * 20;
	}
	// change direction based on required motor velocity
	if(target_velocity < 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	}
	// for some reason 0 and nan are the same value in IEEE float16, so we have to do this check.
	if(isnan(target_velocity)){
		target_velocity = 0.0;
	}
	// calculate target velocity in ticks/s with current step value
	float temp1 =  target_velocity > 0 ? target_velocity : (-1) * target_velocity;
	float temp2 =  angle_rad - target_angle > 0 ? angle_rad - target_angle : (-1) * (angle_rad - target_angle);
	if(temp2 * 17.0  < sqrt(temp1) || temp1 < 0.001 && temp2 < 0.1){
//		target_velocity_ticks = (target_velocity > 0 ? target_velocity : -1 * target_velocity)  * stepper_step_denominator * 100.0 * GEAR_RATIO/ M_PI;
		target_velocity_ticks = 0 ;
		target_velocity = 0;
		goal_status = 1;
	}
	else{
		goal_status = 0;
		target_velocity_ticks = (target_velocity > 0 ? target_velocity : -1 * target_velocity)  * stepper_step_denominator * 100.0 * GEAR_RATIO/ M_PI;
	}
// 	convert and send CAN message
	Angle_To_CAN_Data(angle_rad, velocity, goal_status, stall_status, TxData);
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0){
//		flag_tim7++;
		if ( HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			Error_Handler();
		}
	}
	flag_tim7++;
	flag_2 = 0;
	// adjust step value to the is tested limits. Needed to avoid vibration on lower and higher speeds
	while(1){
		flag++ ;
		if(target_velocity_ticks > 3000 && stepper_step_denominator > 1){
			stepper_step_denominator /=2;
			target_velocity_ticks /= 2;
		}
		else{
			if(target_velocity_ticks < 1500 && stepper_step_denominator < 256){
				stepper_step_denominator *= 2;
				target_velocity_ticks *= 2;
			}
			else{
				break;
			}
		}
		flag_2= 1;
	}

	// write modified step to the registry if it was modified
	if(flag_2 != 0){
		CTRL_Reg_Set();
	}
	// avoid dividing by zero in the next line
	if (target_velocity_ticks == 0){
		TIM3->CCR1 =  0;
	}
	// calculates and sets timer period to achieve the required speed, avoids too low or too high values
	else{
		timer_period = APB1_TIMER_CLOCK_FREQUENCY/((STEPPER_PWM_PRESCALER + 1) * target_velocity_ticks); // clock frequency/(prescaler * gear_ratio * speed_ticks/s)
//		flag_tim7++;
		if(timer_period > 65535){
			TIM3->CCR1 =  0;
		}
		if(timer_period < 999){
			TIM3->ARR = 999;
			TIM3->CCR1 =  (TIM3->ARR + 1)/2;
		}

		if(timer_period >= 999 && timer_period <= 65535){
			TIM3->ARR = timer_period;
			TIM3->CCR1 =  (TIM3->ARR + 1)/2;
		}
	}

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void My_CAN_Rx_Handler( CAN_RxHeaderTypeDef* RxHeader, uint8_t* RxData){
	flag_tim7 = 0;
	if(RxHeader->StdId == MY_CAN_ID){	// checks received ID against this device's ID
		// what to do
		__fp16* temp_float16 = RxData;
//		float* temp = RxData;
		target_angle = *temp_float16;

		f16_tester = *temp_float16;

		temp_float16 = RxData + 2;


		target_velocity = *temp_float16;
		goal_status = 0;
//		ticks_to_make = Angle_To_Ticks(target_angle_rad);	// change target ticks value
	}
	return;
}

float read_angle(){
	HAL_StatusTypeDef ret;
	uint8_t buf[12];	// temporaty buffer for i2c message transmition
	uint8_t buf_recv[12];	// temporaty buffer for i2c message reception
	buf[0] = 0x0E;
	ret = HAL_I2C_Master_Transmit(&hi2c1, (0b0110110 << 1) , buf, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK ){
//		flag ++;
	}
	ret = HAL_I2C_Master_Receive(&hi2c1, 0b0110110 << 1, buf_recv, 2, HAL_MAX_DELAY);
	if(ret == HAL_OK ){
//		flag --;
	}
	angle_rad = Ticks_To_Angle(buf_recv[1] + buf_recv[0] * 256);
	return angle_rad;
}


float Ticks_To_Angle(int ticks){
	return  (ticks)* M_PI * 2.0  / 4096.0;
}

int Angle_To_CAN_Data(float angle, float speed, char goal_status, char stall_status ,uint8_t* TxData){
	// pack angle
	__fp16 temp_float16 = angle;
	char* temp_char = &temp_float16;
	TxData[0] = temp_char[0];
	TxData[1] = temp_char[1];
	//pack velocity
	temp_float16 = speed;
	temp_char = &temp_float16;
	TxData[2] = temp_char[2];
	TxData[3] = temp_char[3];
	//pack goal_status. 0 if goal not reached, 1 if it is reached.
	TxData[4] = goal_status;
	//pack stall_status. 0 if the motor has not stalled, 1 if it has.
	TxData[5] = stall_status;
/*	temp_char = &speed;
	TxData[4] = temp_char[0];
	TxData[5] = temp_char[1];
	TxData[6] = temp_char[2];
	TxData[7] = temp_char[3];
*/
	return 1;
}

/* USER CODE END 1 */

