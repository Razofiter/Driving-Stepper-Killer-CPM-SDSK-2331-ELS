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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

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
TIM_HandleTypeDef htim1;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// Constant char that specifies the motor index/nr
const uint8_t user_motNr = '1';

// Variable that specifies the motor mode of OPERATION
uint8_t USER_VARIABLE_VEL_MODE = TRUE;

double user_acc_progression_time_increment = 0;
double user_dec_progression_time_increment = 0;

// This variable specifies the type of the received message
//uint8_t user_received_flag_USB_msg = FLAG_UNKNOWN;

// Variable that holds the conversion value from distance in mmm to motor steps
double user_mot_revolutions = 0;

// Variable that holds the acceleration time
double user_acc_time = 0;

// Flag to update the RCR of timer 1
uint8_t user_flag_update_RCR = FALSE;

// Flags to account for actions taken by the user regarding, the commanded steps, distance, vel, angular vel,
// elapsed time, time interval, acc_time, dec_time to be performed
uint8_t user_flag_steps_to_be_performed_SET = FALSE,
				user_flag_distance_to_be_performed_SET = FALSE,
				user_flag_mot_RPM_SET = FALSE,
				user_flag_load_vel_SET = FALSE,
				user_flag_elapsed_time_SET = FALSE,
				user_flag_time_interval_SET = FALSE,
				user_flag_acc_time_SET = FALSE,
				user_flag_dec_time_SET = FALSE,
				user_flag_trajectory_repetitions_SET = FALSE;
				
// Flag to account for the update of the timer registers
volatile uint8_t user_flag_TIMER_registers_update_SET = FALSE,							
								 user_flag_mot_START = FALSE;

// Variable that holds the number of the capture compare interrups
volatile uint32_t user_no_of_cc_interrupts = 0;
volatile uint32_t user_no_of_pwm_periods_until_change_when_acc = 0;
volatile uint32_t user_time_increments_added_to_pwm_period_when_acc = 0;

volatile uint32_t user_no_of_pwm_periods_until_change_when_dec = 0;
volatile uint32_t user_time_increments_added_to_pwm_period_when_dec = 0;

// This variables will be used in conjontion with the RCR (timer Repetition Counter Register)
uint32_t user_number_of_RCR_overflows = 0;
uint8_t user_remaining_RCR_val = 0;

// Var that is not used yet
//uint8_t user_maxStepNr = MAX_RCR_VAL;

// Var that holds the total number of performed steps and distance
int32_t user_total_steps_performed = 0;
double user_total_distance_performed = 0;

// Var that is used to compute how long takes for one mechanical rotation after the motor rezolution will be set
// From the above variable we can compute the pwm period and frequency. This will be done later in the code
double user_one_rotation_time = 0.0;
uint32_t user_pwm_period = 0, user_max_pwm_period =0;
uint32_t user_pwm_frequency = 0;

// Array of possible motor step resolutions
uint32_t user_globalStepRes[9] = {STEPS_PER_REV_200,
														 STEPS_PER_REV_400,
														 STEPS_PER_REV_800,
														 STEPS_PER_REV_1600,
														 STEPS_PER_REV_3200,
														 STEPS_PER_REV_6400,
														 STEPS_PER_REV_12800,
														 STEPS_PER_REV_25600,
														 STEPS_PER_REV_51200};

// Variables that hold the minimum accepted period of the PWM signal, which is set according to the USER selected motor rezolution
uint16_t user_pwm_min_period[9] = {PWM_MIN_PERIOD_AT_200_US,
																	PWM_MIN_PERIOD_AT_400_US,
																	PWM_MIN_PERIOD_AT_800_US,
																	PWM_MIN_PERIOD_AT_1600_US,
																	PWM_MIN_PERIOD_AT_3200_US,
																	PWM_MIN_PERIOD_AT_6400_US,
																	PWM_MIN_PERIOD_AT_12800_US,
																	PWM_MIN_PERIOD_AT_25600_US,
																	PWM_MIN_PERIOD_AT_51200_US};
				 
// Variables that hold the motor step rezolution, axis length, screw step, and mass of the load, set by the USER
uint32_t user_set_mot_rez = 0;
uint16_t user_set_axis_len = 0, user_set_load_mass = 0;
uint8_t user_set_screw_step = 0;
														 
/* Variables that hod the comand parameteres of the comand messages */
uint32_t user_set_steps = 0,
					user_set_acc_steps = 0,
					user_set_dec_steps = 0,
					guser_set_acc_steps = 0,
					guser_set_dec_steps = 0;
volatile uint32_t user_set_trajectory_repetitions = 0;
														 
double user_set_distance = 0.0;
uint16_t Iuser_set_distance = 0;
uint8_t Fuser_set_distance = 0;	

double user_mot_max_RPM = 0.0;
double user_set_motor_RPM = 0.0, user_set_motor_current_RPM = 0;
uint16_t Iuser_set_motor_RPM = 0;
uint8_t Fuser_set_motor_RPM = 0;
																	
double user_set_motor_RPM_s = 0.0;
uint16_t Iuser_set_motor_RPM_s = 0;
uint8_t Fuser_set_motor_RPM_s = 0;

double user_load_max_vel = 0.0;
double user_set_load_vel = 0.0;
uint16_t Iuser_set_load_vel = 0;
uint8_t Fuser_set_load_vel = 0;	

double user_set_time_elapsed = 0;
uint16_t Fuser_set_time_elapsed = 0;
uint16_t Iuser_set_time_elapsed = 0;

double user_set_time_interval = 0;
uint16_t Fuser_set_time_interval = 0;
uint16_t Iuser_set_time_interval = 0;

volatile double user_set_motor_acc_time = 0.0;
uint8_t Iuser_set_motor_acc_time = 0;
uint16_t Fuser_set_motor_acc_time = 0;

volatile double user_set_motor_dec_time = 0.0;
uint8_t Iuser_set_motor_dec_time = 0;
uint16_t Fuser_set_motor_dec_time = 0;

/* Variable that holds the desired direction of the motor	*/
uint8_t user_set_mot_direction = 0;

// Fixed messages that are not used yet
//const uint8_t mot_start[]="@M1START\r";
//const uint8_t mot_stop[]="@M1STOP\r";
//const uint8_t okMsg[5]="#EOK\r";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_WWDG_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void USER_Clear_Array(uint8_t* array);
static uint8_t USER_Decode_Msg(void);
static uint8_t USER_ExtractNumber_FromString(uint32_t * Destination,
											 uint8_t * String,
											 uint8_t StartPosition,
											 uint8_t Length);

static void USER_Set_Mot_Dir(uint8_t direction);
static void USER_Set_Load_Dir(uint8_t direction);
static uint32_t USER_Compute_Pwm_Period(double motor_RPM);
//static double USER_Compute_Progression_Time_Step(double motor_RPM_start, double motor_RPM_stop, uint32_t pwm_period, double acc_time);
static double USER_Compute_Progression_Time_Step(double motor_RPM, uint32_t pwm_period, double acc_time);
static void USER_Start_Motor(void);
static void USER_Stop_Motor(void);
static void USER_Move_Motor(void);
static void USER_Variable_Vel_TIM_Config(void);
static void USER_Constant_Vel_TIM_Config(void);
/* USER CODE END PFP */

uint16_t user_pwm_period_array[ROWS][COLUMNS] = {{567,452,389,348,318,294,276,261,248,237,227,219,211,204,198,193,187,183,178,174,171,167,164,161,158,155,153,150,148,146,144,142,140,138,136,135,133,132,130,129,128,126,125,124,123,121,120,119,118,117,116,115,115,114,113,112,111,110,110,109,108,107,107,106,106,105,104,104,103,103,102,101,101,100,100,99,99,98,98,97,97,97,96,96,95,95,95,94,94,93,93,93,92,92,92,91,91,91,91,90,90,90,89,89,89,89,88,88,88,88,87,87,87,87,86,86,86,86,86,85,85,85,85,85,84,84,84,84,84,83,83,83,83,83,83,83,82,82,82,82,82,82,82,81,81,81,81,81,81,81,81,81,80,80,80,80,80,80,80,80,80,80,80,80,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,79,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78}};

volatile uint16_t row = 0;
volatile int16_t col = 0;
volatile uint8_t flag_change_direction = FALSE;
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
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  //MX_WWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	// Start the motor
	HAL_GPIO_WritePin(M3__EN_GPIO_Port,M3__EN_Pin,GPIO_PIN_RESET);
	
	HAL_Delay(20);
  /* Infinite loop */
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		if (flag_change_direction)
//		{
//			if(htim1.Instance->CNT == 
//		}
//		switch(user_received_flag_USB_msg)
//		{
//			case FLAG_UNKNOWN:
//				break;
//					
//			case FLAG_OK:
//				if(USER_Decode_Msg())
//				{
//					user_received_flag_USB_msg = FLAG_UNKNOWN;
//				}
//				else
//				{
//					CDC_Transmit_FS((uint8_t*)"#ERR\r",5);
//					user_received_flag_USB_msg = FLAG_UNKNOWN;
//				}
//				break;
//			
//			case FLAG_OVERFLOW:
//				break;
//			
//			case FLAG_ERR:
//				break;
//		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = user_pwm_period_array[row][col];
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (user_pwm_period_array[row][col]+1)/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_4;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_LED_BLUE_GPIO_Port, PWR_LED_BLUE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BAT_LOW_LVL_GPIO_Port, BAT_LOW_LVL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M3__EN_Pin|M2__EN_Pin|M1__EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M3_ROT_Pin|M2_ROT_Pin|M1_ROT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PWR_LED_BLUE_Pin */
  GPIO_InitStruct.Pin = PWR_LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 PA3 PA5 
                           PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VBAT_Pin */
  GPIO_InitStruct.Pin = VBAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_LOW_LVL_Pin */
  GPIO_InitStruct.Pin = BAT_LOW_LVL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAT_LOW_LVL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB15 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : M3__EN_Pin M2__EN_Pin M1__EN_Pin */
  GPIO_InitStruct.Pin = M3__EN_Pin|M2__EN_Pin|M1__EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HLFB3_Pin */
  GPIO_InitStruct.Pin = HLFB3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HLFB3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HLFB2_Pin HLFB1_Pin */
  GPIO_InitStruct.Pin = HLFB2_Pin|HLFB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_ROT_Pin M2_ROT_Pin M1_ROT_Pin */
  GPIO_InitStruct.Pin = M3_ROT_Pin|M2_ROT_Pin|M1_ROT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
