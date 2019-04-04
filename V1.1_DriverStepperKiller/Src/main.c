/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// Constant char that specifies the motor index/nr
const uint8_t user_motNr = '1';
// This variable specifies the type of the received message
uint8_t user_received_flag_USB_msg = FLAG_UNKNOWN;

// Variable that holds the conversion value from distance in mmm to motor steps
double user_mot_revolutions = 0;

// Flag to update the RCR of timer 1
uint8_t user_flag_update_RCR = FALSE;

// Flags to account for actions taken by the user regarding, the commanded steps,distance,vel, angular vel to be performed
uint8_t user_flag_steps_to_be_performed_SET = FALSE,
				user_flag_distance_to_be_performed_SET = FALSE,
				user_flag_mot_RPM_SET = FALSE,
				user_flag_load_vel_SET = FALSE;
				
// Flag to account for the update of the timer registers
volatile uint8_t user_flag_TIMER_registers_update_SET = FALSE;

// This variables will be used in conjontion with the RCR (timer Repetition Counter Register)
//uint8_t user_currentStepNr = 0;
uint32_t user_number_of_RCR_overflows = 0;
uint8_t user_remaining_RCR_val = 0;

// Var that is not used yet
uint8_t user_maxStepNr = MAX_RCR_VAL;

// Var that holds the total number of performed steps and distance
int32_t user_total_steps_performed = 0;
double user_total_distance_performed = 0;

// Var that is used to compute how long takes for one mechanical rotation after the motor rezolution will be set
// From the above variable we can compute the pwm period and frequency. This will be done later in the code
double user_one_rotation_time = 0.0;
uint32_t user_pwm_period = 0;
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
														 
// Variables that hod the comand parameteres of the comand messages
uint32_t user_set_steps = 0;
														 
double user_set_distance = 0.0;
uint16_t Iuser_set_distance = 0;
uint8_t Fuser_set_distance = 0;	

double user_mot_max_RPM = 0.0;
double user_set_motor_RPM = 0.0;
uint16_t Iuser_set_motor_RPM = 0;
uint8_t Fuser_set_motor_RPM = 0;

double user_load_max_vel = 0.0;
double user_set_load_vel = 0.0;
uint16_t Iuser_set_load_vel = 0;
uint8_t Fuser_set_load_vel = 0;														 

// Variable that holds the desired direction of the motor														 
uint8_t user_set_mot_direction = 0;

// Fixed messages that are not used yet
const uint8_t mot_start[]="@M1START\r";
const uint8_t mot_stop[]="@M1STOP\r";
const uint8_t okMsg[5]="#EOK\r";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_WWDG_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

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
static void USER_Compute_Pwm_Period(double motor_RPM);
static void USER_Start_Motor(void);
static void USER_Stop_Motor(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
//  MX_WWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		switch(user_received_flag_USB_msg)
		{
			case FLAG_UNKNOWN:
				break;
					
			case FLAG_OK:
				//CDC_Transmit_FS((uint8_t*)"#EOK\r",5);
				if(USER_Decode_Msg())
				{
					user_received_flag_USB_msg = FLAG_UNKNOWN;
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				}
				else
					CDC_Transmit_FS((uint8_t*)"#ERR\r",5);
				break;
			
			case FLAG_OVERFLOW:
				break;
			
			case FLAG_ERR:
				break;
		}

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == htim1.Instance)
//	{
//		// If we want an update the above flag is FALSE before executing the following line
//		if(!user_flag_TIMER_registers_update_SET)
//		{
//			user_flag_TIMER_registers_update_SET = TRUE;
//			//CDC_Transmit_FS((uint8_t*)"1",1);
//		}
//		// Else, we have a RCR overflow event
//		else
//		{
//			// Here we account for the number RCR overflows
//			if(user_number_of_RCR_overflows)
//			{
//				//CDC_Transmit_FS((uint8_t*)"#2",1);
//				user_number_of_RCR_overflows--;
//				/* Enable the Peripheral */
//				__HAL_TIM_ENABLE(htim);
//			}
//			else
//			{
//				if(user_remaining_RCR_val)
//				{
//					if(user_set_steps>MAX_RCR_VAL)
//					{
//						//CDC_Transmit_FS((uint8_t*)"#3",1);
//						// Because we need to change the value of RCR, an update of TIM1 registres is needed, therefore we change user_flag_TIMER_registers_update_SET to FALSE
//						// and next time the interrupt is fired, we enter the first IF STATEMENT.
//						htim1.Instance->RCR = user_remaining_RCR_val;
//						htim1.Instance->EGR = TIM_EGR_UG;
//						user_flag_TIMER_registers_update_SET = FALSE;
//						user_remaining_RCR_val = 0;
//						/* Enable the Peripheral */
//						__HAL_TIM_ENABLE(htim);
//					}
//					else
//					{
//						//CDC_Transmit_FS((uint8_t*)"4",1);
//						user_remaining_RCR_val = 0;
//						/* Enable the Peripheral */
//						__HAL_TIM_ENABLE(htim);
//					}
//				}
//				else
//				{
//					//CDC_Transmit_FS((uint8_t*)"5",1);
//					// Change the update TIM1 registers flag back to FALSE for the next motor START procedure
//					user_flag_TIMER_registers_update_SET = FALSE;
//					// If this point is reached, that means we finished to perform the number of steps. Therefore, we stop the PWM generation
//					__HAL_TIM_DISABLE(htim);
//					// We finally update the total number of steps that were performed based on the direction
//					if(user_set_mot_direction == ROTATE_CW)
//						user_total_steps_performed += user_set_steps;
//					else
//						user_total_steps_performed -= user_set_steps;
//					
//					//sprintf((char*)user_sendUSBdata,"Current nr of steps: %0d\r", user_total_steps_performed);
//					//CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
//				}
//			}
//		}
//	}
//}


/**
  * @brief  Messages received over USB OUT endpoint are of certain type and are passed to a local variable here (main.c)
  *
  * @param  flag: type of the received message
  * @retval This function does not return any value. It only updates the variable "received_flag_USB_msg"
  */
void USER_Pass_MsgFlag(uint8_t flag)
{
	user_received_flag_USB_msg = flag;
}


/**
  * @brief  Function that extracts a number from a substring
  *
  * @param  Destination: A pointer to an integer value where the string will be converted to a numerical value
	* @param  String: The string message from where the substring(number) is extracted
	* @param  StartPosition: The string message starting position of the substring
	* @param  Length: The substring message length to be converted to a numerical value
  * @retval This function return TRUE if the substring was succesfully converted to a numerical value and FALSE otherwise
  */
static uint8_t USER_ExtractNumber_FromString(uint32_t * Destination,
											 uint8_t * String,
											 uint8_t StartPosition,
											 uint8_t Length)
	{
	uint8_t index;
	uint8_t val;
	(* Destination) = 0;

	for (index = StartPosition; index < StartPosition + Length; index++)
	{
			val = (String[index] - '0');
		if (val<=9)
		{
			(* Destination) *= 10;
			(* Destination) += val;
		}
		else
		{
			return FALSE;
		}
	}

	// If the function doesn't return to this position we can assune that the parameters are ok
	return TRUE;
	}
	

/**
  * @brief  Function that sets the motor direction.
  *
  * @param  direction: the direction can be set to ROTATE_CW (clockwise) or ROTATE_CCW (counterclockwise)
  * @retval This function does not return any value.
  */
	static void USER_Set_Mot_Dir(uint8_t direction)
{
	if(direction == '+')
	{
		// Go clockwise
		HAL_GPIO_WritePin(M3_ROT_GPIO_Port,M3_ROT_Pin, ROTATE_CW);
		user_set_mot_direction = ROTATE_CW;
	}
	if(direction == '-')
	{
		// Go counter-clockwise
		HAL_GPIO_WritePin(M3_ROT_GPIO_Port,M3_ROT_Pin, ROTATE_CCW);		
		user_set_mot_direction = ROTATE_CCW;
	}
}

/**
  * @brief  Function that sets the linear load direction.
  *
  * @param  direction: the direction can be set to FORWARD or BACKWARD
  * @retval This function does not return any value.
  */
static void USER_Set_Load_Dir(uint8_t direction)
{
	if(direction == '+')
	{
		// Go clockwise
		HAL_GPIO_WritePin(M3_ROT_GPIO_Port,M3_ROT_Pin, FORWARD);
		user_total_distance_performed += user_set_distance;
	}
	if(direction == '-')
	{
		// Go counter-clockwise
		HAL_GPIO_WritePin(M3_ROT_GPIO_Port,M3_ROT_Pin, BACKWARD);		
		user_total_distance_performed -= user_set_distance;
	}
}
	

/**
  * @brief  Function that clears the content of the given array.
  *
  * @param  array: the address of the array to be cleared
  * @retval This function does not return any value.
  */
static void USER_Clear_Array(uint8_t* array)
{
	uint8_t i;
	for(i=0; i< strlen((const char*)array); i++)
	{
		array[i] = 0x00;
	}
}

	

/**
*/
static void USER_Compute_Pwm_Period(double motor_RPM)
{
	// If we set an angular speed grater to the one specific to motor rezolution then we neglect it
		if(motor_RPM > user_mot_max_RPM)
			motor_RPM = user_mot_max_RPM;
		/* Update the prescaler or/and the period to obtain the desired motor RPM */
		// We firstly compute the time needed for one mechanical rotation
		user_one_rotation_time = MINUTE_TO_US/motor_RPM;

		// Now we compute the period in increments of 125 ns.
		user_pwm_period = (uint32_t)((user_one_rotation_time*8)/user_set_mot_rez);
		
		// If the period is grater than the max value taht can be assigned to the TIM->ARR register we need to change the prescaler
		if(user_pwm_period < 65535)
		{
			// Change the prescaler to have an increment of 5us for each count
			htim1.Instance->PSC = 8;	// Using this value, we manage to obtain the minimum time increment, which will be ((8+1)*1s) / 72000000 MHz = 125 ns
			
			// The real result is down-rounded, so we need to assure that in case of MAX vel, the period will be set to a safe value
			// and that means to go up (PWM frequency will go down)
			if(((user_one_rotation_time*8)/user_set_mot_rez) > user_pwm_period)
			{
				user_pwm_period++;
			}
		}
		else
		{
			// Change the prescaler
			htim1.Instance->PSC = 359; // Using this value, we manage to obtain an increment of 5us ((359+1)*1s) / 72000000 MHz = 5 us	
			// Change the prescaler to have an increment of 5us for each count
			// The real result is down-rounded, so we need to assure that in case of MAX vel, the period will be set to a safe value
			// and that means to go up (PWM frequency will go down)
			user_pwm_period /= 40;
			if(((user_one_rotation_time*8)/user_set_mot_rez)/40 > user_pwm_period)
			{
				user_pwm_period++;
			}
		}
		
		// The final value for the period of the PWM signal is obtained here and we need to update the TIM1 registers
		// in order to have the right frequency and duty cycle
		htim1.Instance->ARR = user_pwm_period-1;
		htim1.Instance->CCR3 = user_pwm_period/2;
		sprintf((char*)user_sendUSBdata,"Prescaler: %0d\rPWM period, pulse: %0d, %0d\rRCR value: %0d\r",htim1.Instance->PSC, user_pwm_period, user_pwm_period/2,htim1.Instance->RCR);
		CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
		// We know the period. Now we can compute the frequency and update the timer 1 registers.
		// Last update: 22.11.2018. The pwm frequency might be redundant to be calculated
		//user_pwm_frequency = (uint32_t)((1/user_pwm_period)* 1000000);
}

/**
*/
void USER_Start_Motor(void)
{
	/* Nu pot sa verific aceasta functie in modul DEBUG, pentru ca apare un conflict cu aplicatia TERMINAL.EXE pe care o folosesc sa trimit mesaje!!!!!! */
	// MESAJUL DE MAI SUS VA TREBUI SA FIE STERS DUPA REZOLVARE!
	if((user_flag_steps_to_be_performed_SET || user_flag_distance_to_be_performed_SET) &&
		(user_flag_mot_RPM_SET || user_flag_load_vel_SET))
	{
		//CDC_Transmit_FS((uint8_t*)"#EOK\r",5);
		// We firstly need to enable interrupts to update the PRESCALER, PERIOD AND REPETITTION COUNTER resgister. So, we set the correspondig BIT of TIM1 EGR 
		htim1.Instance->EGR = TIM_EGR_UG;
	
		__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);
		
			//Enable the motor
		HAL_GPIO_WritePin(M3__EN_GPIO_Port,M3__EN_Pin,ON);
		
		// Start PWM to perform steps. 
		// !!! A macro will be used to replace "TIM_CHANNEL_1" from above.
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		// After the interrupt is fired and the registers are all set, the next waiting while statement will become FALSE
		// and the motor start procedure contunies
		//while(!user_flag_TIMER_registers_update_SET);
		//CDC_Transmit_FS((uint8_t*)"6",1);
		
		
		
	}
}

void USER_Stop_Motor(void)
{
	//Disable the motor
	HAL_GPIO_WritePin(M3__EN_GPIO_Port,M3__EN_Pin,OFF);
}

/**
  * @brief  This function checks the received message, sent by the USER over USB.
  *
  * @param  This function does not take any arguments.
  * @retval This function does not return any value.
  */
static uint8_t USER_Decode_Msg(void)
{
	uint16_t pos = 0, i;
	uint32_t local_drecvUSBdata = 0;
	//CDC_Transmit_FS(&rcvUSBdata[pos],1);
	// Before decoding the message, we clear the array of sending char to be updated
	USER_Clear_Array(user_sendUSBdata);
	if(user_rcvUSBdata[pos++] == ASCII_START_CH)
	{
		if(user_rcvUSBdata[pos] == 'M')
		{
			//CDC_Transmit_FS((uint8_t*)"#EOK3\r",6);
			if(user_rcvUSBdata[++pos] == user_motNr)
			{
				//CDC_Transmit_FS((uint8_t*)"#EOK4\r",6);
				switch(user_rcvUSBdata[++pos])
				{
					case 'S':
						if(user_rcvUSBdata[++pos] == 'T')
						{
							if(user_rcvUSBdata[++pos] == 'A')
							{
								if(user_rcvUSBdata[++pos] == 'R')
									if(user_rcvUSBdata[++pos] == 'T')
										if(user_rcvUSBdata[++pos] == '\r')
										{
											//CDC_Transmit_FS((uint8_t*)"#EOK\r",5);
											USER_Start_Motor();
											return TRUE;
										}
										
							}
							if(user_rcvUSBdata[++pos] == 'O')
							{
								if(user_rcvUSBdata[++pos] == 'P')
									if(user_rcvUSBdata[++pos] == '\r')
									{
										USER_Stop_Motor();
										return TRUE;
									}
							}
						}
						break;

					/* ---------------------------------------------	Decode configuration messages --------------------------------------------------------*/
					case SET_STEP_RESOLUTION:
						// Message parameters verification
						if(user_rcvUSBdata[(strlen((const char*)user_rcvUSBdata)-1)]== ASCII_END_CH)
						{
							// If the number of the received characters is the expected one
							//< For the moment we only check that the last char is the ASCII_END_CH. Thus this does not confirm that the length of the message is the right one.....>
							if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,4,6))
							{
								// It seems that doing strlen to a uint32_t array does not work. So we directly inserted the length of the array
								//for( i= 0; i<strlen(const char*)globalStepRes;i++)
								user_set_mot_rez = 0;
								for( i= 0; i<9;i++)
								{
									if(user_globalStepRes[i] == local_drecvUSBdata)
									{
										user_set_mot_rez = local_drecvUSBdata;
										// If we know the rezolution, then we can compute what will be the minimum time for one mechanical rotation in us
										user_one_rotation_time = user_set_mot_rez * user_pwm_min_period[i];
										// Now we can find out what is the maximum RPM in this mode
										user_mot_max_RPM = (double)MINUTE_TO_US/user_one_rotation_time;
										// If the know the screw step, then we can compute the maximum linear vel of the load
										if(user_set_screw_step > 0)
										{
											user_load_max_vel = (user_mot_max_RPM*user_set_screw_step)/MINUTE_TO_SECOND;
										}
									}
								}
								sprintf((char*)user_sendUSBdata,"Motor max RPM: %0.2f\r",user_mot_max_RPM);
								CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
								return TRUE;
							}
						}
						break;
						
					case SET_AXIS_LEN:
						// Message parameters verification
						if(user_rcvUSBdata[(strlen((const char*)user_rcvUSBdata)-1)]== ASCII_END_CH)
							{
								// If the number of the received characters is the expected one
								//< For the moment we only check that the last char is the ASCII_END_CH. Thus this does not confirm that the length of the message is the right one.....>
								if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,4,4))
								{
									//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
									user_set_axis_len = local_drecvUSBdata;
								}
								sprintf((char*)user_sendUSBdata,"Axis length: %d\r",user_set_axis_len);
								CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
								return TRUE;
							}
						break;
						
					case SET_SCREW_STEP:
						// Message parameters verification
						if(user_rcvUSBdata[(strlen((const char*)user_rcvUSBdata)-1)]== ASCII_END_CH)
						{
							// If the number of the received characters is the expected one
							//< For the moment we only check that the last char is the ASCII_END_CH. Thus this does not confirm that the length of the message is the right one.....>
							if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,4,2))
							{
								//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);				
								user_set_screw_step = local_drecvUSBdata;
								// If the know the motor rezolution, we know the maximum motor RPM and we can compute the maximum linear vel of the load
								if(user_mot_max_RPM > 0)
								{
									user_load_max_vel = (user_mot_max_RPM*user_set_screw_step)/MINUTE_TO_SECOND;
								}
								sprintf((char*)user_sendUSBdata,"Load max linear speed: %0.2f\r",user_load_max_vel);
								CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
								return TRUE;
							}
						}
						break;
						
					case SET_LOAD_MASS:
						// Message parameters verification
						if(user_rcvUSBdata[(strlen((const char*)user_rcvUSBdata)-1)]== ASCII_END_CH)
						{
							// If the number of the received characters is the expected one
							//< For the moment we only check that the last char is the ASCII_END_CH. Thus this does not confirm that the length of the message is the right one.....>
							if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,4,4))
							{
								//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);								
								user_set_load_mass = local_drecvUSBdata;
								sprintf((char*)user_sendUSBdata,"Load mass: %d\r",user_set_load_mass);
								CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
								return TRUE;
							}
						}
						break;
						
						/* ---------------------------------------------	Decode command messages --------------------------------------------------------*/
					case STEPS_TO_BE_PERFORMED:
						// Message parameters verification
						if(user_rcvUSBdata[(strlen((const char*)user_rcvUSBdata)-1)]== ASCII_END_CH)
						{
							// If the number of the received characters is the expected one
							//< For the moment we only check that the last char is the ASCII_END_CH. Thus this does not confirm that the length of the message is the right one.....>
							if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,5,9))
							{
								//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);								
								user_set_steps = local_drecvUSBdata;
								// If the number of steps is greater than 0, take action, otherwise, do nothing
								if(user_set_steps>0)
								{
									// We need to update the compute for the value of the RCR in order to perform the desired nr of steps when the motor will be turned ON
									user_number_of_RCR_overflows = user_set_steps / (uint32_t)MAX_RCR_VAL;
									user_remaining_RCR_val = user_set_steps % (uint32_t)MAX_RCR_VAL;
									//If "user_number_of_RCR_overflows" variable is different from 0, we set the RCR to its max value.
									// We therefore decrement "user_number_of_RCR_overflows" until it reaches 0
									if(user_number_of_RCR_overflows>0)
									{
										htim1.Instance->RCR = MAX_RCR_VAL-1;										
									}
									// Otherwise, we check to see if the "user_remaining_RCR_val" variable is different from zero and if true
									// we set the RCR to this value. Therefore we will have less than MAX_RCR_VAL steps to perform.
									else
									{
										if(user_remaining_RCR_val>0)
										{
											htim1.Instance->RCR = user_remaining_RCR_val-1;
										}
										else
										{
											// If this point is reached, there is an error in the code!
											//CDC_Transmit_FS((uint8_t*)"#EOK\r",5);
											return FALSE;
										}
									}
									// We make the flag true for the moment when the USER will hit the START button
									user_flag_steps_to_be_performed_SET = TRUE;
									sprintf((char*)user_sendUSBdata,"No. of RCR overflows/remain: %d/%d\r",user_number_of_RCR_overflows,user_remaining_RCR_val);
									CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
								}
							}
							// We set the motor direction
							USER_Set_Mot_Dir(user_rcvUSBdata[4]);
							return TRUE;
						}
						break;
					
					case DISTANCE_TO_BE_PERFORMED:
						/* ------------------------- LAST UPDATE: 24.11.2018 ----------------------------------------*/
						// WE NEED DO INCREASE PRECISION FROM 2 ---> 4 DIGITS AFTER POINT
						if((user_set_screw_step) && (user_set_mot_rez))
						{
							// Message parameters verification
							if(user_rcvUSBdata[(strlen((const char*)user_rcvUSBdata)-1)]== ASCII_END_CH)
							{
								// If the number of the received characters is the expected one 
								//< For the moment we only check that the last char is the ASCII_END_CH. Thus this does not confirm that the length of the message is the right one.....>
								if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,5,3))
								{						
									Iuser_set_distance = local_drecvUSBdata;
								}
								// Reset the variable to receive a new value
								local_drecvUSBdata = 0;
								if(user_rcvUSBdata[8] == ',')
								{
									if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,9,2))
									{
										Fuser_set_distance = local_drecvUSBdata;
										user_set_distance = (double)Fuser_set_distance/100 + Iuser_set_distance;
										// If there is a value different from 0 for the desired distance to be performed, we take action, otherwise do nothing
										if(user_set_distance>0.0)
										{
											// Here we must convert the distance to be covered in motor steps based on the screw step and rezolution value
											user_mot_revolutions = user_set_distance/user_set_screw_step;
											user_set_steps = (uint32_t)((user_mot_revolutions*STEPS_FOR_1_MECHANICAL_ROTATION)/ 
																				(STEPS_FOR_1_MECHANICAL_ROTATION / user_set_mot_rez));
											
											user_flag_distance_to_be_performed_SET = TRUE;
											sprintf((char*)user_sendUSBdata,"Resulting nr of steps: %0d\r",user_set_steps);
											CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
										}
									}
								}
								// We set the motor direction
								USER_Set_Load_Dir(user_rcvUSBdata[4]);
								return TRUE;
							}
						}							
						break;
					
					case MOT_ANGULAR_VEL:
						if(user_set_mot_rez)
						{
							// Message parameters verification
							if(user_rcvUSBdata[(strlen((const char*)user_rcvUSBdata)-1)]== ASCII_END_CH)
							{
								// If the number of the received characters is the expected one 
								//< For the moment we only check that the last char is the ASCII_END_CH. Thus this does not confirm that the length of the message is the right one.....>
								if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,4,4))
								{
									//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);								
									Iuser_set_motor_RPM = local_drecvUSBdata;
									//sprintf((char*)sendUSBdata,"Numar pasi: %d\r",USER_Set_Steps);
									//CDC_Transmit_FS(sendUSBdata,strlen((const char*)sendUSBdata));
									
								}
								// Reset the variable to receive a new value
								local_drecvUSBdata = 0;
								if(user_rcvUSBdata[8] == ',')
								{
									if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,9,2))
									{
										Fuser_set_motor_RPM = local_drecvUSBdata;
										user_set_motor_RPM = (double)Fuser_set_motor_RPM/100 + Iuser_set_motor_RPM;
										// If we got a value different from 0 take action, otherwise do nothing
										if(user_set_motor_RPM>0)
										{
											USER_Compute_Pwm_Period(user_set_motor_RPM);
											//sprintf((char*)user_sendUSBdata,"Motor RPM: %0.2f\r",user_set_motor_RPM);
											//CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
											user_flag_mot_RPM_SET = TRUE;
											return TRUE;
										}
									}
								}
							}
						}
						break;
					
					case MASS_LINEAR_VEL:
						/* ------------------------- LAST UPDATE: 24.11.2018 ----------------------------------------*/
						// WE NEED DO INCREASE PRECISION FROM 2 ---> 4 DIGITS AFTER POINT
						if((user_set_mot_rez) && (user_set_screw_step))
						{
							// Message parameters verification
							if(user_rcvUSBdata[(strlen((const char*)user_rcvUSBdata)-1)]== ASCII_END_CH)
							{
								// If the number of the received characters is the expected one 
								//< For the moment we only check that the last char is the ASCII_END_CH. Thus this does not confirm that the length of the message is the right one.....>
								if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,4,3))
								{
									//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);								
									Iuser_set_load_vel = local_drecvUSBdata;
									//sprintf((char*)sendUSBdata,"Numar pasi: %d\r",USER_Set_Steps);
									//CDC_Transmit_FS(sendUSBdata,strlen((const char*)sendUSBdata));
									
								}
								// Reset the variable to receive a new value
								local_drecvUSBdata = 0;
								if(user_rcvUSBdata[7] == ',')
								{
									if(USER_ExtractNumber_FromString(&local_drecvUSBdata,user_rcvUSBdata,8,2))
									{
										Fuser_set_load_vel = local_drecvUSBdata;
										user_set_load_vel = (double)Fuser_set_load_vel/100 + Iuser_set_load_vel;
										// If we got a value different from 0 take action, otherwise do nothing
										if(user_set_load_vel>0)
										{
											// Now we can compute the required frequency of the pwm signal based on the load speed and the screw step
											user_set_motor_RPM = (user_set_load_vel * MINUTE_TO_SECOND)/user_set_screw_step;
											USER_Compute_Pwm_Period(user_set_motor_RPM);
											user_flag_load_vel_SET = TRUE;
											//sprintf((char*)user_sendUSBdata,"Load linear vel: %0.2f[mm/s]\r",user_set_load_vel);
											//CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
										}
										return TRUE;
									}
								}
							}
						}
						break;
				}
			}
		}
		if(user_rcvUSBdata[pos] == 'P')
		{
			if(user_rcvUSBdata[++pos] == 'N')
			{
				if(user_rcvUSBdata[++pos] == 'G')
				{
					if(user_rcvUSBdata[++pos] == ASCII_END_CH)
					{
						CDC_Transmit_FS((uint8_t*)"#EOK\r",5);
						return TRUE;
					}						
				}
			}
		 }
	}
	return FALSE;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;	// Using this value, we manage to obtain the minimum time increment, which will be 8s / 72000000 MHz = 125 ns
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 15;	// The period will be modified by the user based on the PWM characteristics for the motor control
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;	// The repetition counter will be also modified by the user based on the number of steps to be performed
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (htim1.Init.Period+1)/2; // To obtain 50% duty cycle
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_4;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
