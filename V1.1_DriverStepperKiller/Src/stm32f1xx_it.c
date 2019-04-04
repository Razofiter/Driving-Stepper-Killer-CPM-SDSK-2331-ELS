/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim1;
extern volatile uint8_t user_flag_TIMER_registers_update_SET;
extern uint32_t user_number_of_RCR_overflows;
extern uint32_t user_number_of_RCR_overflows;
extern uint8_t user_remaining_RCR_val;
extern uint32_t user_set_steps;
extern uint8_t user_set_mot_direction;
extern int32_t user_total_steps_performed;



/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
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
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
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
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
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
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
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
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles USB high priority or CAN TX interrupts.
*/
void USB_HP_CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 0 */

  /* USER CODE END USB_HP_CAN1_TX_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 1 */

  /* USER CODE END USB_HP_CAN1_TX_IRQn 1 */
}

/**
* @brief This function handles USB low priority or CAN RX0 interrupts.
*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles TIM1 break interrupt.
*/
//void TIM1_BRK_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM1_BRK_IRQn 0 */

//  /* USER CODE END TIM1_BRK_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim1);
//	//CDC_Transmit_FS((uint8_t*)"#BRK\r",5);
//  /* USER CODE BEGIN TIM1_BRK_IRQn 1 */

//  /* USER CODE END TIM1_BRK_IRQn 1 */
//}

/**
* @brief This function handles TIM1 update interrupt.
*/
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	// If we want an update the above flag is FALSE before executing the following line
		if(!user_flag_TIMER_registers_update_SET)
		{
			user_flag_TIMER_registers_update_SET = TRUE;
			//CDC_Transmit_FS((uint8_t*)"1",1);
		}
		// Else, we have a RCR overflow event
		else
		{
			// Here we account for the number RCR overflows
			if(user_number_of_RCR_overflows)
			{
				//CDC_Transmit_FS((uint8_t*)"#2",1);
				user_number_of_RCR_overflows--;
				/* Enable the Peripheral */
				__HAL_TIM_ENABLE(&htim1);
			}
			else
			{
				if(user_remaining_RCR_val)
				{
					if(user_set_steps>MAX_RCR_VAL)
					{
						//CDC_Transmit_FS((uint8_t*)"#3",1);
						// Because we need to change the value of RCR, an update of TIM1 registres is needed, therefore we change user_flag_TIMER_registers_update_SET to FALSE
						// and next time the interrupt is fired, we enter the first IF STATEMENT.
						htim1.Instance->RCR = user_remaining_RCR_val;
						htim1.Instance->EGR = TIM_EGR_UG;
						user_flag_TIMER_registers_update_SET = FALSE;
						user_remaining_RCR_val = 0;
						/* Enable the Peripheral */
						__HAL_TIM_ENABLE(&htim1);
					}
					else
					{
						//CDC_Transmit_FS((uint8_t*)"4",1);
						user_remaining_RCR_val = 0;
						/* Enable the Peripheral */
						//__HAL_TIM_ENABLE(&htim1);
					}
				}
				else
				{
					//CDC_Transmit_FS((uint8_t*)"5",1);
					// Change the update TIM1 registers flag back to FALSE for the next motor START procedure
					user_flag_TIMER_registers_update_SET = FALSE;
					// If this point is reached, that means we finished to perform the number of steps. Therefore, we stop the PWM generation
					__HAL_TIM_DISABLE(&htim1);
					// We finally update the total number of steps that were performed based on the direction
					if(user_set_mot_direction == ROTATE_CW)
						user_total_steps_performed += user_set_steps;
					else
						user_total_steps_performed -= user_set_steps;
					
					//sprintf((char*)user_sendUSBdata,"Current nr of steps: %0d\r", user_total_steps_performed);
					//CDC_Transmit_FS(user_sendUSBdata,strlen((const char*)user_sendUSBdata));
				}
			}
		}
	//CDC_Transmit_FS((uint8_t*)"#UPD\r",5);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
* @brief This function handles TIM1 capture compare interrupt.
*/
//void TIM1_CC_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
//	//CDC_Transmit_FS((uint8_t*)"#CCI\r",5);
//  /* USER CODE END TIM1_CC_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim1);
//	
//  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

//  /* USER CODE END TIM1_CC_IRQn 1 */
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
