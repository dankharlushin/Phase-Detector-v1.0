/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_it.h"
#include "wizchip_init.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef enum{
	Axis_X,
	Axis_Y,
	Axis_Z,
	Axis_A,
	AXIS_COUNT
}Axis;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AVG 100
#define TRANSPORT_BUFFER_SIZE 4
#define PORT 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */


volatile uint8_t FlagTim3IT = 0;

uint8_t 	Error = 0,
			Count = 0;

uint16_t	Period = 0,
			dPhase = 0;

uint16_t PeriodAVG[AVG] = {0};
uint8_t iX = 0,
		iY = 0,
		iZ = 0,
		iA = 0;

uint32_t sumX = 0,
		 sumY = 0,
		 sumZ = 0,
		 sumA = 0;

uint16_t PeriodX = 0,
		 PeriodY = 0,
		 PeriodZ = 0,
		 PeriodA = 0;
uint32_t count = 0;


uint16_t Phase[2][AXIS_COUNT] = {0};
int PhaseRelatA[AXIS_COUNT] = {0};
uint8_t var = 86;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
int RefreshTIM(TIM_HandleTypeDef *htim);
void PhaseRelativelyA(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
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
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */
  Error = 0x01; //разница фаз не найдена
  RefreshTIM(&htim9);
  Count = 0;
  //RefreshTIM(&htim12);
  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */
  Error = 0x02; //Период больше 1500мкс
  RefreshTIM(&htim12);
  Count = 0;
  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}

void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);

}

void EXTI2_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);

}

void EXTI3_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);

}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
/*if(GPIO_Pin == GPIO_PIN_0){
	HAL_TIM_Base_Stop_IT(&htim12);
	Period = TIM12 ->CNT;
	PeriodAVG[ij] = Period;
	ij++;
	sum += Period;
	if(ij == AVG){
		PeriodA=sum/AVG;
		ij  = 0;
		sum = 0;
	}
	RefreshTIM(&htim12);
	HAL_TIM_Base_Start_IT(&htim12);
}*/
	/*int32_t startFlag = 0;
	uint8_t buffer[32];

	if((ret = loopback_tcps_server(0, gDATABUF, 5000)) < 0) {
		UART_Printf("SOCKET ERROR : %ld\r\n", ret);
	}*/

	//else {

	//if (count < 50000) {
		dPhase = 0;
		switch (Count)
		{
		case 0:
			RefreshTIM(&htim9);
			HAL_TIM_Base_Start_IT(&htim9);
			Count++;
			break;

		case AXIS_COUNT-1:
		dPhase = TIM9 ->CNT;
		RefreshTIM(&htim9);
		Count = 0;
		PhaseRelativelyA();
		break;

		default:
			dPhase = TIM9 -> CNT;
			Count++;
			break;
		}


		switch (GPIO_Pin)
		{

		case GPIO_PIN_1:
			HAL_TIM_Base_Stop_IT(&htim12);
			Period = TIM12 ->CNT;
			PeriodAVG[iZ] = Period;
			iZ++;
			sumZ += Period;
			if(iZ == AVG){
				PeriodZ = sumZ/AVG;
				iZ  = 0;
				sumZ = 0;
			}
			RefreshTIM(&htim12);
			HAL_TIM_Base_Start_IT(&htim12);
			if (Count == 0)
			{
				Phase[0][Axis_Z] = 4;
				Phase[1][Axis_Z] = dPhase;
				sendDataByWiznet(Phase[1][Axis_Z]);
				//buffer[0] = Phase[1][Axis_Z];
				//loopback_tcps_server(0, buffer, 5000);

			}
			else
			{
				Phase[0][Axis_Z] = Count;
				Phase[1][Axis_Z] = dPhase;
				sendDataByWiznet(Phase[1][Axis_Z]);
			}
			break;

		case GPIO_PIN_0:
			HAL_TIM_Base_Stop_IT(&htim2);
			Period = TIM2 ->CNT;
			PeriodAVG[iY] = Period;
			iY++;
			sumY += Period;
			if(iY == AVG){
				PeriodY = sumY/AVG;
				iY  = 0;
				sumY = 0;
			}
			RefreshTIM(&htim2);
			HAL_TIM_Base_Start_IT(&htim2);
			if (Count == 0)
			{
				Phase[0][Axis_Y] = 4;
				Phase[1][Axis_Y] = dPhase;
				sendDataByWiznet(Phase[1][Axis_Y]);
			}
			else
			{
				Phase[0][Axis_Y] = Count;
				Phase[1][Axis_Y] = dPhase;
				sendDataByWiznet(Phase[1][Axis_Y]);
			}
			break;

		case GPIO_PIN_2:
			HAL_TIM_Base_Stop_IT(&htim3);
			Period = TIM3 ->CNT;
			PeriodAVG[iX] = Period;
			iX++;
			sumX += Period;
			if(iX == AVG){
				PeriodX = sumX/AVG;
				iX  = 0;
				sumX = 0;
			}
			RefreshTIM(&htim3);
			HAL_TIM_Base_Start_IT(&htim3);
			if (Count == 0)
			{
				Phase[0][Axis_X] = 4;
				Phase[1][Axis_X] = dPhase;
				sendDataByWiznet(Phase[1][Axis_X]);
			}
			else
			{
				Phase[0][Axis_X] = Count;
				Phase[1][Axis_X] = dPhase;
				sendDataByWiznet(Phase[1][Axis_X]);
			};
			break;

		case GPIO_PIN_3:
			HAL_TIM_Base_Stop_IT(&htim4);
			Period = TIM4 ->CNT;
			PeriodAVG[iA] = Period;
			iA++;
			sumA += Period;
			if(iA == AVG){
				PeriodA = sumA/AVG;
				iA  = 0;
				sumA = 0;
			}
			RefreshTIM(&htim4);
			HAL_TIM_Base_Start_IT(&htim4);
			if (Count == 0)
			{
				Phase[0][Axis_A] = 4;
				Phase[1][Axis_A] = dPhase;
				sendDataByWiznet(Phase[1][Axis_A]);
			}
			else
			{
				Phase[0][Axis_A] = Count;
				Phase[1][Axis_A] = dPhase;
				sendDataByWiznet(Phase[1][Axis_A]);
			};
			break;

		default:
			break;
		}
	//}
	//count++;
	//}
}

int RefreshTIM(TIM_HandleTypeDef *htim){
	HAL_TIM_Base_Stop_IT(htim);
	htim->Instance->CNT = 0;
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_UPDATE);
	return 0;
}

void PhaseRelativelyA(void){
	for(uint8_t j = 0; j < AXIS_COUNT - 1; j++){
		if(Phase[0][Axis_A]>Phase[0][j]){
			PhaseRelatA[j]=Phase[1][j]-Phase[1][Axis_A];
		}
		else
		{
			PhaseRelatA[j]=Phase[1][j];
		}
	}
}

void sendDataByWiznet(uint16_t data) {

	uint8_t buffer[TRANSPORT_BUFFER_SIZE];

	uint8_t highByteOfData = data >> 8;
	uint8_t lowByteOfData =  data & 0xFF;
	buffer[0] = 10;
	buffer[1] = highByteOfData;
	buffer[2] = lowByteOfData;
	loopback_tcps_server(0, buffer, PORT);


}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
