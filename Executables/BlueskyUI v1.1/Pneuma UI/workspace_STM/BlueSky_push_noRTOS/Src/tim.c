/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "hal_board_cfg.h"
#include "math.h"

#define TIM1_PERIOD_VALUE
#define TIM3_PERIOD_VALUE               678           // PWM frequency = 80 000 000/678

// Variable
uint32_t tim3_per_value = 678;

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef Tim1Handle;
TIM_HandleTypeDef Tim3Handle;

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  // Interrupt duration = 80 000 000/80/1000 = 1000Hz
  Tim1Handle.Instance = TIM1;
  Tim1Handle.Init.Prescaler = 80 - 1;
  Tim1Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  Tim1Handle.Init.Period = 1000 - 1;
  Tim1Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  Tim1Handle.Init.RepetitionCounter = 0;
  Tim1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&Tim1Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&Tim1Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&Tim1Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  uint32_t dutyCycle;
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
#if PROG_TYPE == 16
  dutyCycle = round(tim3_per_value * ( (float) duty / 100));
#else
  dutyCycle = tim3_per_value >> 1;
#endif
  Tim3Handle.Instance = TIM3;
  Tim3Handle.Init.Prescaler = 0;
  Tim3Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  Tim3Handle.Init.Period = tim3_per_value;                    // MY TIM3_PERIOD_VALUE;
  Tim3Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  Tim3Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&Tim3Handle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&Tim3Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = dutyCycle;                 // MY  TIM3_PERIOD_VALUE >> 1
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = dutyCycle;                 // MY  TIM3_PERIOD_VALUE >> 1
  if (HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&Tim3Handle);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
	HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    */
    GPIO_InitStruct.Pin = PWM_A_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(PWM_A_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_MspPosDeInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM3)
  {
      /* USER CODE BEGIN TIM3_MspPostInit 0 */

      /* USER CODE END TIM3_MspPostInit 0 */

      __HAL_RCC_GPIOA_CLK_ENABLE();
      /**TIM3 GPIO Configuration
      PA6     ------> TIM3_CH1
      PA7     ------> TIM3_CH2 */
      GPIO_InitStruct.Pin = PWM_A_PIN;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(PWM_A_PORT, &GPIO_InitStruct);

      GPIO_setOutputLowOnPin( PWM_A_PORT, PWM_A_PIN );

      /* USER CODE BEGIN TIM3_MspPostInit 1 */

      /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
	  /* USER CODE BEGIN TIM1_MspInit 0 */

	  /* USER CODE END TIM1_MspInit 0 */
	    /* TIM1 clock enable */
	    __HAL_RCC_TIM1_CLK_ENABLE();

	    /* TIM1 interrupt Init */
	    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 5, 0);
	    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
	  /* USER CODE BEGIN TIM1_MspInit 1 */

	  /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
} 

uint8_t HAL_TIM3_PWM_Start( uint32_t pwm_ch )
{
	uint8_t rslt = true;

	if ( pwm_ch == TIM_CHANNEL_1 ){
		HAL_TIM_MspPostInit(&Tim3Handle);
		if (HAL_TIM_PWM_Start(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
		{
		  /* PWM Generation Error */
			rslt = false;
		}
	}
	else if ( pwm_ch == TIM_CHANNEL_2 ){
		if (HAL_TIM_PWM_Start(&Tim3Handle, TIM_CHANNEL_2) != HAL_OK)
		{
		  /* PWM Generation Error */
			rslt = false;
		}
	}
	else {
		rslt = false;
	}

	return rslt;
}

uint8_t HAL_TIM3_PWM_Stop( uint32_t pwm_ch )
{
	uint8_t rslt = true;

	if ( pwm_ch == TIM_CHANNEL_1){
		if (HAL_TIM_PWM_Stop(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
		{
		  /* PWM Generation Error */
			rslt = false;
		}
		HAL_TIM_MspPosDeInit(&Tim3Handle);
	}
	else if ( pwm_ch == TIM_CHANNEL_2){
		if (HAL_TIM_PWM_Stop(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
		{
		  /* PWM Generation Error */
			rslt = false;
		}
	}
	else {
		rslt = false;
	}

	return rslt;

}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
