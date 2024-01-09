/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#include "stm32l452xx.h"
#include "adc.h"
#include "hal_board_cfg.h"

extern uint32_t adc_current_ch;

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef Adc1Handle;

/* ADC1 init function */
#if ADC_NO_DMA
void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	// Common config
	adc1Handle.Instance = ADC1;
	adc1Handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	adc1Handle.Init.Resolution = ADC_RESOLUTION_12B;
	adc1Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adc1Handle.Init.ScanConvMode = ADC_SCAN_DISABLE;
	adc1Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	adc1Handle.Init.LowPowerAutoWait = DISABLE;
	adc1Handle.Init.ContinuousConvMode = DISABLE;
	adc1Handle.Init.NbrOfConversion = 1;
	adc1Handle.Init.DiscontinuousConvMode = DISABLE;
	adc1Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	adc1Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	adc1Handle.Init.DMAContinuousRequests = DISABLE;
	adc1Handle.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	adc1Handle.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&adc1Handle) != HAL_OK)
	{
		Error_Handler();
	}
	// Configure Regular Channel
	sConfig.Channel = ADC_CHANNEL_16;               // ADC_CHANNEL_16;  adc_current_ch  //TT
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&adc1Handle, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
#else
void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	  /* ### - 1 - Initialize ADC peripheral #################################### */
	  Adc1Handle.Instance          = ADC1;
	  if (HAL_ADC_DeInit(&Adc1Handle) != HAL_OK){
		  // ADC de-initialization Error
		  Error_Handler();
	  }

	  Adc1Handle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;      /* Synchronous clock mode, input ADC clock divided by 2*/
	  Adc1Handle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
	  Adc1Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
	  Adc1Handle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	  Adc1Handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
	  Adc1Handle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
	  Adc1Handle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
	  Adc1Handle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
	  Adc1Handle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
	  Adc1Handle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
	  Adc1Handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
	  Adc1Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
	  Adc1Handle.Init.DMAContinuousRequests = ENABLE;                        /* ADC DMA continuous request to match with DMA circular mode */
	  Adc1Handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
	  Adc1Handle.Init.OversamplingMode      = DISABLE;                       /* No oversampling */
	  /* Initialize ADC peripheral according to the passed parameters */
	  if (HAL_ADC_Init(&Adc1Handle) != HAL_OK){
	    Error_Handler();
	  }

	  /* ### - 2 - Start calibration ############################################ */
	  if (HAL_ADCEx_Calibration_Start(&Adc1Handle, ADC_SINGLE_ENDED) !=  HAL_OK){
		  Error_Handler();
	  }

	  /* ### - 3 - Channel configuration ######################################## */
	  sConfig.Channel      = adc_current_ch;        // adc_current_ch       /* Sampled channel number */
	  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
	  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;  //ADC_SAMPLETIME_6CYCLES_5;   /* Sampling time (number of clock cycles unit) */
	  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
	  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
	  if (HAL_ADC_ConfigChannel(&Adc1Handle, &sConfig) != HAL_OK){
		  Error_Handler();
	  }
}
#endif

#if ADC_NO_DMA
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration    
    PB0     ------> ADC1_IN15
    PB1     ------> ADC1_IN16 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}
#else
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  static DMA_HandleTypeDef         DmaHandle;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* ADC Periph clock enable */
  __HAL_RCC_ADC_CLK_ENABLE();
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
  /* Enable DMA clock */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /*##- 2- Configure peripheral GPIO #########################################*/
  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = ADC_CUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init( ADC_CUT_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ADC_BAT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init( ADC_BAT_PORT, &GPIO_InitStruct);

  /*##- 3- Configure DMA #####################################################*/

  /*********************** Configure DMA parameters ***************************/
  DmaHandle.Instance                 = DMA1_Channel1;
  DmaHandle.Init.Request             = DMA_REQUEST_0;
  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  DmaHandle.Init.Mode                = DMA_NORMAL;                    // 0814   DF=DMA_CIRCULAR
  DmaHandle.Init.Priority            = DMA_PRIORITY_MEDIUM;
  /* Deinitialize  & Initialize the DMA for new transfer */
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);

  /* Associate the DMA handle */
  __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

  /* NVIC configuration for DMA Input data interrupt */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}
#endif

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration */
    HAL_GPIO_DeInit( ADC_CUT_PORT, ADC_CUT_PIN );
    HAL_GPIO_DeInit( ADC_BAT_PORT, ADC_BAT_PIN );

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
