/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "string.h"
#include "usart.h"
#include "hal_board_cfg.h"


#define TXBUFFERSIZE                100
#define RXBUFFERSIZE                200

#define UART_TASK_IDLE              0x00
#define UART_TASK_TX                0x01
#define UART_TASK_RX                0x02
//------------------------------------------------------------------------------

ITStatus UartReady = RESET;
uint8_t aTxBuffer[TXBUFFERSIZE];
uint8_t aRxBuffer[RXBUFFERSIZE];

static void UART_RX_ISR( struct __UART_HandleTypeDef *huart );
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef Uart1Handle;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  Uart1Handle.Instance = USART1;
  Uart1Handle.Init.BaudRate = 9600;
  Uart1Handle.Init.WordLength = UART_WORDLENGTH_8B;
  Uart1Handle.Init.StopBits = UART_STOPBITS_1;
  Uart1Handle.Init.Parity = UART_PARITY_NONE;
  Uart1Handle.Init.Mode = UART_MODE_TX_RX;
  Uart1Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  Uart1Handle.Init.OverSampling = UART_OVERSAMPLING_16;
  Uart1Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  Uart1Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  // MY
  //Uart1Handle.RxISR = UART_RX_ISR;

  if (HAL_UART_Init(&Uart1Handle) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* Uart1Handle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(Uart1Handle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = UART_TXD_PIN | UART_RXD_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init( UART_PORT, &GPIO_InitStruct );

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* Uart1Handle)
{

  if(Uart1Handle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/**
  * @brief  Tx Transfer completed callback
  * @param  Uart1Handle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *Uart1Handle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;


}

/**
  * @brief  Rx Transfer completed callback
  * @param  Uart1Handle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *Uart1Handle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;
}

/**
  * @brief  UART error callbacks
  * @param  Uart1Handle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *Uart1Handle)
{
    Error_Handler();
    aRxBuffer[100] = 0xEE;
}

uint8_t HAL_UART_Send( uint8_t *data, uint8_t length )
{
	uint8_t rslt = HAL_OK;

	if ( length > TXBUFFERSIZE ){
		length = TXBUFFERSIZE;
	}
	memcpy( data, (uint8_t*)aTxBuffer, length );
	// Start the transmission process, While the UART in reception process, user can transmit data through "aTxBuffer" buffer
	if( HAL_UART_Transmit_IT(&Uart1Handle, (uint8_t*)aTxBuffer, length)!= HAL_OK ){
		rslt = HAL_ERROR;
		Error_Handler();
	}

	// Wait for the end of the transfer
	while (UartReady != SET){
	}
	UartReady = RESET;

	return rslt;
}

uint8_t HAL_UART_Read( uint8_t *data, uint8_t length )
{

	if ( length > RXBUFFERSIZE ){
		length = RXBUFFERSIZE;
	}
	if(HAL_UART_Receive_IT(&Uart1Handle, (uint8_t *)aRxBuffer, length) != HAL_OK){
		//Error_Handler();
		return HAL_ERROR;
	}
	// Wait for the end of the transfer
	while (UartReady != SET){

	}
	// Reset transmission flag
	UartReady = RESET;

	return HAL_OK;
}

uint8_t uartTest( void )
{
	uint8_t rslt;
	uint16_t i;

	for ( i = 0; i < 50; i++ ){
		aTxBuffer[i] = 0xA0 + i;
	}

	rslt = HAL_UART_Send( aTxBuffer, 50 );

	if ( rslt == HAL_OK ){
		rslt = HAL_UART_Read( aTxBuffer, 8 );
	}

	return rslt;
}


/*
static void UART_RX_ISR( struct __UART_HandleTypeDef *huart )
{
	int8 rcvData;

	if ( Uart1Handle.Instance == USART1 ){
	    rcvData = USCI_A_UART_receiveData(USCI_A0_BASE);
	    if ( dmaCfg.rxTail < HAL_UART_DMA_RX_MAX ){
	        *(dmaCfg.rxBuf + dmaCfg.rxTail) = rcvData;
	        dmaCfg.rxTail++;
	    }
	}
}
*/
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
