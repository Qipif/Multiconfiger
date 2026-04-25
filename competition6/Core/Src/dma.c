/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  // ADC DMA优先级设低(6)，不抢占I2C(默认5)
  // DAC DMA保持较低优先级
  /* DMA1_Stream0_IRQn interrupt configuration (ADC1) */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration (DAC1_CH1) */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration (ADC2) */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration (DAC1_CH2) */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration (ADC3) */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

